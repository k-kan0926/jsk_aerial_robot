#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NARX + MPPI controller (robust meta handling) for antagonistic McKibben actuators.
- Works even if meta.json lacks keys: order / mu / std / dt
- Avoids deque mutated during iteration
- Adds safe pressure delay buffer & feature builder with automatic lag inference
- CSV logger identical column order to your logs (plus a few debug fields at end)

ROS params (with sane defaults):
  ~meta                 : path to meta.json
  ~model                : path to state_dict .pt (expects keys net.0/2/4)
  ~theta_topic          : topic name for current joint angle [rad] (Float32) or JointState
  ~theta_index          : if JointState, which index in position array is the target joint
  ~target_topic         : topic name for reference angle [deg] (Float32)
  ~pressure_topic       : topic with measured p1,p2 [MPa] (geometry_msgs/Vector3)
  ~mpa_cmd_topic        : output command p1,p2 [MPa] (geometry_msgs/Vector3)
  ~rate                 : main loop rate [Hz]
  ~frame_skip           : evaluate control every N frames (default 2 => dt ~ 2 / rate)
  ~pmax                 : max pressure [MPa]
  ~pressure_delay_s     : measurement delay [s] for pressure sensing pipeline
  ~K                    : MPPI population
  ~horizon              : MPPI horizon (steps)
  ~lambda               : MPPI temperature
  ~sigma_u              : sampling std for [dps, dpd]
  ~w_path, ~w_term, ~w_rate, ~w_z, ~w_box : cost weights
  ~cmd_gain, ~cmd_clip_max : convert MPa->raw and clip (kept for parity, not used by publisher)
  ~log_csv              : CSV output path (dir will be created)
  ~dt_override          : if >0, forces model dt
  ~feature_order        : optional explicit feature name list (overrides meta)

Notes:
- Input vector construction: if no meta['order'], assumes repeating [theta, ps, pd, dz] for L lags,
  where L = in_dim / 4, dz defaults to 0 if not measured.
- The NARX weight reader accepts the compact 3-layer MLP with tanh activations.
- If your real order differs, set ~feature_order or fix meta.json for best accuracy.

(c) 2025 Kei collaborator edition
"""
from __future__ import annotations
import os, io, sys, json, time, math, csv, threading
from collections import deque
from typing import List, Tuple, Optional

import numpy as np

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState

# -------------------------------
# Torch state_dict loader (weights -> numpy)
# -------------------------------

def load_torch_weights(pt_path: str):
    """Load a simple 3-layer MLP (net.0, net.2, net.4) into numpy arrays.
    Returns (W1, b1, W2, b2, W3, b3) with shapes:
      W1: (H, Din),  b1: (H,)
      W2: (H, H),    b2: (H,)
      W3: (Dout, H), b3: (Dout,)
    """
    try:
        import torch
    except Exception as e:
        raise RuntimeError("PyTorch is required to load the .pt state_dict: %r" % e)

    sd = torch.load(pt_path, map_location="cpu")
    # Accept OrderedDict or raw dict
    if hasattr(sd, "state_dict"):
        sd = sd.state_dict()

    # Try both flat keys and nested
    candidates = [
        ("net.0.weight", "net.0.bias", "net.2.weight", "net.2.bias", "net.4.weight", "net.4.bias"),
        ("0.weight", "0.bias", "2.weight", "2.bias", "4.weight", "4.bias"),
    ]
    keys = None
    for cand in candidates:
        if all(k in sd for k in cand):
            keys = cand
            break
    if keys is None:
        raise KeyError("State dict missing required keys. Have: %s" % list(sd.keys())[:10])

    rospy.loginfo("[NARX] matched weight keys: %s", str(keys))

    def to_np(t):
        return t.detach().cpu().numpy().astype(np.float32)

    W1 = to_np(sd[keys[0]]); b1 = to_np(sd[keys[1]])
    W2 = to_np(sd[keys[2]]); b2 = to_np(sd[keys[3]])
    W3 = to_np(sd[keys[4]]); b3 = to_np(sd[keys[5]])

    # Sanity checks
    assert W1.ndim == 2 and W2.ndim == 2 and W3.ndim == 2
    assert b1.ndim == 1 and b2.ndim == 1 and b3.ndim == 1
    assert W1.shape[0] == b1.shape[0]
    assert W2.shape[0] == b2.shape[0]
    assert W3.shape[0] == b3.shape[0]
    assert W2.shape[1] == W1.shape[0]
    assert W3.shape[1] == W2.shape[0]

    return W1, b1, W2, b2, W3, b3

# -------------------------------
# NARX (numpy) with robust meta handling
# -------------------------------
class NarxNumpy(object):
    def __init__(self, meta_path: str, pt_path: str, dt_fallback: float = 0.01, feature_order_override: Optional[List[str]] = None):
        W1,b1,W2,b2,W3,b3 = load_torch_weights(pt_path)
        self.W1,self.b1 = W1.astype(np.float32), b1.astype(np.float32)
        self.W2,self.b2 = W2.astype(np.float32), b2.astype(np.float32)
        self.W3,self.b3 = W3.astype(np.float32), b3.astype(np.float32)

        in_dim  = self.W1.shape[1]
        hid_dim = self.W1.shape[0]
        hid2    = self.W2.shape[0]
        out_dim = self.W3.shape[0]
        assert self.W2.shape[1] == hid_dim
        assert self.W3.shape[1] == hid2

        meta = {}
        try:
            with open(meta_path, "r") as f:
                meta = json.load(f) or {}
        except Exception as e:
            rospy.logwarn(f"[NARX] meta load failed: {e}  (fallback defaults will be used)")

        if feature_order_override and isinstance(feature_order_override, (list,tuple)):
            self.order = list(feature_order_override)
            rospy.loginfo("[NARX] feature order: from ROS param (~feature_order)")
        else:
            self.order = meta.get("order") or meta.get("feature_order")
            if not self.order:
                # assume 4 features (theta, ps, pd, dz) with L lags
                self.order = [f"f{i}" for i in range(in_dim)]
                rospy.logwarn(f"[NARX] 'order' missing -> synthesized {self.order[:16]}{'...' if in_dim>16 else ''}")

        mu  = meta.get("mu");  std = meta.get("std")
        if (not isinstance(mu, list)) or len(mu) != in_dim:
            rospy.logwarn(f"[NARX] 'mu' missing or wrong size -> zeros used (in_dim={in_dim})")
            mu = [0.0]*in_dim
        if (not isinstance(std, list)) or len(std) != in_dim:
            rospy.logwarn(f"[NARX] 'std' missing or wrong size -> ones used (in_dim={in_dim})")
            std = [1.0]*in_dim
        self.mu  = np.array(mu,  dtype=np.float32)
        self.std = np.array(std, dtype=np.float32)

        self.dt = float(meta.get("dt", dt_fallback))
        if "dt" not in meta:
            rospy.logwarn(f"[NARX] 'dt' missing -> fallback dt={self.dt}")

        rospy.loginfo("[NARX] order[:min]=%s", str(self.order[:min(16,len(self.order))]) + ("..." if len(self.order)>16 else ""))
        rospy.loginfo("[NARX] mu/std shapes = %s / %s", str(self.mu.shape), str(self.std.shape))
        rospy.loginfo("[NARX] W shapes = %s %s %s (out=%d)", str(self.W1.shape), str(self.W2.shape), str(self.W3.shape), out_dim)
        self._dbg_dumped = False

    def _act(self, x: np.ndarray) -> np.ndarray:
        return np.tanh(x, dtype=np.float32)

    def forward(self, x_raw_batch: np.ndarray) -> np.ndarray:
        x_raw = x_raw_batch.astype(np.float32)
        x_norm = (x_raw - self.mu) / (self.std + 1e-8)
        if not self._dbg_dumped:
            rospy.loginfo("[NARX] first x_raw[:8]=%s", np.array2string(x_raw[0,:8], precision=4))
            rospy.loginfo("[NARX] first x_norm[:8]=%s", np.array2string(x_norm[0,:8], precision=4))
            self._dbg_dumped = True
        h1 = self._act(x_norm @ self.W1.T + self.b1)
        h2 = self._act(h1    @ self.W2.T + self.b2)
        y  =           h2    @ self.W3.T + self.b3
        return y.astype(np.float32)

# -------------------------------
# Utility
# -------------------------------
def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

# Convert ps/pd -> p1/p2  (and back)
def ps_pd_to_p12(ps: float, pd: float) -> Tuple[float,float]:
    p1 = 0.5*(ps + pd)
    p2 = 0.5*(ps - pd)
    return p1, p2

def p12_to_ps_pd(p1: float, p2: float) -> Tuple[float,float]:
    return (p1 + p2), (p1 - p2)

# -------------------------------
# MPPI Node
# -------------------------------
class NarxMPPINode(object):
    def __init__(self):
        self.meta_path = rospy.get_param("~meta")
        self.pt_path   = rospy.get_param("~model")
        if not self.meta_path:
            raise RuntimeError("~meta is required")
        if not self.pt_path:
            raise RuntimeError("~model is required")

        self.theta_topic    = rospy.get_param("~theta_topic", "/kinikun1/joint_states")
        self.theta_index    = int(rospy.get_param("~theta_index", 2))
        self.target_topic   = rospy.get_param("~target_topic", "/theta_target_deg")
        self.pressure_topic = rospy.get_param("~pressure_topic", "/mpa_pressure")
        self.mpa_cmd_topic  = rospy.get_param("~mpa_cmd_topic", "/mpa_cmd")

        self.rate_hz   = float(rospy.get_param("~rate", 100.0))
        self.frame_skip= int(rospy.get_param("~frame_skip", 2))
        self.K         = int(rospy.get_param("~K", 128))
        self.H         = int(rospy.get_param("~horizon", 20))
        self.temperature = float(rospy.get_param("~lambda", 2.0))
        self.sigma_u   = float(rospy.get_param("~sigma_u", 0.12))
        self.pmax      = float(rospy.get_param("~pmax", 0.7))
        self.w_path    = float(rospy.get_param("~w_path", 20.0))
        self.w_term    = float(rospy.get_param("~w_term", 8.0))
        self.w_rate    = float(rospy.get_param("~w_rate", 0.01))
        self.w_z       = float(rospy.get_param("~w_z", 0.05))
        self.w_box     = float(rospy.get_param("~w_box", 600.0))
        self.cmd_gain  = float(rospy.get_param("~cmd_gain", 4551.11111111))
        self.cmd_clip  = float(rospy.get_param("~cmd_clip_max", 4096.0))
        self.pressure_delay_s = float(rospy.get_param("~pressure_delay_s", 0.06))

        self.dt_override = float(rospy.get_param("~dt_override", 0.0))
        feature_order_override = rospy.get_param("~feature_order", None)
        dt_fallback = self.dt_override if self.dt_override > 0.0 else float(self.frame_skip)/self.rate_hz

        self.narx = NarxNumpy(self.meta_path, self.pt_path, dt_fallback=dt_fallback,
                               feature_order_override=feature_order_override)

        # Buffers & states
        self.lock = threading.Lock()
        self.theta_rad = 0.0
        self.theta_ref_rad = math.radians(float(rospy.get_param("~theta_target_deg", 20.0)))
        self.dz_est = 0.0  # if z unavailable

        self.ps_meas = 0.0
        self.pd_meas = 0.0

        # Delay buffers for measured pressures (t, ps, pd)
        self.press_buf: deque = deque(maxlen=2000)

        # Feature history buffers (for lagged features if order missing)
        self.hist_theta: deque = deque([0.0]*64, maxlen=64)
        self.hist_ps:    deque = deque([0.0]*64, maxlen=64)
        self.hist_pd:    deque = deque([0.0]*64, maxlen=64)
        self.hist_dz:    deque = deque([0.0]*64, maxlen=64)

        # CSV logger
        self.csv_path = rospy.get_param("~log_csv", "")
        self.csv_writer = None
        self.csv_fh = None
        if self.csv_path:
            os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
            self.csv_fh = open(self.csv_path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_fh)
            self.csv_writer.writerow([
                "t","th_sensor","th_model","th_ref",
                "p1_cmd_MPa","p2_cmd_MPa",
                "p1_meas","p2_meas","ps_meas","pd_meas",
                "Jmin","Jmean","a","b","err","ps_cmd","pd_cmd",
                "cycle_ms","used_meas"
            ])

        # ROS I/O
        self.pub_cmd = rospy.Publisher(self.mpa_cmd_topic, Vector3, queue_size=1)
        self.sub_tgt = rospy.Subscriber(self.target_topic, Float32, self._cb_target, queue_size=1)

        # Theta source: Float32(rad) or JointState
        # Decide by message type at runtime using a small helper subscriber that tries to parse
        self.sub_theta = rospy.Subscriber(self.theta_topic, rospy.AnyMsg, self._cb_theta_any, queue_size=10)
        self.sub_press = rospy.Subscriber(self.pressure_topic, Vector3, self._cb_pressure, queue_size=50)

        rospy.loginfo("narx_mppi_node: initialized. rate=%.1fHz frame_skip=%d (dt=%.4fs)",
                      self.rate_hz, self.frame_skip, float(self.frame_skip)/self.rate_hz)

    # ---------- Callbacks ----------
    def _cb_target(self, msg: Float32):
        self.theta_ref_rad = math.radians(float(msg.data))

    def _cb_theta_any(self, anymsg):
        # Try to deserialize as Float32 first
        try:
            m = Float32().deserialize(anymsg._buff)
            th = float(m.data)
            with self.lock:
                self.theta_rad = th
                self.hist_theta.appendleft(th)
            return
        except Exception:
            pass
        # Try JointState
        try:
            m = JointState().deserialize(anymsg._buff)
            if 0 <= self.theta_index < len(m.position):
                th = float(m.position[self.theta_index])
                with self.lock:
                    self.theta_rad = th
                    self.hist_theta.appendleft(th)
        except Exception:
            pass

    def _cb_pressure(self, msg: Vector3):
        tnow = rospy.get_time()
        p1 = float(msg.x); p2 = float(msg.y)
        ps, pd = p12_to_ps_pd(p1, p2)
        with self.lock:
            self.press_buf.append((tnow, ps, pd))
            self.hist_ps.appendleft(ps)
            self.hist_pd.appendleft(pd)

    # ---------- Helpers ----------
    def _pop_delayed_meas(self, tnow: float) -> Tuple[float,float,bool]:
        """Return (ps,pd,used) for target time (tnow - delay). Safe against mutation.
        Pops entries older than target time and keeps the latest not newer than target.
        """
        target_t = tnow - self.pressure_delay_s
        ps = self.ps_meas
        pd = self.pd_meas
        used = False
        with self.lock:
            # Pop left while older than target, remember last popped
            last = None
            while self.press_buf and self.press_buf[0][0] <= target_t:
                last = self.press_buf.popleft()
            if last is not None:
                _, ps, pd = last
                self.ps_meas, self.pd_meas = ps, pd
                used = True
        return ps, pd, used

    def _build_feature_vector(self, ps: float, pd: float) -> np.ndarray:
        """Build 1 x Din input for the NARX MLP.
        If order unknown, assume repeated [theta, ps, pd, dz] for L lags.
        dz is estimated (placeholder = 0 unless user feeds it externally).
        """
        Din = self.narx.W1.shape[1]
        names = self.narx.order
        x = np.zeros((1, Din), dtype=np.float32)

        # quick path if names look like f0..fN: push a default layout
        if all(n.startswith("f") for n in names):
            # assume 4 features with lag L
            L = max(1, Din // 4)
            # Ensure we have enough history
            th_hist = list(self.hist_theta)[:L]
            ps_hist = list(self.hist_ps)[:L]
            pd_hist = list(self.hist_pd)[:L]
            dz_hist = list(self.hist_dz)[:L]
            vec = []
            for k in range(L):
                vec.extend([th_hist[k], ps_hist[k], pd_hist[k], dz_hist[k]])
            vec = np.array(vec[:Din], dtype=np.float32)
            x[0,:] = vec
            return x

        # else map by names
        # Prepare a lookup for current and lagged values
        curr = {
            "theta": self.theta_rad,
            "ps": ps,
            "pd": pd,
            "dz": self.dz_est,
        }
        # Simple lag getter
        def get_lagged(name: str, lag: int) -> float:
            if name == "theta":
                hist = self.hist_theta
            elif name == "ps":
                hist = self.hist_ps
            elif name == "pd":
                hist = self.hist_pd
            elif name == "dz":
                hist = self.hist_dz
            else:
                return 0.0
            if lag < len(hist):
                return float(hist[lag])
            return float(hist[-1]) if hist else 0.0

        for i, nm in enumerate(names):
            # Allow forms like "theta", "ps[-1]", "pd[-3]", etc.
            base = nm
            lag = 0
            if "[" in nm and nm.endswith("]"):
                base = nm.split("[")[0]
                try:
                    lag = int(nm[nm.find("[")+1:-1])
                except Exception:
                    lag = 0
            if lag == 0 and base in curr:
                val = curr[base]
            else:
                val = get_lagged(base, max(0, lag))
            x[0,i] = float(val)
        return x

    # ---------- MPPI rollout & step ----------
    def _cost(self, th: float, th_ref: float, ps: float, pd: float, ps_prev: float, pd_prev: float, k: int, H: int) -> float:
        e = th_ref - th
        # simple quadratic cost
        c = self.w_path * (e*e)
        if k == H-1:
            c += self.w_term * (e*e)
        # rate cost on ps/pd (approx)
        c += self.w_rate * ((ps-ps_prev)**2 + (pd-pd_prev)**2)
        # box cost (soft) to keep p1,p2 in [0,pmax]
        p1, p2 = ps_pd_to_p12(ps, pd)
        viol = 0.0
        if p1 < 0.0:   viol += (-p1)
        if p2 < 0.0:   viol += (-p2)
        if p1 > self.pmax: viol += (p1-self.pmax)
        if p2 > self.pmax: viol += (p2-self.pmax)
        c += self.w_box * (viol*viol)
        return c

    def _rollout_predict(self, th0: float, ps0: float, pd0: float, U: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Given initial state proxy (th0, ps0, pd0) and U of shape (H,2) in [dps,dpd],
        simulate using the NARX forward map as a one-step predictor of theta.
        Returns arrays th_seq, ps_seq, pd_seq of length H.
        """
        th = th0
        ps = ps0
        pd = pd0
        th_seq = np.zeros(self.H, dtype=np.float32)
        ps_seq = np.zeros(self.H, dtype=np.float32)
        pd_seq = np.zeros(self.H, dtype=np.float32)

        for k in range(self.H):
            # apply delta and clamp to feasible ps/pd via p1,p2 box
            dps, dpd = float(U[k,0]), float(U[k,1])
            ps = ps + dps
            pd = pd + dpd
            p1,p2 = ps_pd_to_p12(ps, pd)
            p1 = clamp(p1, 0.0, self.pmax)
            p2 = clamp(p2, 0.0, self.pmax)
            ps, pd = p12_to_ps_pd(p1, p2)

            # Build feature and predict next theta (single-step ahead)
            x = self._build_feature_vector(ps, pd)
            y = self.narx.forward(x)
            if y.shape[1] >= 1:
                th = float(y[0,0])  # assume output[0] is theta_next[rad]
            # Optional: if y has z, you could update dz_est here

            th_seq[k] = th
            ps_seq[k] = ps
            pd_seq[k] = pd
        return th_seq, ps_seq, pd_seq

    def step(self):
        t0 = time.time()
        tnow = rospy.get_time()
        ps, pd, used_meas = self._pop_delayed_meas(tnow)
        with self.lock:
            th = self.theta_rad
        th_ref = self.theta_ref_rad

        # Initialize Gaussian control sequence around zero deltas
        U = np.random.normal(loc=0.0, scale=self.sigma_u, size=(self.K, self.H, 2)).astype(np.float32)

        J = np.zeros(self.K, dtype=np.float32)
        best_idx = 0
        best_J = float("inf")
        ps_prev = ps
        pd_prev = pd

        for i in range(self.K):
            th_seq, ps_seq, pd_seq = self._rollout_predict(th, ps, pd, U[i])
            # accumulate cost
            cost = 0.0
            ps_k = ps
            pd_k = pd
            for k in range(self.H):
                cost += self._cost(th_seq[k], th_ref, ps_seq[k], pd_seq[k], ps_k, pd_k, k, self.H)
                ps_k = ps_seq[k]
                pd_k = pd_seq[k]
            J[i] = cost
            if cost < best_J:
                best_J = cost
                best_idx = i

        # MPPI weights
        beta = np.min(J)
        w = np.exp(-(J - beta)/max(1e-6, self.temperature))
        w_sum = np.sum(w) + 1e-9
        dU = (w[:,None,None] * U).sum(axis=0) / w_sum

        # Apply only first control increment
        dps_cmd, dpd_cmd = float(dU[0,0]), float(dU[0,1])
        ps_cmd = ps + dps_cmd
        pd_cmd = pd + dpd_cmd

        # enforce feasibility by converting to p1,p2 box
        p1_cmd, p2_cmd = ps_pd_to_p12(ps_cmd, pd_cmd)
        p1_cmd = clamp(p1_cmd, 0.0, self.pmax)
        p2_cmd = clamp(p2_cmd, 0.0, self.pmax)
        ps_cmd, pd_cmd = p12_to_ps_pd(p1_cmd, p2_cmd)

        # Publish in MPa directly
        self.pub_cmd.publish(Vector3(x=p1_cmd, y=p2_cmd, z=0.0))

        # For logging: compute simple error & summary
        err = float(th_ref - th)
        Jmin = float(np.min(J)); Jmean = float(np.mean(J))

        # Derive convenience a,b from normalized box coordinates (for continuity with your logs)
        # a ~ ps/pmax in [0..2] clipped to [0..1.2] just for display, b ~ mapped from pd
        a = clamp(ps_cmd / max(1e-9, self.pmax), 0.0, 1.2)
        b = clamp(0.5 + 0.5 * (pd_cmd / max(1e-6, self.pmax)), 0.0, 1.2)

        # CSV log
        if self.csv_writer:
            now = rospy.get_time()
            p1_meas, p2_meas = ps_pd_to_p12(self.ps_meas, self.pd_meas)
            self.csv_writer.writerow([
                f"{now:.3f}", f"{th:.6f}", f"{th_seq[0] if 'th_seq' in locals() else th:.6f}", f"{th_ref:.6f}",
                f"{p1_cmd:.5f}", f"{p2_cmd:.5f}",
                f"{p1_meas:.5f}", f"{p2_meas:.5f}", f"{self.ps_meas:.5f}", f"{self.pd_meas:.5f}",
                f"{Jmin:.4f}", f"{Jmean:.4f}", f"{a:.5f}", f"{b:.5f}", f"{err:.6f}",
                f"{ps_cmd:.5f}", f"{pd_cmd:.5f}",
                f"{(time.time()-t0)*1000.0:.2f}", int(1 if used_meas else 0)
            ])
            if self.csv_fh:
                self.csv_fh.flush()

        # Console
        rospy.loginfo("[MPPI] th=%.2fdeg -> ref=%.2fdeg  ps=%.3f pd=%.3f  Jmin=%.2f Jmean=%.2f",
                      math.degrees(th), math.degrees(th_ref), ps_cmd, pd_cmd, Jmin, Jmean)

    def spin(self):
        r = rospy.Rate(self.rate_hz)
        i = 0
        try:
            while not rospy.is_shutdown():
                if i % max(1,self.frame_skip) == 0:
                    self.step()
                i += 1
                r.sleep()
        finally:
            if self.csv_fh:
                try:
                    self.csv_fh.close()
                except Exception:
                    pass

# -------------------------------
# Main
# -------------------------------

def main():
    rospy.init_node("narx_mppi_node")
    node = NarxMPPINode()
    node.spin()

if __name__ == "__main__":
    main()
