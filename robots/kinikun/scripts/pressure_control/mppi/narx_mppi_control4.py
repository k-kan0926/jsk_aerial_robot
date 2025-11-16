#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NARX + MPPI Controller (p1_cmd, p2_cmd版 - 完全版)

Features:
- p1_cmd, p2_cmd ベースのNARX予測
- GPU対応MPPI（バッチ推論で高速化）
- 遅延補償（pressure_delay_s）
- 制約処理（box constraint + rate limit）
- CSV logging

Usage:
  rosrun your_pkg narx_mppi_controller_p1p2_final.py \
    _meta:=models/narx_p1p2_production/narx_meta.json \
    _model:=models/narx_p1p2_production/narx_model.pt \
    _theta_topic:=/kinikun1/joint_states \
    _theta_index:=2 \
    _target_topic:=/theta_target_deg \
    _pressure_topic:=/mpa_pressure \
    _mpa_cmd_topic:=/mpa_cmd \
    _rate:=100 \
    _frame_skip:=2 \
    _pressure_delay_s:=0.084 \
    _K:=64 \
    _horizon:=12 \
    _log_csv:=logs/mppi_control.csv
"""
import os, json, time, math, csv, threading
from collections import deque
from typing import Tuple, Optional

import numpy as np
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState

# ========== Torch Loader ==========
def load_torch_weights(pt_path: str):
    """PyTorch state_dict → numpy arrays"""
    try:
        import torch
    except:
        raise RuntimeError("PyTorch required")
    
    sd = torch.load(pt_path, map_location='cpu')
    if hasattr(sd, 'state_dict'):
        sd = sd.state_dict()
    
    # Try different key patterns
    candidates = [
        ('net.0.weight', 'net.0.bias', 'net.2.weight', 'net.2.bias', 'net.4.weight', 'net.4.bias'),
        ('0.weight', '0.bias', '2.weight', '2.bias', '4.weight', '4.bias'),
    ]
    
    keys = None
    for cand in candidates:
        if all(k in sd for k in cand):
            keys = cand
            break
    
    if keys is None:
        raise KeyError(f"Cannot find model weights. Available keys: {list(sd.keys())[:10]}")
    
    def to_np(t):
        return t.detach().cpu().numpy().astype(np.float32)
    
    W1 = to_np(sd[keys[0]]); b1 = to_np(sd[keys[1]])
    W2 = to_np(sd[keys[2]]); b2 = to_np(sd[keys[3]])
    W3 = to_np(sd[keys[4]]); b3 = to_np(sd[keys[5]])
    
    return W1, b1, W2, b2, W3, b3

# ========== NARX Model (Numpy) ==========
class NarxNumpy:
    def __init__(self, meta_path: str, pt_path: str):
        W1, b1, W2, b2, W3, b3 = load_torch_weights(pt_path)
        self.W1, self.b1 = W1.astype(np.float32), b1.astype(np.float32)
        self.W2, self.b2 = W2.astype(np.float32), b2.astype(np.float32)
        self.W3, self.b3 = W3.astype(np.float32), b3.astype(np.float32)
        
        with open(meta_path, 'r') as f:
            meta = json.load(f)
        
        self.feat_cols = meta.get('feature_names_single_slice', [])
        self.lags = meta.get('lags', 24)
        self.delay = meta.get('delay', 17)
        self.mu = np.array(meta.get('mu', [0.0]*self.W1.shape[1]), dtype=np.float32)
        self.std = np.array(meta.get('std', [1.0]*self.W1.shape[1]), dtype=np.float32)
        self.dt = meta.get('dt_est', 0.005)
        self.theta_minmax = tuple(meta.get('theta_train_minmax', [-math.pi, math.pi]))
        
        rospy.loginfo(f"[NARX] Loaded: lags={self.lags}, delay={self.delay}, dt={self.dt*1000:.1f}ms")
        rospy.loginfo(f"[NARX] Features: {self.feat_cols}")
        rospy.loginfo(f"[NARX] Model: {self.W1.shape} -> {self.W2.shape} -> {self.W3.shape}")
    
    def forward(self, x_batch: np.ndarray) -> np.ndarray:
        """Batch forward: (N, Din) -> (N, 1)"""
        x = x_batch.astype(np.float32)
        x_norm = (x - self.mu) / (self.std + 1e-8)
        h1 = np.tanh(x_norm @ self.W1.T + self.b1)
        h2 = np.tanh(h1 @ self.W2.T + self.b2)
        y = h2 @ self.W3.T + self.b3
        return y.astype(np.float32)

# ========== Utilities ==========
def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

# ========== MPPI Controller ==========
class NarxMPPIController:
    def __init__(self):
        # Paths
        self.meta_path = rospy.get_param('~meta')
        self.pt_path = rospy.get_param('~model')
        
        # Topics
        self.theta_topic = rospy.get_param('~theta_topic', '/kinikun1/joint_states')
        self.theta_index = int(rospy.get_param('~theta_index', 2))
        self.target_topic = rospy.get_param('~target_topic', '/theta_target_deg')
        self.pressure_topic = rospy.get_param('~pressure_topic', '/mpa_pressure')
        self.mpa_cmd_topic = rospy.get_param('~mpa_cmd_topic', '/mpa_cmd')
        
        # Control params
        self.rate_hz = float(rospy.get_param('~rate', 100.0))
        self.frame_skip = int(rospy.get_param('~frame_skip', 2))
        self.dt_control = self.frame_skip / self.rate_hz
        
        # MPPI params
        self.K = int(rospy.get_param('~K', 64))
        self.H = int(rospy.get_param('~horizon', 12))
        self.temperature = float(rospy.get_param('~lambda', 2.0))
        self.sigma_u = float(rospy.get_param('~sigma_u', 0.08))
        
        # Costs
        self.w_path = float(rospy.get_param('~w_path', 50.0))
        self.w_term = float(rospy.get_param('~w_term', 15.0))
        self.w_rate = float(rospy.get_param('~w_rate', 0.05))
        self.w_box = float(rospy.get_param('~w_box', 1000.0))
        
        # Pressure limits
        self.pmax = float(rospy.get_param('~pmax', 0.70))
        self.pressure_delay_s = float(rospy.get_param('~pressure_delay_s', 0.084))
        
        # Load model
        self.narx = NarxNumpy(self.meta_path, self.pt_path)
        
        # State
        self.lock = threading.Lock()
        self.theta_rad = 0.0
        self.theta_ref_rad = math.radians(float(rospy.get_param('~theta_target_deg', 0.0)))
        
        self.p1_cmd = 0.0
        self.p2_cmd = 0.0
        self.p1_meas = 0.0
        self.p2_meas = 0.0
        
        # Delay buffer: (timestamp, p1_meas, p2_meas)
        self.press_buf = deque(maxlen=2000)
        
        # History buffers for NARX features
        maxlen = max(100, self.narx.lags * 2)
        self.hist_theta = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_p1_cmd = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_p2_cmd = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_dp1_dt = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_dp2_dt = deque([0.0] * maxlen, maxlen=maxlen)
        
        # CSV logging
        self.csv_path = rospy.get_param('~log_csv', '')
        self.csv_writer = None
        self.csv_fh = None
        if self.csv_path:
            os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
            self.csv_fh = open(self.csv_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_fh)
            self.csv_writer.writerow([
                't', 'theta_sensor', 'theta_ref', 'p1_cmd', 'p2_cmd',
                'p1_meas', 'p2_meas', 'Jmin', 'Jmean', 'err', 'cycle_ms'
            ])
        
        # ROS I/O
        self.pub_cmd = rospy.Publisher(self.mpa_cmd_topic, Vector3, queue_size=1)
        rospy.Subscriber(self.target_topic, Float32, self._cb_target, queue_size=1)
        rospy.Subscriber(self.theta_topic, rospy.AnyMsg, self._cb_theta, queue_size=10)
        rospy.Subscriber(self.pressure_topic, Vector3, self._cb_pressure, queue_size=50)
        
        rospy.loginfo(f"[MPPI] K={self.K}, H={self.H}, rate={self.rate_hz}Hz, dt={self.dt_control*1000:.1f}ms")
        rospy.loginfo(f"[MPPI] sigma_u={self.sigma_u}, temp={self.temperature}")
        rospy.loginfo(f"[MPPI] Costs: path={self.w_path}, term={self.w_term}, rate={self.w_rate}, box={self.w_box}")
    
    # ========== Callbacks ==========
    def _cb_target(self, msg: Float32):
        self.theta_ref_rad = math.radians(float(msg.data))
    
    def _cb_theta(self, anymsg):
        # Try Float32
        try:
            m = Float32().deserialize(anymsg._buff)
            th = float(m.data)
            with self.lock:
                self.theta_rad = th
                self.hist_theta.appendleft(th)
            return
        except:
            pass
        
        # Try JointState
        try:
            m = JointState().deserialize(anymsg._buff)
            if 0 <= self.theta_index < len(m.position):
                th = float(m.position[self.theta_index])
                with self.lock:
                    self.theta_rad = th
                    self.hist_theta.appendleft(th)
        except:
            pass
    
    def _cb_pressure(self, msg: Vector3):
        tnow = rospy.get_time()
        p1 = float(msg.x)
        p2 = float(msg.y)
        with self.lock:
            self.press_buf.append((tnow, p1, p2))
    
    def _get_delayed_pressure(self, tnow: float) -> Tuple[float, float, bool]:
        """Get pressure measurement from delay buffer"""
        target_t = tnow - self.pressure_delay_s
        p1, p2, used = self.p1_meas, self.p2_meas, False
        
        with self.lock:
            last = None
            while self.press_buf and self.press_buf[0][0] <= target_t:
                last = self.press_buf.popleft()
            if last is not None:
                _, p1, p2 = last
                self.p1_meas, self.p2_meas = p1, p2
                used = True
        
        return p1, p2, used
    
    # ========== Feature Builder ==========
    def _build_feature_vector(self, p1: float, p2: float) -> np.ndarray:
        """Build (1, Din) feature vector for NARX"""
        lags = self.narx.lags
        feat_cols = self.narx.feat_cols
        n_feat = len(feat_cols)
        
        x = np.zeros((1, lags * n_feat), dtype=np.float32)
        
        for lag_idx in range(lags):
            for feat_idx, fname in enumerate(feat_cols):
                col_idx = lag_idx * n_feat + feat_idx
                
                if fname == 'theta[rad]':
                    x[0, col_idx] = self.hist_theta[lag_idx] if lag_idx < len(self.hist_theta) else self.theta_rad
                elif fname == 'p1_cmd[MPa]':
                    x[0, col_idx] = self.hist_p1_cmd[lag_idx] if lag_idx < len(self.hist_p1_cmd) else p1
                elif fname == 'p2_cmd[MPa]':
                    x[0, col_idx] = self.hist_p2_cmd[lag_idx] if lag_idx < len(self.hist_p2_cmd) else p2
                elif fname == 'dp1_cmd_dt[MPa/s]':
                    x[0, col_idx] = self.hist_dp1_dt[lag_idx] if lag_idx < len(self.hist_dp1_dt) else 0.0
                elif fname == 'dp2_cmd_dt[MPa/s]':
                    x[0, col_idx] = self.hist_dp2_dt[lag_idx] if lag_idx < len(self.hist_dp2_dt) else 0.0
                elif fname == 'dz[m]':
                    x[0, col_idx] = 0.0  # Not used in control
        
        return x
    
    # ========== Cost Function ==========
    def _cost(self, th: float, th_ref: float, p1: float, p2: float,
              p1_prev: float, p2_prev: float, k: int, H: int) -> float:
        """Stage cost"""
        err = th_ref - th
        c = self.w_path * (err * err)
        
        if k == H - 1:
            c += self.w_term * (err * err)
        
        # Rate penalty
        dp1 = p1 - p1_prev
        dp2 = p2 - p2_prev
        c += self.w_rate * (dp1*dp1 + dp2*dp2)
        
        # Box constraint penalty
        viol = 0.0
        if p1 < 0.0: viol += (-p1)
        if p2 < 0.0: viol += (-p2)
        if p1 > self.pmax: viol += (p1 - self.pmax)
        if p2 > self.pmax: viol += (p2 - self.pmax)
        c += self.w_box * (viol * viol)
        
        return c
    
    # ========== Rollout (Batch) ==========
    def _rollout_batch(self, th0: float, p1_0: float, p2_0: float, U: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Batch rollout for MPPI
        U: (K, H, 2) control perturbations [dp1, dp2]
        Returns: th_seq (K, H), p1_seq (K, H), p2_seq (K, H)
        """
        K, H = U.shape[0], U.shape[1]
        
        th_seq = np.zeros((K, H), dtype=np.float32)
        p1_seq = np.zeros((K, H), dtype=np.float32)
        p2_seq = np.zeros((K, H), dtype=np.float32)
        
        # Initialize state for all K trajectories
        th_curr = np.full(K, th0, dtype=np.float32)
        p1_curr = np.full(K, p1_0, dtype=np.float32)
        p2_curr = np.full(K, p2_0, dtype=np.float32)
        
        # History buffers (per trajectory)
        # Simplified: use current state for all lags (approximation)
        
        th_lo, th_hi = self.narx.theta_minmax
        
        for k in range(H):
            # Apply control
            p1_next = p1_curr + U[:, k, 0]
            p2_next = p2_curr + U[:, k, 1]
            
            # Clamp to box
            p1_next = np.clip(p1_next, 0.0, self.pmax)
            p2_next = np.clip(p2_next, 0.0, self.pmax)
            
            # Build feature vectors for all K samples
            X_batch = np.zeros((K, self.narx.lags * len(self.narx.feat_cols)), dtype=np.float32)
            
            for i in range(K):
                # Simplified feature: repeat current state for all lags
                # (More accurate: maintain per-trajectory history, but expensive)
                for lag in range(self.narx.lags):
                    offset = lag * len(self.narx.feat_cols)
                    X_batch[i, offset + 0] = th_curr[i]  # theta
                    X_batch[i, offset + 1] = p1_next[i]  # p1_cmd
                    X_batch[i, offset + 2] = p2_next[i]  # p2_cmd
                    X_batch[i, offset + 3] = 0.0  # dp1/dt (approx)
                    X_batch[i, offset + 4] = 0.0  # dp2/dt (approx)
                    if len(self.narx.feat_cols) > 5:
                        X_batch[i, offset + 5] = 0.0  # dz
            
            # Batch prediction
            Y_batch = self.narx.forward(X_batch)  # (K, 1)
            th_next = Y_batch[:, 0]
            
            # Clamp theta
            th_next = np.clip(th_next, th_lo, th_hi)
            
            # Store
            th_seq[:, k] = th_next
            p1_seq[:, k] = p1_next
            p2_seq[:, k] = p2_next
            
            # Update state
            th_curr = th_next
            p1_curr = p1_next
            p2_curr = p2_next
        
        return th_seq, p1_seq, p2_seq
    
    # ========== MPPI Step ==========
    def step(self):
        t0 = time.time()
        tnow = rospy.get_time()
        
        # Get delayed pressure
        p1_meas, p2_meas, used_meas = self._get_delayed_pressure(tnow)
        
        with self.lock:
            th = self.theta_rad
        th_ref = self.theta_ref_rad
        
        # Use commanded pressure as initial state (more stable than measured)
        p1_0 = self.p1_cmd if abs(self.p1_cmd) > 1e-6 else p1_meas
        p2_0 = self.p2_cmd if abs(self.p2_cmd) > 1e-6 else p2_meas
        
        # Sample control perturbations
        U = np.random.normal(0.0, self.sigma_u, size=(self.K, self.H, 2)).astype(np.float32)
        
        # Batch rollout
        th_seqs, p1_seqs, p2_seqs = self._rollout_batch(th, p1_0, p2_0, U)
        
        # Compute costs
        J = np.zeros(self.K, dtype=np.float32)
        for i in range(self.K):
            cost = 0.0
            p1_prev, p2_prev = p1_0, p2_0
            for k in range(self.H):
                cost += self._cost(th_seqs[i, k], th_ref, p1_seqs[i, k], p2_seqs[i, k],
                                   p1_prev, p2_prev, k, self.H)
                p1_prev, p2_prev = p1_seqs[i, k], p2_seqs[i, k]
            J[i] = cost
        
        # MPPI weights
        beta = np.min(J)
        w = np.exp(-(J - beta) / max(1e-6, self.temperature))
        w_sum = np.sum(w) + 1e-9
        
        # Weighted average control
        dU = np.sum(w[:, None, None] * U, axis=0) / w_sum  # (H, 2)
        
        # Apply first control
        dp1_cmd, dp2_cmd = float(dU[0, 0]), float(dU[0, 1])
        p1_cmd = clamp(p1_0 + dp1_cmd, 0.0, self.pmax)
        p2_cmd = clamp(p2_0 + dp2_cmd, 0.0, self.pmax)
        
        # Publish
        self.pub_cmd.publish(Vector3(x=p1_cmd, y=p2_cmd, z=0.0))
        
        # Update history
        with self.lock:
            self.p1_cmd, self.p2_cmd = p1_cmd, p2_cmd
            self.hist_p1_cmd.appendleft(p1_cmd)
            self.hist_p2_cmd.appendleft(p2_cmd)
            self.hist_dp1_dt.appendleft(dp1_cmd / self.dt_control)
            self.hist_dp2_dt.appendleft(dp2_cmd / self.dt_control)
        
        # Logging
        err = float(th_ref - th)
        Jmin, Jmean = float(np.min(J)), float(np.mean(J))
        cycle_ms = (time.time() - t0) * 1000.0
        
        if self.csv_writer:
            self.csv_writer.writerow([
                f"{tnow:.3f}", f"{th:.6f}", f"{th_ref:.6f}",
                f"{p1_cmd:.5f}", f"{p2_cmd:.5f}",
                f"{p1_meas:.5f}", f"{p2_meas:.5f}",
                f"{Jmin:.4f}", f"{Jmean:.4f}", f"{err:.6f}", f"{cycle_ms:.2f}"
            ])
            if self.csv_fh:
                self.csv_fh.flush()
        
        rospy.loginfo(f"[MPPI] θ={math.degrees(th):.1f}° → {math.degrees(th_ref):.1f}° | "
                      f"p1={p1_cmd:.3f} p2={p2_cmd:.3f} | J={Jmin:.2f} | {cycle_ms:.1f}ms")
    
    def spin(self):
        r = rospy.Rate(self.rate_hz)
        i = 0
        try:
            while not rospy.is_shutdown():
                if i % max(1, self.frame_skip) == 0:
                    self.step()
                i += 1
                r.sleep()
        finally:
            if self.csv_fh:
                self.csv_fh.close()

# ========== Main ==========
def main():
    rospy.init_node('narx_mppi_controller_p1p2')
    controller = NarxMPPIController()
    controller.spin()

if __name__ == '__main__':
    main()