#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Neural Hammerstein inverse MPC (with delay) + HOLD mode for real robot.

- Loads neural static kernel (MonoDeltaNN) and Hammerstein dynamics (alpha,kS,kD,delay) from meta.
- Runs receding-horizon inverse MPC to compute p1,p2 given current theta and target theta*.
- No pressure sensors required: the input delay line uses previously commanded pressures.
- HOLD mode: when inside a small deadband and low velocity for some dwell time, keep last pressures.

ROS I/O (defaults can be overridden via params):
  ~model_pt        : path to *.pt (default: ~/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/kinikun/models/model_k4b_nh.pt)
  ~model_meta      : path to *_nh_meta.npz (same folder, *_nh_meta.npz)
  ~joint_topic     : /kinikun1/joint_states
  ~joint_name      : arm1_joint
  ~target_topic    : /target_angle   (Float32; deg if ~target_in_deg=true)
  ~target_in_deg   : true
  ~pub_topic_p12   : /p1p2_cmd       (Vector3: x=p1, y=p2, MPa)
  ~also_pub_mpa_cmd: false
  ~pub_topic_mpa_cmd: /mpa_cmd       (Vector3: x=raw1, y=raw2 counts)
  ~raw_counts_per_MPa: 4096/0.9

HOLD parameters:
  ~hold_enabled        : true
  ~hold_deadband_deg   : 0.8
  ~hold_rate_deg_s     : 2.0
  ~hold_dwell_s        : 0.25
  ~hold_exit_deg       : 1.2

MPC core parameters (good starting values):
  ~H            : 10
  ~w_stage      : 1.0
  ~w_term       : 6.0
  ~w_z          : 0.2
  ~w_sigma      : 0.02       # keep small to avoid drifting back when holding is desired
  ~w_rate       : 0.4
  ~block        : 2          # move blocking (keep command constant within each block)
  ~w_block      : 8.0
  ~rate_sigma   : 0.12       # MPa/step (soft via penalty)
  ~rate_delta   : 0.12
  ~max_nfev     : 600
  ~rate_hz      : 100.0
  ~p_max_MPa    : 1.05

Usage:
rosrun kinikun neural_B_hammerstein_mpc.py   _target_in_deg:=true   _joint_topic:=/kinikun1/joint_states   _joint_name:=arm1_joint   _target_topic:=/target_angle   _pub_topic_p12:=/p1p2_cmd   _model_pt:=/home/USER/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/kinikun/models/model_k4b_nh.pt   _model_meta:=/home/USER/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/kinikun/models/model_k4b_nh_meta.npz   _H:=10 _w_stage:=1.0 _w_term:=6.0 _w_sigma:=0.02 _w_rate:=0.4 _w_block:=8.0 _block:=2   _rate_sigma:=0.12 _rate_delta:=0.12 _p_max_MPa:=1.05   _hold_enabled:=true _hold_deadband_deg:=0.8 _hold_rate_deg_s:=2.0 _hold_dwell_s:=0.25 _hold_exit_deg:=1.2


"""

import os, math, time, importlib
import numpy as np
import rospy
import torch
import torch.nn as nn
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

# ----------------- Neural static model (must match training) -----------------
class MonoDeltaNN(nn.Module):
    def __init__(self, M, hidden, c_grid):
        super().__init__()
        self.M = int(M)
        self.c = nn.Parameter(torch.tensor(c_grid, dtype=torch.float32), requires_grad=False)
        self.enc = nn.Sequential(
            nn.Linear(1, hidden), nn.ELU(),
            nn.Linear(hidden, hidden), nn.ELU()
        )
        self.head_A = nn.Linear(hidden, 1)
        self.head_w = nn.Linear(hidden, self.M)
        self.head_s = nn.Linear(hidden, self.M)
        nn.init.zeros_(self.head_w.weight); nn.init.zeros_(self.head_w.bias)
        nn.init.zeros_(self.head_s.weight); nn.init.constant_(self.head_s.bias, 1.0)

    def forward(self, Sigma_hat, Delta_hat):
        h = self.enc(Sigma_hat)
        A = self.head_A(h)
        w = torch.nn.functional.softplus(self.head_w(h)) + 1e-6
        s = torch.nn.functional.softplus(self.head_s(h)) + 1e-6
        d = Delta_hat - self.c.view(1, -1)
        bank = torch.tanh(s * d)
        return A + (w * bank).sum(dim=1, keepdim=True)

# ----------------- Utilities -----------------
def clip(x, lo, hi): return max(lo, min(hi, x))

def clamp_box_p1p2(p1, p2, pmax):
    return clip(p1, 0.0, pmax), clip(p2, 0.0, pmax)

def z_pred(ps, pd, zc):
    if zc is None or len(zc) < 4: return 0.0
    return float(zc[0] + zc[1]*ps + zc[2]*pd + zc[3]*ps*pd)

def invert_delta_bisect(ps, theta_star, theta_stat_fn, lo=-2.0, hi=2.0, it=50):
    # monotone in Delta assumption; returns mid even if no exact root (nearest end if sign not changing)
    f_lo = theta_stat_fn(ps, lo) - theta_star
    f_hi = theta_stat_fn(ps, hi) - theta_star
    if f_lo * f_hi > 0:
        # pick the closer endpoint
        return lo if abs(f_lo) < abs(f_hi) else hi
    a, b = lo, hi
    for _ in range(it):
        m = 0.5*(a + b)
        fm = theta_stat_fn(ps, m) - theta_star
        if abs(fm) < 1e-5:
            return m
        if f_lo * fm <= 0:
            b = m; f_hi = fm
        else:
            a = m; f_lo = fm
    return 0.5*(a + b)

# ----------------- Node -----------------
class NeuralHammersteinMPCNode:
    def __init__(self):
        # Model paths (defaults to workspace)
        default_pt   = os.path.expanduser("~/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/kinikun/models/model_k4b_nh.pt")
        default_meta = os.path.expanduser("~/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/kinikun/models/model_k4b_nh_meta.npz")
        self.model_pt   = rospy.get_param("~model_pt", default_pt)
        self.model_meta = rospy.get_param("~model_meta", default_meta)

        # Topics / names
        self.joint_topic   = rospy.get_param("~joint_topic", "/kinikun1/joint_states")
        self.joint_name    = rospy.get_param("~joint_name",  "arm1_joint")
        self.target_topic  = rospy.get_param("~target_topic","/target_angle")
        self.target_in_deg = rospy.get_param("~target_in_deg", True)
        self.rate_hz       = float(rospy.get_param("~rate_hz", 100.0))

        # Publishers
        self.pub_topic_p12   = rospy.get_param("~pub_topic_p12", "/p1p2_cmd")
        self.also_pub_mpa    = rospy.get_param("~also_pub_mpa_cmd", False)
        self.pub_topic_mpa   = rospy.get_param("~pub_topic_mpa_cmd", "/mpa_cmd")
        self.raw_counts_per_MPa = float(rospy.get_param("~raw_counts_per_MPa", 4096.0/0.9))
        self.pub_p12 = rospy.Publisher(self.pub_topic_p12, Vector3, queue_size=10)
        self.pub_mpa = rospy.Publisher(self.pub_topic_mpa, Vector3, queue_size=10) if self.also_pub_mpa else None

        # Plant / constraints
        self.pmax     = float(rospy.get_param("~p_max_MPa", 1.05))
        self.rateS    = float(rospy.get_param("~rate_sigma", 0.12))  # MPa/step (soft penalty)
        self.rateD    = float(rospy.get_param("~rate_delta", 0.12))

        # HOLD mode
        self.hold_enabled  = rospy.get_param("~hold_enabled", True)
        self.hold_db_deg   = float(rospy.get_param("~hold_deadband_deg", 0.8))
        self.hold_rate_deg = float(rospy.get_param("~hold_rate_deg_s", 2.0))
        self.hold_dwell_s  = float(rospy.get_param("~hold_dwell_s", 0.25))
        self.hold_exit_deg = float(rospy.get_param("~hold_exit_deg", 1.2))
        self._hold_on = False
        self._hold_since = None
        self._p1_hold = 0.0
        self._p2_hold = 0.0

        # MPC weights / horizon
        self.H        = int(rospy.get_param("~H", 10))
        self.w_stage  = float(rospy.get_param("~w_stage", 1.0))
        self.w_term   = float(rospy.get_param("~w_term", 6.0))
        self.wz       = float(rospy.get_param("~w_z", 0.2))
        self.wsig     = float(rospy.get_param("~w_sigma", 0.02))
        self.wrate    = float(rospy.get_param("~w_rate", 0.4))
        self.block    = int(rospy.get_param("~block", 2))
        self.wblock   = float(rospy.get_param("~w_block", 8.0))
        self.max_nfev = int(rospy.get_param("~max_nfev", 600))

        # State
        self.theta = None
        self.theta_ref = 0.0
        self._last_theta = None
        self._last_t_s = None

        # Commands & delay history
        self.ps_cmd = 0.05
        self.pd_cmd = 0.0
        self.hist_sigma = []
        self.hist_delta = []

        # Load model & meta
        self._load_model()

        # Subscribers
        rospy.Subscriber(self.joint_topic, JointState, self._cb_joint)
        rospy.Subscriber(self.target_topic, Float32,   self._cb_target)

        self._last_loop_t = rospy.Time.now().to_sec()

        rospy.loginfo("[NH-MPC] loaded. delay=%d, dt=%.4f, alpha=%.3f, kS=%.3f, kD=%.3f",
                      self.delay, self.dt, self.alpha, self.kS, self.kD)

    # ----------------- Model load -----------------
    def _load_model(self):
        meta = np.load(self.model_meta, allow_pickle=True)

        self.dt     = float(meta["dt"])
        self.muS    = float(meta["muS"]); self.sdS = float(meta["sdS"])
        self.muD    = float(meta["muD"]); self.sdD = float(meta["sdD"])
        self.M      = int(meta["M"]);     self.hidden = int(meta["hidden"])
        self.alpha  = float(meta["alpha"])
        self.kS     = float(meta["kSigma"])
        self.kD     = float(meta["kDelta"])
        self.delay  = int(meta["delay"])
        self.sigma_ref = float(meta["sigma_ref"]) if "sigma_ref" in meta.files else float(np.median([0.5*self.pmax, self.pmax]))
        self.z_coef = meta["z_coef"] if "z_coef" in meta.files else np.array([])

        # centers for Delta (normalized space)
        c_grid = meta["c_grid"] if "c_grid" in meta.files else None
        if c_grid is None:
            # fallback: evenly spaced in [-1,1]
            c_grid = np.linspace(-1.0, 1.0, self.M).astype(np.float32)
        self.c_grid = np.asarray(c_grid, dtype=np.float32)

        # NN
        state = torch.load(self.model_pt, map_location="cpu")  # NOTE: weights_only=True may be enabled in future PyTorch
        self.model = MonoDeltaNN(self.M, self.hidden, self.c_grid)
        self.model.load_state_dict(state)
        self.model.eval()

        # initialize delay history with current command
        self.hist_sigma = [self.ps_cmd]*self.delay
        self.hist_delta = [self.pd_cmd]*self.delay

    # ----------------- ROS callbacks -----------------
    def _cb_target(self, msg: Float32):
        val = float(msg.data)
        self.theta_ref = math.radians(val) if self.target_in_deg else val
        # target change => exit HOLD immediately
        self._hold_on = False
        self._hold_since = None

    def _cb_joint(self, msg: JointState):
        if self.joint_name in msg.name:
            i = msg.name.index(self.joint_name)
            self.theta = float(msg.position[i])

    # ----------------- Holding helpers -----------------
    def _theta_rate(self, theta, t_s):
        if self._last_theta is None or self._last_t_s is None:
            self._last_theta, self._last_t_s = theta, t_s
            return 0.0
        dt = max(1e-3, t_s - self._last_t_s)
        rate = (theta - self._last_theta)/dt
        self._last_theta, self._last_t_s = theta, t_s
        return rate

    def _maybe_enter_hold(self, e_deg, rate_deg_s, now_s):
        if not self.hold_enabled: return
        inside = (abs(e_deg) < self.hold_db_deg) and (abs(rate_deg_s) < self.hold_rate_deg)
        if inside:
            if self._hold_since is None:
                self._hold_since = now_s
            elif (now_s - self._hold_since) >= self.hold_dwell_s:
                if not self._hold_on:
                    self._hold_on = True
                    self._p1_hold = 0.5*(self.ps_cmd + self.pd_cmd)
                    self._p2_hold = 0.5*(self.ps_cmd - self.pd_cmd)
        else:
            self._hold_since = None
            if self._hold_on and abs(e_deg) > self.hold_exit_deg:
                self._hold_on = False

    # ----------------- Static theta(ps,pd) -----------------
    @torch.no_grad()
    def theta_stat(self, ps, pd):
        S = torch.tensor([(ps - self.muS)/self.sdS], dtype=torch.float32).view(1,1)
        D = torch.tensor([(pd - self.muD)/self.sdD], dtype=torch.float32).view(1,1)
        y = self.model(S, D)
        return float(y.item())

    # ----------------- MPC core -----------------
    def _warm_start(self, theta_star):
        # start from sigma_ref + static inverse for pd
        u0 = np.zeros(2*self.H, float)
        for k in range(self.H):
            ps = self.sigma_ref
            pd = invert_delta_bisect(ps, theta_star, self.theta_stat, lo=-self.pmax, hi=self.pmax)
            p1 = 0.5*(ps + pd); p2 = 0.5*(ps - pd)
            p1, p2 = clamp_box_p1p2(p1, p2, self.pmax)
            u0[2*k+0] = p1 + p2
            u0[2*k+1] = p1 - p2
        return u0

    def _simulate_cost(self, u_vec, theta0, theta_star):
        # Apply command with input delay using history (sensorless)
        sig_appl = np.r_[self.hist_sigma, u_vec[0::2]]
        del_appl = np.r_[self.hist_delta, u_vec[1::2]]

        theta = theta0
        costs = []
        for k in range(self.H):
            # effective inputs to plant (after delay)
            ps_eff = float(sig_appl[k])
            pd_eff = float(del_appl[k])
            if k == 0:
                ps_prev = self.hist_sigma[-1] if self.delay>0 else self.ps_cmd
                pd_prev = self.hist_delta[-1] if self.delay>0 else self.pd_cmd
            else:
                ps_prev = float(sig_appl[k-1])
                pd_prev = float(del_appl[k-1])

            dS_eff = (ps_eff - ps_prev) / self.dt
            dD_eff = (pd_eff - pd_prev) / self.dt

            ths = self.theta_stat(ps_eff, pd_eff)
            theta = theta + self.dt*( self.alpha*(ths - theta) + self.kS*dS_eff + self.kD*dD_eff )

            # command penalties (on decision variables)
            ps_cmd = float(u_vec[2*k+0])
            pd_cmd = float(u_vec[2*k+1])

            # box penalty (soft)
            p1 = 0.5*(ps_cmd + pd_cmd)
            p2 = 0.5*(ps_cmd - pd_cmd)
            box_violation = 0.0
            if p1<0: box_violation += -p1
            if p2<0: box_violation += -p2
            if p1>self.pmax: box_violation += (p1 - self.pmax)
            if p2>self.pmax: box_violation += (p2 - self.pmax)

            # rate penalty on commands
            if k == 0:
                dS_cmd = ps_cmd - self.ps_cmd
                dD_cmd = pd_cmd - self.pd_cmd
            else:
                dS_cmd = ps_cmd - float(u_vec[2*(k-1)+0])
                dD_cmd = pd_cmd - float(u_vec[2*(k-1)+1])
            rate_over = max(0.0, abs(dS_cmd) - self.rateS) + max(0.0, abs(dD_cmd) - self.rateD)

            # move blocking penalty
            block_pen = 0.0
            if self.block > 1 and (k % self.block != 0):
                ps_prev_cmd = float(u_vec[2*(k-1)+0])
                pd_prev_cmd = float(u_vec[2*(k-1)+1])
                block_pen = (ps_cmd - ps_prev_cmd)**2 + (pd_cmd - pd_prev_cmd)**2

            c = ( self.w_stage*(theta - theta_star)**2
                + self.wz*(z_pred(ps_cmd, pd_cmd, self.z_coef)**2)
                + self.wsig*((ps_cmd - self.sigma_ref)**2)
                + self.wrate*(dS_cmd**2 + dD_cmd**2)
                + self.w_block*block_pen
                + 1e6*box_violation + 1e3*rate_over )
            costs.append(c)

        c_term = self.w_term*(theta - theta_star)**2
        return np.array(costs + [c_term], float)

    def _solve_mpc(self, theta0, theta_star):
        u0 = self._warm_start(theta_star)
        try:
            from scipy.optimize import least_squares
            res = least_squares(lambda x: self._simulate_cost(x, theta0, theta_star),
                                u0, method="trf", ftol=1e-9, xtol=1e-9, gtol=1e-9,
                                max_nfev=self.max_nfev)
            u_opt = res.x
        except Exception:
            # simple coordinate descent fallback
            u_opt = u0.copy()
            for _ in range(60):
                base = self._simulate_cost(u_opt, theta0, theta_star).sum()
                for i in range(len(u_opt)):
                    h = 1e-3
                    u_opt[i] += h
                    c_plus = self._simulate_cost(u_opt, theta0, theta_star).sum()
                    u_opt[i] -= 2*h
                    c_minus = self._simulate_cost(u_opt, theta0, theta_star).sum()
                    u_opt[i] += h
                    g = (c_plus - c_minus)/(2*h)
                    u_opt[i] -= 0.1*g
        return u_opt

    # ----------------- Main compute -----------------
    def compute(self, dt):
        if self.theta is None:
            return None

        now_s = rospy.Time.now().to_sec()
        e = self.theta_ref - self.theta
        e_deg = math.degrees(e)
        rate_deg_s = math.degrees(self._theta_rate(self.theta, now_s))

        # HOLD logic
        self._maybe_enter_hold(e_deg, rate_deg_s, now_s)
        if self._hold_on:
            # keep last determined hold pressures (rate-limited towards hold setpoints)
            p1_cur = 0.5*(self.ps_cmd + self.pd_cmd)
            p2_cur = 0.5*(self.ps_cmd - self.pd_cmd)
            # simple rate limit in MPa/s -> MPa/step already handled by MPC; here be gentle
            stepS = self.rateS*0.5; stepD = self.rateD*0.5
            p1 = clip(self._p1_hold, p1_cur - stepS*dt, p1_cur + stepS*dt)
            p2 = clip(self._p2_hold, p2_cur - stepD*dt, p2_cur + stepD*dt)
            p1, p2 = clamp_box_p1p2(p1, p2, self.pmax)
            self.ps_cmd, self.pd_cmd = p1+p2, p1-p2
            return p1, p2

        # MPC planning
        u_opt = self._solve_mpc(self.theta, self.theta_ref)
        ps0 = float(u_opt[0]); pd0 = float(u_opt[1])
        p1 = 0.5*(ps0 + pd0); p2 = 0.5*(ps0 - pd0)
        p1, p2 = clamp_box_p1p2(p1, p2, self.pmax)

        # Update internal command (will feed delay line)
        self.ps_cmd = p1 + p2
        self.pd_cmd = p1 - p2

        # push command into delay history tail for next iteration
        if self.delay > 0:
            self.hist_sigma.pop(0); self.hist_sigma.append(self.ps_cmd)
            self.hist_delta.pop(0); self.hist_delta.append(self.pd_cmd)

        # record as potential hold setpoint
        self._p1_hold, self._p2_hold = p1, p2
        return p1, p2

    # ----------------- Spin -----------------
    def spin(self):
        r = rospy.Rate(self.rate_hz)
        # ensure delay buffers initialized
        self.hist_sigma = [self.ps_cmd]*self.delay
        self.hist_delta = [self.pd_cmd]*self.delay

        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            dt = max(1e-3, now - self._last_loop_t)
            self._last_loop_t = now

            res = self.compute(dt)
            if res is not None:
                p1, p2 = res
                self.pub_p12.publish(Vector3(x=p1, y=p2, z=0.0))
                if self.also_pub_mpa and self.pub_mpa is not None:
                    raw1 = int(round(p1 * self.raw_counts_per_MPa))
                    raw2 = int(round(p2 * self.raw_counts_per_MPa))
                    self.pub_mpa.publish(Vector3(x=raw1, y=raw2, z=0))
            r.sleep()

# ----------------- main -----------------
if __name__ == "__main__":
    rospy.init_node("neural_hammerstein_mpc_node")
    node = NeuralHammersteinMPCNode()
    node.spin()
