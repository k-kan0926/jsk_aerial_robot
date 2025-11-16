#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NARX + MPPI コントローラ (ROS Noetic, 100 Hz)
- 内部dt=meta['dt_est']≈0.005s、1周期(10ms)で2サブステップ進める (frame_skip=2)
- 実圧 /mpa_pressure (Vector3: x=p1,y=p2[MPa]) を購読して NARX先頭スライスへ反映
- 無い/欠測時はアクチュエータ簡易一次遅れ & 非対称レートで補完
- 出力 /mpa_cmd (Vector3) は p1/p2 を MPa→DACカウントに変換: counts = clip(MPa*4096/0.9, 0, 4096)
"""

from __future__ import annotations
import os, json, math, time, argparse
import numpy as np
import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

# ========= ユーティリティ =========

def clip(x, lo, hi): return lo if x < lo else (hi if x > hi else x)

def smoothstep_scurve(t):
    # 0..1 -> S-curve
    return 3*t*t - 2*t*t*t

def lowpass(prev, cur, alpha):
    # y = alpha*y + (1-alpha)*x
    return alpha*prev + (1.0-alpha)*cur

# ========= NARX（NumPy） =========

class NarxNumpy:
    def __init__(self, meta, weights, pmax, act="tanh"):
        self.L = int(meta["lags"])
        self.pmax = float(pmax)
        self.act = act

        self.mu  = np.asarray(meta["mu"], dtype=np.float32).reshape(1, -1)
        self.std = np.asarray(meta["std"], dtype=np.float32).reshape(1, -1)
        self.std[self.std==0] = 1.0

        W1,b1,W2,b2,W3,b3 = weights
        self.W1 = W1.astype(np.float32); self.b1 = b1.astype(np.float32)
        self.W2 = W2.astype(np.float32); self.b2 = b2.astype(np.float32)
        self.W3 = W3.astype(np.float32); self.b3 = b3.astype(np.float32)

        th_lo, th_hi = meta.get("theta_train_minmax", [-math.pi, math.pi])
        dz_lo, dz_hi = meta.get("dz_train_minmax", [-0.1, 0.1])
        self.th_lo = np.float32(th_lo); self.th_hi = np.float32(th_hi)
        self.dz_lo = np.float32(dz_lo); self.dz_hi = np.float32(dz_hi)

        pl = meta.get("pressure_limits", {})
        self.ps_rate_lim = float(pl.get("ps_rate_limit_MPa_s", 1e9))
        self.pd_rate_lim = float(pl.get("pd_rate_limit_MPa_s", 1e9))

        # index helpers
        self.i_th = lambda j: 6*j + 0
        self.i_ps = lambda j: 6*j + 1
        self.i_pd = lambda j: 6*j + 2
        self.i_dps= lambda j: 6*j + 3
        self.i_dpd= lambda j: 6*j + 4
        self.i_dz = lambda j: 6*j + 5

    def _act(self, x):
        return np.tanh(x) if self.act=="tanh" else np.maximum(x, 0)

    def forward_mlp(self, z_norm):
        # z_norm: (K, 6L)
        h1 = self._act(z_norm @ self.W1.T + self.b1)
        h2 = self._act(h1     @ self.W2.T + self.b2)
        y  = h2      @ self.W3.T + self.b3  # (K,2)
        return y

    def step(self, X, U, dt, use_meas=None, proxy=None):
        """
        X: (K,6L)  状態レジスタ
        U: (K,2)   [a,b] 0..1
        dt:        サブステップ dt (例 0.005)
        use_meas:  実測 (ps, pd) or None
        proxy:     dict  (一次遅れパラメータ), 実測が無いときに使用
        """
        K = X.shape[0]
        a = np.clip(U[:,0:1], 0.0, 1.0)
        b = np.clip(U[:,1:2], 0.0, 1.0)
        ps_tgt = self.pmax * (a + b)
        pd_tgt = self.pmax * (a - b)

        ps_prev = X[:, self.i_ps(0):self.i_ps(0)+1]
        pd_prev = X[:, self.i_pd(0):self.i_pd(0)+1]

        if use_meas is not None:  # 実測がある：目標ではなく実測をそのまま使う
            ps = use_meas[:,0:1]
            pd = use_meas[:,1:2]
            dps = (ps - ps_prev)/dt
            dpd = (pd - pd_prev)/dt
        else:
            # 一次遅れ＋非対称レート＋デッドゾーン（簡易アクチュエータ）
            cfg = proxy or {}
            tau_up_ps   = cfg.get("tau_up_ps",   0.015)
            tau_down_ps = cfg.get("tau_down_ps", 0.025)
            tau_up_pd   = cfg.get("tau_up_pd",   0.020)
            tau_down_pd = cfg.get("tau_down_pd", 0.030)
            eps_ps      = cfg.get("eps_ps",      0.003)
            eps_pd      = cfg.get("eps_pd",      0.003)
            rlim_ps     = cfg.get("ps_rate",     min(self.ps_rate_lim, 1.0))
            rlim_pd     = cfg.get("pd_rate",     min(self.pd_rate_lim, 0.5))

            e_ps = ps_tgt - ps_prev
            e_pd = pd_tgt - pd_prev

            alpha_ps = dt / (np.where(e_ps>0, tau_up_ps, tau_down_ps) + dt)
            alpha_pd = dt / (np.where(e_pd>0, tau_up_pd, tau_down_pd) + dt)

            ps = np.where(np.abs(e_ps) < eps_ps, ps_prev, ps_prev + alpha_ps*e_ps)
            pd = np.where(np.abs(e_pd) < eps_pd, pd_prev, pd_prev + alpha_pd*e_pd)

            # レート制限（厳しめ）
            ps = np.clip(ps, ps_prev - rlim_ps*dt, ps_prev + rlim_ps*dt)
            pd = np.clip(pd, pd_prev - rlim_pd*dt, pd_prev + rlim_pd*dt)

            dps = (ps - ps_prev)/dt
            dpd = (pd - pd_prev)/dt

        # NARX 入力
        z = X.copy()
        z[:, self.i_ps(0):self.i_ps(0)+1] = ps_prev  # 先頭スライスは「前の値」
        z[:, self.i_pd(0):self.i_pd(0)+1] = pd_prev
        z_norm = (z - self.mu) / self.std
        y = self.forward_mlp(z_norm)  # (K,2)
        th_next = np.clip(y[:,0:1], self.th_lo, self.th_hi)
        dz_next = np.clip(y[:,1:2], self.dz_lo, self.dz_hi)

        s_next = np.concatenate([th_next, ps, pd, dps, dpd, dz_next], axis=1)
        if self.L > 1:
            X_next = np.concatenate([s_next, X[:, :6*(self.L-1)]], axis=1)
        else:
            X_next = s_next
        return X_next

    def unpack_front(self, X):
        return (X[:, self.i_th(0)],
                X[:, self.i_ps(0)],
                X[:, self.i_pd(0)],
                X[:, self.i_dz(0)])

# ========= モデルロード（PyTorch state_dict） =========

def load_torch_weights(pt_path):
    import torch
    try:
        sd = torch.load(pt_path, map_location="cpu", weights_only=True)
    except TypeError:
        sd = torch.load(pt_path, map_location="cpu")
    W1 = sd["net.0.weight"].cpu().numpy(); b1 = sd["net.0.bias"].cpu().numpy()
    W2 = sd["net.2.weight"].cpu().numpy(); b2 = sd["net.2.bias"].cpu().numpy()
    W3 = sd["net.4.weight"].cpu().numpy(); b3 = sd["net.4.bias"].cpu().numpy()
    return (W1, b1, W2, b2, W3, b3)

# ========= MPPI =========

def mppi_step_numpy(model:NarxNumpy, x0, theta_refs, U_nom,
                    K, lam, sigma_u, dt_sub, frame_skip,
                    w_path, w_rate, w_z, w_term, w_box,
                    meas_pspd=None, proxy_cfg=None):
    """
    x0: (6L,)
    theta_refs: (T+1,)  ※T=horizon*frame_skip
    U_nom: (T,2)
    meas_pspd: None or (T,2) 実測 ps/pd（サブステップ時系列）※無ければNone
    """
    T = U_nom.shape[0]
    # 初期化
    X = np.repeat(x0.reshape(1, -1), K, axis=0).astype(np.float32)
    noise = sigma_u * np.random.randn(K, T, 2).astype(np.float32)
    J = np.zeros((K,), dtype=np.float32)

    for t in range(T):
        U_t = np.clip(U_nom[t] + noise[:, t, :], 0.0, 1.0)
        # サブステップをまとめて（同じUで進める）
        use_meas = None
        if meas_pspd is not None:
            use_meas = np.repeat(meas_pspd[t][None,:], K, axis=0)
        X_next = model.step(X, U_t, dt_sub, use_meas=use_meas, proxy=proxy_cfg)
        th, ps, pd, dz = model.unpack_front(X_next)
        theta_ref = theta_refs[t+1]
        ps_prev = X[:, model.i_ps(0)]
        pd_prev = X[:, model.i_pd(0)]

        cost = (w_path*(th - theta_ref)**2 +
                w_rate*(ps - ps_prev)**2 + w_rate*(pd - pd_prev)**2 +
                w_z*(dz**2))

        over_hi = np.clip(th - model.th_hi, 0.0, None)
        over_lo = np.clip(model.th_lo - th, 0.0, None)
        cost += w_box*(over_hi**2 + over_lo**2)

        J += cost
        X = X_next

    # 終端
    thT, _, _, _ = model.unpack_front(X)
    err_term = thT - theta_refs[-1]
    J += w_term*(err_term**2)
    over_hi = np.clip(thT - model.th_hi, 0.0, None)
    over_lo = np.clip(model.th_lo - thT, 0.0, None)
    J += w_box*(over_hi**2 + over_lo**2)

    Jmin = float(np.min(J))
    w = np.exp(-(J - Jmin) / max(1e-6, lam))
    w = w / (np.sum(w) + 1e-8)

    # 加重平均で U 更新
    U_new = np.zeros_like(U_nom)
    for t in range(T):
        U_t = np.clip(U_nom[t] + noise[:, t, :], 0.0, 1.0)
        U_new[t] = np.sum(w[:,None]*U_t, axis=0)

    u0 = U_new[0].copy()
    return u0, U_new, Jmin, float(np.mean(J))

# ========= ROS ノード =========

class NarxMppiNode:
    def __init__(self):
        # --- Parameters ---
        self.meta_path   = rospy.get_param("~meta")
        self.model_path  = rospy.get_param("~model")
        self.pmax        = float(rospy.get_param("~pmax", 0.7))
        self.rate_hz     = float(rospy.get_param("~rate", 100.0))
        self.horizon     = int(rospy.get_param("~horizon", 40))
        self.K           = int(rospy.get_param("~K", 192))
        self.lam         = float(rospy.get_param("~lambda", 0.9))
        self.sigma_u     = float(rospy.get_param("~sigma_u", 0.005))
        self.w_term      = float(rospy.get_param("~w_term", 12.0))
        self.w_path      = float(rospy.get_param("~w_path", 8.0))
        self.w_rate      = float(rospy.get_param("~w_rate", 0.03))
        self.w_z         = float(rospy.get_param("~w_z", 120.0))
        self.w_box       = float(rospy.get_param("~w_box", 500.0))
        self.theta_topic = rospy.get_param("~theta_topic", "/kinikun1/joint_states")
        self.theta_index = int(rospy.get_param("~theta_index", 2))
        self.theta_deg_default = float(rospy.get_param("~theta_target_deg", 0.0))
        self.target_topic= rospy.get_param("~target_topic", "/theta_target_deg")
        self.cmd_topic   = rospy.get_param("~mpa_cmd_topic", "/mpa_cmd")
        self.press_topic = rospy.get_param("~pressure_topic", "/mpa_pressure")
        self.cmd_gain    = float(rospy.get_param("~cmd_gain", 4096.0/0.9))
        self.cmd_clip    = float(rospy.get_param("~cmd_clip_max", 4096.0))
        self.act         = rospy.get_param("~act", "tanh")
        self.backend     = rospy.get_param("~backend", "numpy")  # "numpy" or "torch"

        # --- Load meta & model ---
        with open(self.meta_path, "r") as f:
            self.meta = json.load(f)
        self.dt_model = float(self.meta.get("dt_est", 0.005))
        self.frame_skip = max(1, int(round((1.0/self.rate_hz)/self.dt_model)))
        # 100Hz(10ms) × dt=0.005 → frame_skip=2
        T = self.horizon * self.frame_skip

        weights = load_torch_weights(self.model_path)
        self.model = NarxNumpy(self.meta, weights, self.pmax, act=self.act)

        # --- State buffer x (6L) ---
        L = self.model.L
        theta0 = 0.0; ps0 = 0.0; pd0 = 0.0; dps=0.0; dpd=0.0; dz=0.0
        s0 = np.array([theta0, ps0, pd0, dps, dpd, dz], dtype=np.float32)
        self.x = np.tile(s0, L).astype(np.float32)

        # --- Nominal sequence U ---
        self.U_nom = np.clip(0.5*np.ones((T,2), dtype=np.float32), 0.0, 1.0)

        # --- Targets & measurements ---
        self.theta_cur = 0.0
        self.theta_target_deg = self.theta_deg_default
        self.theta_target_deg_filt = self.theta_target_deg
        self.alpha_target = 0.9  # ローパス

        self.ps_meas = None  # MPa
        self.pd_meas = None
        self.last_press_t = 0.0

        # --- Subscribers/Publishers ---
        self.pub_cmd  = rospy.Publisher(self.cmd_topic, Vector3, queue_size=1)
        rospy.Subscriber(self.theta_topic, JointState, self.cb_joint)
        rospy.Subscriber(self.target_topic, Float32,  self.cb_target)
        rospy.Subscriber(self.press_topic,  Vector3,  self.cb_press)

        rospy.loginfo("narx_mppi_node: initialized. rate=%.1fHz frame_skip=%d (dt=%.4fs)",
                      self.rate_hz, self.frame_skip, self.dt_model)

    # ---- callbacks ----
    def cb_joint(self, msg:JointState):
        try:
            th = float(msg.position[self.theta_index])  # [rad]
            self.theta_cur = th
        except Exception:
            pass

    def cb_target(self, msg:Float32):
        self.theta_target_deg = float(msg.data)

    def cb_press(self, msg:Vector3):
        # x=p1, y=p2 [MPa]
        p1 = float(msg.x); p2 = float(msg.y)
        self.ps_meas = p1 + p2
        self.pd_meas = p1 - p2
        self.last_press_t = rospy.get_time()

    # ---- main step ----
    def step(self):
        self.x[self.model.i_th(0)] = np.float32(self.theta_cur)
        tnow = rospy.get_time()
        # 目標ローパス
        tnow = rospy.get_time()
        if (tnow - self.last_press_t) < 0.1 and (self.ps_meas is not None):
            self.x[self.model.i_ps(0)]  = np.float32(self.ps_meas)
            self.x[self.model.i_pd(0)]  = np.float32(self.pd_meas)
        self.theta_target_deg_filt = lowpass(self.theta_target_deg_filt, self.theta_target_deg, self.alpha_target)
        theta_ref = math.radians(self.theta_target_deg_filt)

        # S-curve 参照（サブステップ長）
        T = self.U_nom.shape[0]
        th0 = float(self.x[0])  # 先頭スライスの theta
        s = np.linspace(0.0, 1.0, T+1)
        ladder = th0 + (theta_ref - th0)*smoothstep_scurve(s).astype(np.float32)

        # 実測圧力をサブステップ列に展開（新鮮なら使用）
        meas = None
        if (tnow - self.last_press_t) < 0.1 and (self.ps_meas is not None):
            ps = np.float32(self.ps_meas); pd = np.float32(self.pd_meas)
            meas = np.tile(np.array([ps, pd], dtype=np.float32), (T,1))

        # MPPI
        u0, Unew, Jmin, Jmean = mppi_step_numpy(
            model=self.model, x0=self.x.copy(), theta_refs=ladder, U_nom=self.U_nom,
            K=self.K, lam=self.lam, sigma_u=self.sigma_u,
            dt_sub=self.dt_model, frame_skip=self.frame_skip,
            w_path=self.w_path, w_rate=self.w_rate, w_z=self.w_z, w_term=self.w_term, w_box=self.w_box,
            meas_pspd=meas,
            proxy_cfg=dict(tau_up_ps=0.015, tau_down_ps=0.025,
                           tau_up_pd=0.020, tau_down_pd=0.030,
                           eps_ps=0.003, eps_pd=0.003,
                           ps_rate=min(self.model.ps_rate_lim,1.0),
                           pd_rate=min(self.model.pd_rate_lim,0.5))
        )
        self.U_nom = Unew  # warm-start

        # u0 -> ps,pd -> p1,p2 [MPa]
        a = clip(float(u0[0]), 0.0, 1.0)
        b = clip(float(u0[1]), 0.0, 1.0)
        ps = self.pmax*(a+b); pd = self.pmax*(a-b)
        p1 = 0.5*(ps + pd); p2 = 0.5*(ps - pd)

        # NARX内部状態をサブステップ進めておく（実行系と同期）
        X = self.x.reshape(1,-1).astype(np.float32)
        meas_sub = None
        if meas is not None:
            meas_sub = np.tile(np.array([self.ps_meas, self.pd_meas], np.float32), (1,1))
        # 2サブステップ（frame_skip回）
        for _ in range(self.frame_skip):
            X = self.model.step(X, np.array([[a,b]], np.float32), self.dt_model, use_meas=meas_sub,
                                proxy=dict(tau_up_ps=0.015, tau_down_ps=0.025,
                                           tau_up_pd=0.020, tau_down_pd=0.030,
                                           eps_ps=0.003, eps_pd=0.003,
                                           ps_rate=min(self.model.ps_rate_lim,1.0),
                                           pd_rate=min(self.model.pd_rate_lim,0.5)))
        self.x = X.reshape(-1)

        # Publish command (MPa -> counts)
        c1 = int(clip(p1 * self.cmd_gain, 0.0, self.cmd_clip))
        c2 = int(clip(p2 * self.cmd_gain, 0.0, self.cmd_clip))
        self.pub_cmd.publish(Vector3(c1, c2, 0.0))

        # ログ（1Hz程度で抑制）
        if int(time.time()) % 1 == 0:
            rospy.loginfo_throttle(1.0, "[MPPI] th=%.2fdeg -> ref=%.2fdeg  u0=(%.3f,%.3f)  p1=%.3f p2=%.3f  Jmin=%.3f Jmean=%.3f",
                                   math.degrees(self.x[0]), math.degrees(theta_ref), a, b, p1, p2, Jmin, Jmean)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            t0 = time.time()
            self.step()
            # 締切ガード
            elapsed = time.time() - t0
            # 10msより遅いときは次周期で追いつく（ここではログのみ）
            if elapsed > 1.0/self.rate_hz:
                rospy.logwarn_throttle(2.0, "cycle overrun: %.1f ms", elapsed*1000.0)
            rate.sleep()

def main():
    rospy.init_node("narx_mppi_node")
    node = NarxMppiNode()
    node.spin()

if __name__ == "__main__":
    main()
