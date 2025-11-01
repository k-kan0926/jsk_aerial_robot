#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import math
import numpy as np
import torch
import pandas as pd  # 起動時のcontext読みで使うかもしれないので入れておく

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from pathlib import Path


class NarxNumpy(object):
    """
    ROS版用のNARXラッパ
    - 出力を必ず学習範囲にクリップする
    - a,b -> ps,pd もacados版と揃える
    """
    def __init__(self, meta, weights, pmax, dt, act="tanh"):
        self.meta = meta
        self.weights = weights
        self.pmax = float(pmax)
        self.dt = float(dt)
        self.L = int(meta["lags"])
        self.mu = np.asarray(meta["mu"], dtype=np.float32).reshape(1, -1)
        self.std = np.asarray(meta["std"], dtype=np.float32).reshape(1, -1)
        self.act = act

        # 学習時の範囲
        self.theta_lo, self.theta_hi = -math.pi, math.pi
        if "theta_train_minmax" in meta:
            self.theta_lo = float(meta["theta_train_minmax"][0])
            self.theta_hi = float(meta["theta_train_minmax"][1])

        self.dz_lo, self.dz_hi = -0.1, 0.1
        if "dz_train_minmax" in meta:
            self.dz_lo = float(meta["dz_train_minmax"][0])
            self.dz_hi = float(meta["dz_train_minmax"][1])

        self.ps_rate = float(meta.get("pressure_limits", {}).get("ps_rate_limit_MPa_s", 999.0))
        self.pd_rate = float(meta.get("pressure_limits", {}).get("pd_rate_limit_MPa_s", 999.0))

        self.idx_theta = lambda j: 6*j + 0
        self.idx_ps    = lambda j: 6*j + 1
        self.idx_pd    = lambda j: 6*j + 2
        self.idx_dps   = lambda j: 6*j + 3
        self.idx_dpd   = lambda j: 6*j + 4
        self.idx_dz    = lambda j: 6*j + 5

    @staticmethod
    def _act_fn(x, act):
        if act == "tanh":
            return np.tanh(x)
        elif act == "relu":
            return np.maximum(x, 0.0)
        else:
            raise ValueError("act must be tanh or relu")

    def step(self, X, U):
        """
        X: (B, 6L)
        U: (B, 2) a,b in [0,1]
        """
        B = X.shape[0]
        L = self.L
        dt = self.dt

        U = np.clip(U, 0.0, 1.0)
        a = U[:, 0:1]
        b = U[:, 1:2]
        ps = self.pmax * (a + b)
        pd = self.pmax * (a - b)

        ps_prev = X[:, self.idx_ps(0):self.idx_ps(0)+1]
        pd_prev = X[:, self.idx_pd(0):self.idx_pd(0)+1]

        dps = (ps - ps_prev) / dt
        dpd = (pd - pd_prev) / dt

        # rate limitあれば適用
        if self.ps_rate < 900.0:
            dps = np.clip(dps, -self.ps_rate, self.ps_rate)
            ps = ps_prev + dps * dt
        if self.pd_rate < 900.0:
            dpd = np.clip(dpd, -self.pd_rate, self.pd_rate)
            pd = pd_prev + dpd * dt

        z = X.copy()
        z_norm = (z - self.mu) / self.std

        W1, b1, W2, b2, W3, b3 = self.weights
        h1 = self._act_fn(z_norm @ W1.T + b1[None, :], self.act)
        h2 = self._act_fn(h1 @ W2.T + b2[None, :], self.act)
        y  = h2 @ W3.T + b3[None, :]

        theta_next = np.clip(y[:, 0:1], self.theta_lo, self.theta_hi)
        dz_next    = np.clip(y[:, 1:2], self.dz_lo,    self.dz_hi)

        s_next = np.concatenate([theta_next, ps, pd, dps, dpd, dz_next], axis=1)
        if L > 1:
            X_next = np.concatenate([s_next, X[:, :6*(L-1)]], axis=1)
        else:
            X_next = s_next
        return X_next

    def unpack_front(self, X):
        th = X[:, self.idx_theta(0)]
        ps = X[:, self.idx_ps(0)]
        pd = X[:, self.idx_pd(0)]
        dz = X[:, self.idx_dz(0)]
        return th, ps, pd, dz


def load_meta(path):
    return json.loads(Path(path).read_text())


def load_torch_weights(path):
    try:
        sd = torch.load(path, map_location="cpu", weights_only=True)
    except TypeError:
        sd = torch.load(path, map_location="cpu")
    W1 = sd["net.0.weight"].cpu().numpy()
    b1 = sd["net.0.bias"].cpu().numpy()
    W2 = sd["net.2.weight"].cpu().numpy()
    b2 = sd["net.2.bias"].cpu().numpy()
    W3 = sd["net.4.weight"].cpu().numpy()
    b3 = sd["net.4.bias"].cpu().numpy()
    return (W1, b1, W2, b2, W3, b3)


def smooth_theta_ref(theta0, theta_target, T):
    t = np.linspace(0., 1., T+1)
    s = 3 * t**2 - 2 * t**3
    return theta0 + (theta_target - theta0) * s


def mppi_step(model, x0, theta_refs, U_nom,
              K, lam, sigma_u,
              w_path, w_rate, w_z, w_term, w_box):
    T = U_nom.shape[0]
    X = np.repeat(x0[None, :], K, axis=0)
    noise = sigma_u * np.random.randn(K, T, 2).astype(np.float32)
    J = np.zeros((K,), dtype=np.float32)

    theta_lo, theta_hi = model.theta_lo, model.theta_hi

    for t in range(T):
        U_t = U_nom[t][None, :] + noise[:, t, :]
        X_next = model.step(X, U_t)
        theta, ps, pd, dz = model.unpack_front(X_next)
        theta_ref_t = theta_refs[t+1]

        ps_prev = X[:, model.idx_ps(0)]
        pd_prev = X[:, model.idx_pd(0)]

        cost_t = (w_path * (theta - theta_ref_t)**2
                  + w_rate * (ps - ps_prev)**2
                  + w_rate * (pd - pd_prev)**2
                  + w_z    * (dz**2))

        over_hi = np.maximum(0.0, theta - theta_hi)
        over_lo = np.maximum(0.0, theta_lo - theta)
        cost_t += w_box * (over_hi**2 + over_lo**2)

        J += cost_t.astype(np.float32)
        X = X_next

    theta, _, _, _ = model.unpack_front(X)
    err_term = theta - theta_refs[-1]
    J += (w_term * err_term**2).astype(np.float32)

    over_hi = np.maximum(0.0, theta - theta_hi)
    over_lo = np.maximum(0.0, theta_lo - theta)
    J += w_box * (over_hi**2 + over_lo**2)

    J_min = float(np.min(J))
    w = np.exp(-(J - J_min) / max(1e-6, lam))
    w /= (np.sum(w) + 1e-8)

    U_new = np.zeros_like(U_nom)
    for t in range(T):
        U_t_samp = U_nom[t][None, :] + noise[:, t, :]
        U_t_samp = np.clip(U_t_samp, 0.0, 1.0)
        U_new[t] = np.sum(w[:, None] * U_t_samp, axis=0)
    u0 = U_new[0].copy()
    return u0, U_new, J_min, float(np.mean(J))


def pressure_to_cmd(p_MPa, reg_pmax=0.9, cmd_max=4096.0):
    # よくあるやつ: 0〜reg_pmax MPa → 0〜cmd_max
    p = np.clip(float(p_MPa), 0.0, reg_pmax)
    return p / reg_pmax * cmd_max
    # もし本当に「p * 0.9 / 4096」で送りたいなら↑を↓に変える
    # return p * 0.9 / 4096.0


class NarxMppiRosNode(object):
    def __init__(self):
        # params
        meta_path  = rospy.get_param("~meta")
        model_path = rospy.get_param("~model")
        self.theta_target_deg = rospy.get_param("~theta_target_deg", 20.0)
        self.pmax_model = rospy.get_param("~pmax", 0.7)
        self.horizon = rospy.get_param("~horizon", 15)
        self.K = rospy.get_param("~K", 256)
        self.lam = rospy.get_param("~lambda", 1.0)
        self.sigma_u = rospy.get_param("~sigma_u", 0.15)
        self.rate_hz = rospy.get_param("~rate", 50.0)
        self.theta_topic = rospy.get_param("~theta_topic", "/joint_states")
        self.theta_index = rospy.get_param("~theta_index", 0)
        self.mpa_cmd_topic = rospy.get_param("~mpa_cmd_topic", "/mpa_cmd")
        self.reg_pmax = rospy.get_param("~reg_pmax", 0.9)
        self.reg_cmdmax = rospy.get_param("~reg_cmdmax", 4096.0)

        self.w_term = rospy.get_param("~w_term", 8.0)
        self.w_path = rospy.get_param("~w_path", 5.0)
        self.w_rate = rospy.get_param("~w_rate", 0.10)
        self.w_z    = rospy.get_param("~w_z", 0.10)
        self.w_box  = rospy.get_param("~w_box", 200.0)

        # load model
        meta = load_meta(meta_path)
        weights = load_torch_weights(model_path)
        dt = float(meta["dt_est"])
        self.model = NarxNumpy(meta, weights,
                               pmax=self.pmax_model,
                               dt=dt,
                               act="tanh")

        # state buffer init
        theta0 = 0.0
        s = np.array([theta0, 0.3, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.x = np.tile(s, (self.model.L,))  # (6L,)

        # nominal input
        a0 = (0.3 + 0.0) / (2 * self.pmax_model)
        b0 = (0.3 - 0.0) / (2 * self.pmax_model)
        self.U_nom = np.tile(np.clip([a0, b0], 0.0, 1.0), (self.horizon, 1)).astype(np.float32)

        # ROS I/O
        self.sub = rospy.Subscriber(self.theta_topic, JointState, self.joint_cb, queue_size=1)
        self.pub = rospy.Publisher(self.mpa_cmd_topic, Vector3, queue_size=1)

        self.last_theta_meas = None
        self.last_ps = 0.3
        self.last_pd = 0.0
        rospy.loginfo("narx_mppi_node: ready.")

    def joint_cb(self, msg: JointState):
        try:
            th = msg.position[self.theta_index]
            self.last_theta_meas = float(th)
        except Exception:
            pass

    def spin(self):
        r = rospy.Rate(self.rate_hz)
        step = 0
        while not rospy.is_shutdown():
            dt = self.model.dt

            # θがまだ来てないなら何もしない
            if self.last_theta_meas is not None:
                # ---- 観測をバッファに反映（先頭スライスだけ）----
                # 現在のps/pdは前回のコマンドから再構成
                ps_obs = getattr(self, "last_ps", 0.3)
                pd_obs = getattr(self, "last_pd", 0.0)
                # 1サイクル差分でdps/dpd
                dps_obs = 0.0
                dpd_obs = 0.0
                # dzは観測がなければ0
                dz_obs = 0.0

                s0 = np.array([self.last_theta_meas,
                               ps_obs,
                               pd_obs,
                               dps_obs,
                               dpd_obs,
                               dz_obs], dtype=np.float32)
                if self.model.L > 1:
                    self.x = np.concatenate([s0, self.x[:6*(self.model.L-1)]], axis=0)
                else:
                    self.x = s0

            # ---- MPPIを1回回す ----
            theta_cur = float(self.x[0])
            theta_target = math.radians(self.theta_target_deg)
            theta_refs = smooth_theta_ref(theta_cur, theta_target, self.horizon)
            # 参照も守る
            theta_refs = np.clip(theta_refs,
                                 self.model.theta_lo + 1e-3,
                                 self.model.theta_hi - 1e-3)

            u0, self.U_nom, Jmin, Jmean = mppi_step(
                model=self.model,
                x0=self.x,
                theta_refs=theta_refs,
                U_nom=self.U_nom,
                K=self.K,
                lam=self.lam,
                sigma_u=self.sigma_u,
                w_path=self.w_path,
                w_rate=self.w_rate,
                w_z=self.w_z,
                w_term=self.w_term,
                w_box=self.w_box,
            )

            # ---- 選ばれたu0を「実機」へ出す変換 ----
            # a,b→ps,pd
            a = float(np.clip(u0[0], 0.0, 1.0))
            b = float(np.clip(u0[1], 0.0, 1.0))
            ps = self.pmax_model * (a + b)
            pd = self.pmax_model * (a - b)
            p1 = 0.5 * (ps + pd)
            p2 = 0.5 * (ps - pd)

            # 保持しておく（次回の観測補完に使う）
            self.last_ps = ps
            self.last_pd = pd

            # MPa -> コマンド値
            cmd = Vector3()
            cmd.x = pressure_to_cmd(p1, reg_pmax=self.reg_pmax, cmd_max=self.reg_cmdmax)
            cmd.y = pressure_to_cmd(p2, reg_pmax=self.reg_pmax, cmd_max=self.reg_cmdmax)
            cmd.z = 0.0
            self.pub.publish(cmd)

            if step % 20 == 0:
                rospy.loginfo("[mppi] th=%.2fdeg -> ref=%.2fdeg  u0=(%.3f,%.3f)  Jmin=%.3f",
                              math.degrees(theta_cur),
                              math.degrees(theta_refs[1]),
                              a, b, Jmin)

            step += 1
            r.sleep()


def main():
    rospy.init_node("narx_mppi_node")
    node = NarxMppiRosNode()
    node.spin()


if __name__ == "__main__":
    main()
