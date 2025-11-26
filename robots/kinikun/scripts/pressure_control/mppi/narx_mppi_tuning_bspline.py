#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
narx_mppi_tuning_bspline.py
NARX-MPPI制御 + ランダムBスプライン目標（ゲインチューニング用）

- 制御ロジックは narx_mppi_controller_production.py とほぼ同じ
- 目標角度は [-theta_range_deg, +theta_range_deg] からランダムに
  複数点を選び，waypoint_interval_s ごとに通過するよう B-spline で補間
- 評価用に RMSE などの簡単な指標を集計して終了時に表示
"""

import os, json, time, math, threading
from collections import deque
from typing import Tuple

import numpy as np
import rospy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState

import torch
import torch.nn as nn

try:
    # Bスプライン補間（なければ線形補間にフォールバック）
    from scipy.interpolate import make_interp_spline
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


# ========== Utility ==========

class SimpleKalmanFilter:
    """1次元カルマンフィルタ"""
    def __init__(self, process_noise=1e-5, measurement_noise=1e-3):
        self.Q = process_noise
        self.R = measurement_noise
        self.P = 1.0
        self.x = 0.0
    
    def update(self, z):
        P_pred = self.P + self.Q
        K = P_pred / (P_pred + self.R)
        self.x = self.x + K * (z - self.x)
        self.P = (1 - K) * P_pred
        return self.x
    
    def reset(self, x0):
        self.x = x0
        self.P = 1.0


class SafetyMonitor:
    """簡易安全監視（stuck チェックはオプション）"""
    def __init__(self, theta_rate_max=0.3, theta_abs_max=1.5,
                 enable_stuck_check=False, stuck_count_max=2000):
        self.last_theta = 0.0
        self.last_time = time.time()
        self.theta_rate_max = theta_rate_max  # rad/s
        self.theta_abs_max = theta_abs_max    # rad
        self.stuck_count = 0
        self.enable_stuck_check = enable_stuck_check
        self.stuck_count_max = stuck_count_max

    def check(self, theta):
        now = time.time()
        dt = now - self.last_time

        if abs(theta) > self.theta_abs_max:
            return False, f"Theta out of range: {theta:.3f} rad"

        if dt > 1e-6:
            rate = abs(theta - self.last_theta) / dt
            if rate > self.theta_rate_max:
                return False, f"Theta rate too high: {rate:.2f} rad/s"

        if self.enable_stuck_check:
            if abs(theta - self.last_theta) < 1e-6:
                self.stuck_count += 1
                if self.stuck_count > self.stuck_count_max:
                    return False, "Sensor appears stuck"
            else:
                self.stuck_count = 0

        self.last_theta = theta
        self.last_time = now
        return True, "OK"


class MLP_NARX(nn.Module):
    def __init__(self, in_dim, hidden=[256, 256], out_dim=1, dropout=0.0):
        super().__init__()
        layers = []
        d = in_dim
        for h in hidden:
            layers.append(nn.Linear(d, h))
            layers.append(nn.ReLU())
            if dropout > 0:
                layers.append(nn.Dropout(dropout))
            d = h
        layers.append(nn.Linear(d, out_dim))
        self.net = nn.Sequential(*layers)
    
    def forward(self, x):
        return self.net(x)


# ========== Controller ==========

class NARX_MPPITuningController:
    """
    NARX-MPPI制御ノード（ゲインチューニング用）
    - 制御ロジックは production 版と同じ
    - 目標は内部でランダムBスプライン生成
    """
    
    def __init__(self):
        rospy.init_node('narx_mppi_tuning_controller', anonymous=False)
        
        # ========== Parameters ==========
        self.model_dir = rospy.get_param("~model_dir", "models/out_narx2")
        self.rate_hz = float(rospy.get_param("~rate", 100.0))
        self.frame_skip = int(rospy.get_param("~frame_skip", 2))
        self.dt = float(self.frame_skip) / self.rate_hz
        
        # MPPI
        self.K = int(rospy.get_param("~K", 32))
        self.H = int(rospy.get_param("~horizon", 15))
        self.temperature = float(rospy.get_param("~lambda", 2.0))
        self.sigma_u = float(rospy.get_param("~sigma_u", 0.10))  # [MPa]
        
        # Costs
        self.w_tracking = float(rospy.get_param("~w_tracking", 30.0))
        self.w_smooth = float(rospy.get_param("~w_smooth", 0.05))
        self.w_effort = float(rospy.get_param("~w_effort", 0.01))
        self.w_constraint = float(rospy.get_param("~w_constraint", 500.0))
        
        # Physical limits
        self.p_max = float(rospy.get_param("~p_max", 0.70))
        self.dp_max = float(rospy.get_param("~dp_max", 3.5))  # MPa/s
        self.pressure_delay_s = float(rospy.get_param("~pressure_delay_s", 0.084))
        
        # Topics
        self.theta_topic = rospy.get_param("~theta_topic", "/kinikun1/joint_states")
        self.theta_index = int(rospy.get_param("~theta_index", 2))
        self.target_topic = rospy.get_param("~target_topic", "/theta_target_deg")
        self.pressure_topic = rospy.get_param("~pressure_topic", "/mpa_pressure")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/mpa_cmd")
        
        # B-spline random target parameters
        self.use_internal_ref = rospy.get_param("~use_internal_ref", True)
        self.theta_min_deg = float(rospy.get_param("~theta_min_deg", -30.0))
        self.theta_max_deg = float(rospy.get_param("~theta_max_deg", 30.0))
        self.n_waypoints = int(rospy.get_param("~n_waypoints", 8))
        self.waypoint_interval_s = float(rospy.get_param("~waypoint_interval_s", 10.0))
        self.seed = int(rospy.get_param("~seed", 42))
        
        # Evaluation (自動で止めて評価する時間[s], 0なら無限ループ)
        self.eval_duration_s = float(rospy.get_param("~eval_duration_s", 0.0))
        
        # Logging
        self.log_path = rospy.get_param("~log_csv", "")
        self.log_buffer = deque(maxlen=10000)
        self.log_thread = None
        self.log_file = None  # 後で close で落ちないように初期化
        
        # ========== Load Model ==========
        rospy.loginfo("[MPPI-TUNE] Loading model...")
        self.load_model()
        
        # ========== State Variables ==========
        self.lock = threading.Lock()
        
        # Filtered state
        self.theta_rad = 0.0
        self.theta_filter = SimpleKalmanFilter(process_noise=1e-5, measurement_noise=5e-4)
        
        # Command / measured pressures
        self.p1_cmd = 0.0
        self.p2_cmd = 0.0
        self.p1_meas = 0.0
        self.p2_meas = 0.0
        
        # Target (rad)
        self.theta_ref_rad = 0.0
        
        # History buffers (for NARX features, 実機側)
        maxlen = self.lags + 10
        self.hist_theta = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_p1_cmd = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_p2_cmd = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_dp1_dt = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_dp2_dt = deque([0.0] * maxlen, maxlen=maxlen)
        
        # Pressure delay buffer（将来拡張用）
        self.press_buf = deque(maxlen=200)  # (time, p1, p2)
        
        # Safety
        self.safety = SafetyMonitor(
            theta_rate_max=5.0,
            theta_abs_max=1.5,
            enable_stuck_check=False,
            stuck_count_max=2000
        )
        self.emergency_stop = False
        
        # Performance monitoring
        self.comp_time_buf = deque(maxlen=100)
        
        # 評価メトリクス蓄積（radベース）
        self.eval_started = False
        self.eval_sum_sq_error = 0.0
        self.eval_sum_abs_error = 0.0
        self.eval_max_abs_error = 0.0
        self.eval_samples = 0
        
        # 参照軌道（内部生成）
        self.ref_func = None   # t[s] -> theta_ref[rad]
        self.ref_total_duration = None
        self.ref_start_time = None
        
        # ========== ROS Interface ==========
        self.pub_cmd = rospy.Publisher(self.cmd_topic, Vector3, queue_size=1)
        self.pub_status = rospy.Publisher("/mppi/status", String, queue_size=1, latch=True)
        self.pub_target = rospy.Publisher(self.target_topic, Float32, queue_size=1)
        
        self.sub_theta = rospy.Subscriber(self.theta_topic, JointState,
                                          self.cb_theta, queue_size=10)
        self.sub_pressure = rospy.Subscriber(self.pressure_topic, Vector3,
                                             self.cb_pressure, queue_size=50)
        
        # Logging
        if self.log_path:
            self.setup_logging()
        
        # 参照軌道の準備
        if self.use_internal_ref:
            self.build_reference_trajectory()
        else:
            rospy.logwarn("[MPPI-TUNE] use_internal_ref=false ですが、外部目標のSubscriberは実装していません。")
        
        rospy.loginfo("[MPPI-TUNE] Initialization complete")
        rospy.loginfo(f"  Model: {self.model_dir}")
        rospy.loginfo(f"  Rate: {self.rate_hz} Hz (frame_skip={self.frame_skip}, dt={self.dt:.4f}s)")
        rospy.loginfo(f"  MPPI: K={self.K}, H={self.H}")
        rospy.loginfo(f"  Device: {self.device}")
    
    # ========== Model Loading ==========
    
    def load_model(self):
        meta_path = os.path.join(self.model_dir, 'narx_meta.json')
        model_path = os.path.join(self.model_dir, 'narx_model.pt')

        with open(meta_path, 'r') as f:
            self.meta = json.load(f)

        self.lags = self.meta['lags']
        self.delay = self.meta['delay']
        self.feat_cols = self.meta['feature_names_single_slice']
        self.mu = np.array(self.meta['mu'], dtype=np.float32)
        self.std = np.array(self.meta['std'], dtype=np.float32)
        self.hidden = self.meta['hidden']
        self.dropout = self.meta.get('dropout', 0.0)

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        if self.device.type == 'cpu':
            rospy.logwarn("[MPPI-TUNE] Running on CPU - performance will be limited!")

        in_dim = self.lags * len(self.feat_cols)

        self.model = MLP_NARX(
            in_dim,
            hidden=[self.hidden, self.hidden],
            out_dim=1,
            dropout=self.dropout
        )

        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.to(self.device)
        self.model.eval()

        rospy.loginfo(
            f"[MPPI-TUNE] Model loaded: lags={self.lags}, delay={self.delay}, "
            f"hidden={self.hidden}, dropout={self.dropout}"
        )
    
    # ========== Reference trajectory ==========
    
    def build_reference_trajectory(self):
        """ランダムなBスプライン参照軌道を生成"""
        rng = np.random.RandomState(self.seed)
        
        # 時刻: 0, T, 2T, ... でウェイポイント
        n_wp = max(2, self.n_waypoints)  # 少なくとも2点
        times = np.arange(n_wp + 1, dtype=np.float32) * self.waypoint_interval_s
        
        # 角度: 先頭は 0deg, 残りはランダム
        angles_deg = np.empty(n_wp + 1, dtype=np.float32)
        angles_deg[0] = 0.0
        angles_deg[1:] = rng.uniform(self.theta_min_deg, self.theta_max_deg, size=n_wp)
        
        angles_rad = np.deg2rad(angles_deg)
        
        if HAS_SCIPY and len(times) >= 4:
            k = 3
            if len(times) <= k:
                k = len(times) - 1
            spline = make_interp_spline(times, angles_rad, k=k)
            rospy.loginfo("[MPPI-TUNE] Reference: B-spline (SciPy) used")
            
            def ref_func(t):
                t_clamped = np.clip(t, times[0], times[-1])
                return float(spline(t_clamped))
        else:
            rospy.logwarn("[MPPI-TUNE] SciPy not available or too few points -> using linear interpolation")
            def ref_func(t):
                t_clamped = np.clip(t, times[0], times[-1])
                return float(np.interp(t_clamped, times, angles_rad))
        
        self.ref_func = ref_func
        self.ref_total_duration = float(times[-1])
        
        rospy.loginfo("[MPPI-TUNE] Reference waypoints (deg):")
        rospy.loginfo("  times  : " + ", ".join(f"{v:.1f}" for v in times))
        rospy.loginfo("  angles : " + ", ".join(f"{v:.1f}" for v in angles_deg))
        rospy.loginfo(f"[MPPI-TUNE] Total duration: {self.ref_total_duration:.1f} s")
    
    def update_reference(self, t_now):
        """現在時刻に対応する参照角度を更新"""
        if not self.use_internal_ref or self.ref_func is None or self.ref_start_time is None:
            return
        
        t_local = t_now - self.ref_start_time
        if self.ref_total_duration is not None and self.ref_total_duration > 0:
            # 周期的に繰り返す
            t_mod = math.fmod(t_local, self.ref_total_duration)
            if t_mod < 0:
                t_mod += self.ref_total_duration
        else:
            t_mod = max(0.0, t_local)
        
        theta_ref_rad = self.ref_func(t_mod)
        self.theta_ref_rad = theta_ref_rad
        
        # 可視化用に /theta_target_deg へも投げる（deg 表示）
        theta_ref_deg = math.degrees(theta_ref_rad)
        self.pub_target.publish(Float32(theta_ref_deg))
    
    # ========== ROS Callbacks ==========
    
    def cb_theta(self, msg: JointState):
        """関節角度のコールバック（カルマンフィルタ適用）"""
        if self.theta_index < len(msg.position):
            theta_raw = float(msg.position[self.theta_index])
            with self.lock:
                self.theta_rad = self.theta_filter.update(theta_raw)
                self.hist_theta.appendleft(self.theta_rad)
    
    def cb_pressure(self, msg: Vector3):
        """圧力測定値のコールバック"""
        t = rospy.get_time()
        p1 = float(msg.x)
        p2 = float(msg.y)
        with self.lock:
            self.press_buf.append((t, p1, p2))
            self.p1_meas = p1
            self.p2_meas = p2
    
    # ========== MPPI Core ==========
    
    def enforce_constraints(self, p1, p2, p1_prev, p2_prev, dt):
        """物理制約を適用（レート + ボックス）"""
        # Rate limit
        dp_max_step = self.dp_max * dt
        p1 = np.clip(p1, p1_prev - dp_max_step, p1_prev + dp_max_step)
        p2 = np.clip(p2, p2_prev - dp_max_step, p2_prev + dp_max_step)
        # Box constraint
        p1 = np.clip(p1, 0.0, self.p_max)
        p2 = np.clip(p2, 0.0, self.p_max)
        return p1, p2
    
    def cost_function(self, theta, theta_ref, p1, p2, p1_prev, p2_prev,
                      dp1, dp2, k, H):
        """コスト関数"""
        # Tracking error
        err = theta_ref - theta
        cost = self.w_tracking * (err ** 2)
        
        # Terminal cost
        if k == H - 1:
            cost += self.w_tracking * 0.5 * (err ** 2)
        
        # Smoothness
        cost += self.w_smooth * (dp1 ** 2 + dp2 ** 2)
        
        # Effort
        cost += self.w_effort * (p1 ** 2 + p2 ** 2)
        
        # Constraint violation (soft)
        viol = 0.0
        if p1 < 0:
            viol += (-p1) ** 2
        if p2 < 0:
            viol += (-p2) ** 2
        if p1 > self.p_max:
            viol += (p1 - self.p_max) ** 2
        if p2 > self.p_max:
            viol += (p2 - self.p_max) ** 2
        
        cost += self.w_constraint * viol
        return cost
    
    def rollout_batch(self, theta0, p1_0, p2_0, U):
        K, H = U.shape[0], U.shape[1]
        dt = self.dt

        theta_seq = np.zeros((K, H), dtype=np.float32)
        p1_seq = np.zeros((K, H), dtype=np.float32)
        p2_seq = np.zeros((K, H), dtype=np.float32)

        # 初期状態（各候補で共通）
        theta_k = np.full(K, theta0, dtype=np.float32)
        p1_k = np.full(K, p1_0, dtype=np.float32)
        p2_k = np.full(K, p2_0, dtype=np.float32)

        # 実機の履歴をベースに、各候補用の履歴を複製
        with self.lock:
            theta_hist0 = list(self.hist_theta)[:self.lags]
            p1_hist0 = list(self.hist_p1_cmd)[:self.lags]
            p2_hist0 = list(self.hist_p2_cmd)[:self.lags]
            dp1_hist0 = list(self.hist_dp1_dt)[:self.lags]
            dp2_hist0 = list(self.hist_dp2_dt)[:self.lags]

        def pad_hist(hist, fill):
            if len(hist) == 0:
                return [fill] * self.lags
            if len(hist) < self.lags:
                last = hist[-1]
                hist = hist + [last] * (self.lags - len(hist))
            return hist

        theta_hist0 = pad_hist(theta_hist0, theta0)
        p1_hist0 = pad_hist(p1_hist0, p1_0)
        p2_hist0 = pad_hist(p2_hist0, p2_0)
        dp1_hist0 = pad_hist(dp1_hist0, 0.0)
        dp2_hist0 = pad_hist(dp2_hist0, 0.0)

        # shape: (K, lags)
        theta_hist = np.tile(np.array(theta_hist0, dtype=np.float32), (K, 1))
        p1_hist = np.tile(np.array(p1_hist0, dtype=np.float32), (K, 1))
        p2_hist = np.tile(np.array(p2_hist0, dtype=np.float32), (K, 1))
        dp1_hist = np.tile(np.array(dp1_hist0, dtype=np.float32), (K, 1))
        dp2_hist = np.tile(np.array(dp2_hist0, dtype=np.float32), (K, 1))

        for h in range(H):
            # === 1) control を適用 ===
            dp1 = U[:, h, 0]
            dp2 = U[:, h, 1]

            # 前回値を控えてから更新（レート制約用）
            p1_prev = p1_k.copy()
            p2_prev = p2_k.copy()

            p1_k = p1_k + dp1
            p2_k = p2_k + dp2

            # 制約適用
            for i in range(K):
                p1_k[i], p2_k[i] = self.enforce_constraints(
                    p1_k[i], p2_k[i], p1_prev[i], p2_prev[i], dt
                )

            # dp/dt を計算
            dp1_dt = (p1_k - p1_prev) / dt
            dp2_dt = (p2_k - p2_prev) / dt

            # === 2) 履歴を更新（最新値を先頭に push） ===
            theta_hist = np.concatenate(
                [theta_k[:, None], theta_hist[:, :-1]], axis=1
            )
            p1_hist = np.concatenate(
                [p1_k[:, None], p1_hist[:, :-1]], axis=1
            )
            p2_hist = np.concatenate(
                [p2_k[:, None], p2_hist[:, :-1]], axis=1
            )
            dp1_hist = np.concatenate(
                [dp1_dt[:, None], dp1_hist[:, :-1]], axis=1
            )
            dp2_hist = np.concatenate(
                [dp2_dt[:, None], dp2_hist[:, :-1]], axis=1
            )

            # === 3) NARX用特徴量を構築 ===
            # feat_cols = [theta, p1_cmd, p2_cmd, dp1_cmd_dt, dp2_cmd_dt]
            X_chunks = []
            for k in range(self.lags):
                X_chunks.append(theta_hist[:, k][:, None])
                X_chunks.append(p1_hist[:, k][:, None])
                X_chunks.append(p2_hist[:, k][:, None])
                X_chunks.append(dp1_hist[:, k][:, None])
                X_chunks.append(dp2_hist[:, k][:, None])
            X_batch = np.concatenate(X_chunks, axis=1).astype(np.float32)

            # 正規化
            X_norm = (X_batch - self.mu) / (self.std + 1e-8)

            # === 4) バッチ推論 ===
            with torch.no_grad():
                Y_batch = self.model(torch.from_numpy(X_norm).to(self.device))
            theta_k = Y_batch.cpu().numpy().flatten()

            # === 5) ログ用に保存 ===
            theta_seq[:, h] = theta_k
            p1_seq[:, h] = p1_k
            p2_seq[:, h] = p2_k

        return theta_seq, p1_seq, p2_seq
    
    def mppi_step(self):
        """MPPI制御ステップ"""
        t_start = time.time()
        
        # 現在状態を取得
        with self.lock:
            theta = self.theta_rad
            theta_ref = self.theta_ref_rad
            p1_prev = self.p1_cmd
            p2_prev = self.p2_cmd
        
        # Safety check
        is_safe, msg = self.safety.check(theta)
        if not is_safe:
            rospy.logerr(f"[MPPI-TUNE] Safety violation: {msg}")
            self.emergency_stop = True
            self.publish_cmd(0.0, 0.0)
            return
        
        # 制御ノイズサンプル U: (K, H, 2)
        U = np.random.normal(
            loc=0.0,
            scale=self.sigma_u,
            size=(self.K, self.H, 2)
        ).astype(np.float32)
        
        # Rollout
        theta_seq, p1_seq, p2_seq = self.rollout_batch(theta, p1_prev, p2_prev, U)
        
        # コスト計算
        dt = self.dt
        J = np.zeros(self.K, dtype=np.float32)
        
        for i in range(self.K):
            cost = 0.0
            p1_h, p2_h = p1_prev, p2_prev
            for h in range(self.H):
                dp1 = U[i, h, 0]
                dp2 = U[i, h, 1]
                cost += self.cost_function(
                    theta_seq[i, h], theta_ref,
                    p1_seq[i, h], p2_seq[i, h],
                    p1_h, p2_h, dp1, dp2, h, self.H
                )
                p1_h = p1_seq[i, h]
                p2_h = p2_seq[i, h]
            J[i] = cost
        
        # MPPI weight computation
        beta = np.min(J)
        w = np.exp(-(J - beta) / max(1e-6, self.temperature))
        w_sum = np.sum(w) + 1e-9
        
        # 重み付き平均で最初の入力方向を決定
        dU = np.sum(w[:, None, None] * U, axis=0) / w_sum  # (H, 2)
        
        # Apply first control
        dp1_cmd, dp2_cmd = dU[0, 0], dU[0, 1]
        p1_cmd = p1_prev + dp1_cmd
        p2_cmd = p2_prev + dp2_cmd
        
        # Final constraint enforcement
        p1_cmd, p2_cmd = self.enforce_constraints(p1_cmd, p2_cmd, p1_prev, p2_prev, dt)
        
        # Publish
        self.publish_cmd(p1_cmd, p2_cmd)
        
        # Update history（実機側の履歴）
        with self.lock:
            self.p1_cmd = p1_cmd
            self.p2_cmd = p2_cmd
            self.hist_p1_cmd.appendleft(p1_cmd)
            self.hist_p2_cmd.appendleft(p2_cmd)
            if len(self.hist_p1_cmd) > 1:
                dp1_dt = (self.hist_p1_cmd[0] - self.hist_p1_cmd[1]) / dt
                dp2_dt = (self.hist_p2_cmd[0] - self.hist_p2_cmd[1]) / dt
            else:
                dp1_dt, dp2_dt = 0.0, 0.0
            self.hist_dp1_dt.appendleft(dp1_dt)
            self.hist_dp2_dt.appendleft(dp2_dt)
        
        # Performance monitoring
        comp_time = time.time() - t_start
        self.comp_time_buf.append(comp_time)
        
        # Logging + 評価メトリクス
        if self.log_path or self.eval_started:
            err = theta_ref - theta
            if self.eval_started:
                self.eval_sum_sq_error += err * err
                self.eval_sum_abs_error += abs(err)
                if abs(err) > self.eval_max_abs_error:
                    self.eval_max_abs_error = abs(err)
                self.eval_samples += 1
            
            if self.log_path:
                self.log_buffer.append({
                    't': rospy.get_time(),
                    'theta': theta,
                    'theta_ref': theta_ref,
                    'error': err,
                    'p1_cmd': p1_cmd,
                    'p2_cmd': p2_cmd,
                    'p1_meas': self.p1_meas,
                    'p2_meas': self.p2_meas,
                    'J_min': float(np.min(J)),
                    'J_mean': float(np.mean(J)),
                    'comp_time_ms': comp_time * 1000.0
                })
    
    # ========== Command Publishing ==========
    
    def publish_cmd(self, p1, p2):
        """圧力指令を出力（ハード側スケーリング込み）"""
        msg = Vector3()
        # MPa → DAC値への変換（4096 / 0.9）
        msg.x = float(p1) * 4096.0 / 0.9
        msg.y = float(p2) * 4096.0 / 0.9
        msg.z = 0.0
        self.pub_cmd.publish(msg)
    
    # ========== Logging ==========
    
    def setup_logging(self):
        import csv
        
        dirpath = os.path.dirname(self.log_path)
        if dirpath:
            os.makedirs(dirpath, exist_ok=True)
        
        self.log_file = open(self.log_path, 'w', newline='')
        self.log_writer = csv.DictWriter(self.log_file, fieldnames=[
            't', 'theta', 'theta_ref', 'error',
            'p1_cmd', 'p2_cmd', 'p1_meas', 'p2_meas',
            'J_min', 'J_mean', 'comp_time_ms'
        ])
        self.log_writer.writeheader()
        
        # Start logging thread
        self.log_thread = threading.Thread(target=self.logging_worker, daemon=True)
        self.log_thread.start()
        
        rospy.loginfo(f"[MPPI-TUNE] Logging to: {self.log_path}")
    
    def logging_worker(self):
        rate = rospy.Rate(10)  # 10Hz 書き込み
        while not rospy.is_shutdown():
            if len(self.log_buffer) > 0:
                batch = []
                while len(self.log_buffer) > 0 and len(batch) < 100:
                    batch.append(self.log_buffer.popleft())
                try:
                    self.log_writer.writerows(batch)
                    self.log_file.flush()
                except Exception as e:
                    rospy.logerr(f"[MPPI-TUNE] Logging error: {e}")
            rate.sleep()
    
    # ========== Evaluation report ==========
    
    def report_eval_metrics(self):
        if not self.eval_started or self.eval_samples <= 0:
            rospy.logwarn("[MPPI-TUNE] No evaluation samples collected.")
            return
        
        rmse_rad = math.sqrt(self.eval_sum_sq_error / self.eval_samples)
        mae_rad = self.eval_sum_abs_error / self.eval_samples
        max_err_rad = self.eval_max_abs_error
        
        rmse_deg = math.degrees(rmse_rad)
        mae_deg = math.degrees(mae_rad)
        max_err_deg = math.degrees(max_err_rad)
        
        rospy.loginfo("[MPPI-TUNE] ===== Evaluation summary =====")
        rospy.loginfo(f"[MPPI-TUNE] Samples   : {self.eval_samples}")
        rospy.loginfo(f"[MPPI-TUNE] RMSE      : {rmse_deg:.3f} deg")
        rospy.loginfo(f"[MPPI-TUNE] MAE       : {mae_deg:.3f} deg")
        rospy.loginfo(f"[MPPI-TUNE] Max |err| : {max_err_deg:.3f} deg")
        rospy.loginfo("[MPPI-TUNE] =================================")
    
    # ========== Main Loop ==========
    
    def spin(self):
        """メインループ"""
        rate = rospy.Rate(self.rate_hz)
        frame_count = 0
        
        rospy.loginfo("[MPPI-TUNE] Starting control loop...")
        self.pub_status.publish(String("running"))
        
        # Warmup period
        warmup_duration = 2.0  # seconds
        warmup_start = rospy.get_time()
        
        rospy.loginfo(f"[MPPI-TUNE] Warmup for {warmup_duration}s...")
        while not rospy.is_shutdown():
            if rospy.get_time() - warmup_start > warmup_duration:
                break
            self.publish_cmd(0.0, 0.0)
            rate.sleep()
        
        rospy.loginfo("[MPPI-TUNE] Control active!")
        self.ref_start_time = rospy.get_time()
        self.eval_started = True
        eval_start = self.ref_start_time
        
        try:
            while not rospy.is_shutdown():
                if self.emergency_stop:
                    self.publish_cmd(0.0, 0.0)
                    rospy.logerr("[MPPI-TUNE] Emergency stop active")
                    rate.sleep()
                    continue
                
                # 参照更新
                self.update_reference(rospy.get_time())
                
                # Control update
                if frame_count % self.frame_skip == 0:
                    self.mppi_step()
                
                frame_count += 1
                
                # Performance report (every 10s)
                if frame_count % int(self.rate_hz * 10) == 0:
                    if len(self.comp_time_buf) > 0:
                        avg_time = np.mean(self.comp_time_buf)
                        max_time = np.max(self.comp_time_buf)
                        rospy.loginfo(
                            f"[MPPI-TUNE] Comp time: avg={avg_time*1000:.1f}ms, "
                            f"max={max_time*1000:.1f}ms"
                        )
                
                # 自動終了（評価時間が指定されている場合）
                if self.eval_duration_s > 0.0:
                    if rospy.get_time() - eval_start > self.eval_duration_s:
                        rospy.loginfo("[MPPI-TUNE] Evaluation duration reached. Stopping.")
                        break
                
                rate.sleep()
        
        except rospy.ROSInterruptException:
            pass
        
        finally:
            rospy.loginfo("[MPPI-TUNE] Shutting down...")
            self.publish_cmd(0.0, 0.0)
            self.pub_status.publish(String("stopped"))
            self.report_eval_metrics()
            if self.log_file:
                self.log_file.close()


def main():
    controller = NARX_MPPITuningController()
    controller.spin()

if __name__ == '__main__':
    main()
