#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
narx_mppi_controller_enhanced.py
機能拡張版MPPI制御ノード

新機能:
- B-spline軌道平滑化
- 適応的パラメータ調整
- 予測軌道追従
- パフォーマンスモニタリング強化
"""
import os, json, time, math, threading
from collections import deque
from typing import Tuple, Optional, List
import numpy as np
from scipy.interpolate import BSpline, make_interp_spline
from scipy import signal

import rospy
from std_msgs.msg import Float32, Float32MultiArray, String
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState

import torch
import torch.nn as nn

# ==================== Utility Classes ====================

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
    """安全監視システム"""
    def __init__(self, theta_rate_max=0.3, theta_abs_max=1.5,
                 enable_stuck_check=False, stuck_count_max=2000):
        self.last_theta = 0.0
        self.last_time = time.time()
        self.theta_rate_max = theta_rate_max
        self.theta_abs_max = theta_abs_max
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


# ==================== NARX Model ====================

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


# ==================== Enhanced MPPI Controller ====================

class EnhancedMPPIController:
    """機能拡張版MPPI制御ノード"""
    
    def __init__(self):
        rospy.init_node('enhanced_mppi_controller', anonymous=False)
        
        # ========== Base Parameters ==========
        self.model_dir = rospy.get_param("~model_dir", "models/out_narx2")
        self.rate_hz = float(rospy.get_param("~rate", 100.0))
        self.frame_skip = int(rospy.get_param("~frame_skip", 2))
        self.dt = float(self.frame_skip) / self.rate_hz
        
        # ========== MPPI Core Parameters ==========
        self.K = int(rospy.get_param("~K", 256))  # 増加
        self.H = int(rospy.get_param("~horizon", 30))  # 増加
        self.temperature = float(rospy.get_param("~lambda", 1.0))
        self.sigma_u = float(rospy.get_param("~sigma_u", 0.15))
        
        # ========== Enhanced Features ==========
        # B-spline
        self.use_bspline = rospy.get_param("~use_bspline", False)
        self.bspline_control_points = int(rospy.get_param("~bspline_control_points", 8))
        self.bspline_degree = int(rospy.get_param("~bspline_degree", 3))
        
        # Adaptive sigma
        self.use_adaptive_sigma = rospy.get_param("~use_adaptive_sigma", False)
        self.sigma_u_min = float(rospy.get_param("~sigma_u_min", 0.05))
        self.sigma_u_max = float(rospy.get_param("~sigma_u_max", 0.30))
        self.sigma_adaptation_rate = float(rospy.get_param("~sigma_adaptation_rate", 0.1))
        
        # Trajectory prediction
        self.use_trajectory_prediction = rospy.get_param("~use_trajectory_prediction", False)
        self.prediction_method = rospy.get_param("~prediction_method", "linear")
        
        # ========== Cost Weights ==========
        self.w_tracking = float(rospy.get_param("~w_tracking", 50.0))
        self.w_smooth = float(rospy.get_param("~w_smooth", 0.05))
        self.w_effort = float(rospy.get_param("~w_effort", 0.01))
        self.w_constraint = float(rospy.get_param("~w_constraint", 500.0))
        self.w_terminal = float(rospy.get_param("~w_terminal", 100.0))
        
        # ========== Physical limits ==========
        self.p_max = float(rospy.get_param("~p_max", 0.70))
        self.dp_max = float(rospy.get_param("~dp_max", 3.5))
        
        # ========== Topics ==========
        self.theta_topic = rospy.get_param("~theta_topic", "/kinikun1/joint_states")
        self.theta_index = int(rospy.get_param("~theta_index", 2))
        self.target_topic = rospy.get_param("~target_topic", "/theta_target_deg")
        self.trajectory_topic = rospy.get_param("~trajectory_topic", "/theta_trajectory")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/mpa_cmd")
        self.pressure_topic = rospy.get_param("~pressure_topic", "/mpa_pressure")
        
        # ========== Logging ==========
        self.log_path = rospy.get_param("~log_csv", "")
        self.log_buffer = deque(maxlen=10000)
        self.log_thread = None
        self.log_file = None
        
        # ========== Initialize Components ==========
        rospy.loginfo("[Enhanced MPPI] Loading model...")
        self.load_model()
        
        # State variables
        self.lock = threading.Lock()
        self.theta_rad = 0.0
        self.theta_filter = SimpleKalmanFilter(process_noise=1e-5, measurement_noise=5e-4)
        
        self.p1_cmd = 0.0
        self.p2_cmd = 0.0
        self.p1_meas = 0.0
        self.p2_meas = 0.0
        
        self.theta_ref_rad = 0.0
        self.theta_trajectory = None  # 予測軌道
        self.target_history = deque(maxlen=100)
        
        # History buffers
        maxlen = self.lags + 10
        self.hist_theta = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_p1_cmd = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_p2_cmd = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_dp1_dt = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_dp2_dt = deque([0.0] * maxlen, maxlen=maxlen)
        
        # B-spline control points (初期化)
        if self.use_bspline:
            self.control_points = np.zeros((self.bspline_control_points, 2))
        
        # Safety
        self.safety = SafetyMonitor(
            theta_rate_max=5.0,
            theta_abs_max=1.5,
            enable_stuck_check=False
        )
        self.emergency_stop = False
        
        # Performance monitoring
        self.comp_time_buf = deque(maxlen=100)
        self.tracking_error_buf = deque(maxlen=100)
        
        # ========== ROS Interface ==========
        self.setup_ros_interface()
        
        if self.log_path:
            self.setup_logging()
        
        self.print_configuration()
    
    def print_configuration(self):
        """設定情報を表示"""
        rospy.loginfo("="*70)
        rospy.loginfo("[Enhanced MPPI Controller Configuration]")
        rospy.loginfo("="*70)
        rospy.loginfo(f"  Model: {self.model_dir}")
        rospy.loginfo(f"  Rate: {self.rate_hz} Hz, dt={self.dt:.4f}s")
        rospy.loginfo(f"  MPPI: K={self.K}, H={self.H}, lambda={self.temperature}")
        rospy.loginfo(f"  Device: {self.device}")
        rospy.loginfo("  Enhanced Features:")
        rospy.loginfo(f"    - B-spline: {self.use_bspline}")
        if self.use_bspline:
            rospy.loginfo(f"      Control points: {self.bspline_control_points}")
            rospy.loginfo(f"      Degree: {self.bspline_degree}")
        rospy.loginfo(f"    - Adaptive Sigma: {self.use_adaptive_sigma}")
        if self.use_adaptive_sigma:
            rospy.loginfo(f"      Range: [{self.sigma_u_min}, {self.sigma_u_max}]")
        rospy.loginfo(f"    - Trajectory Prediction: {self.use_trajectory_prediction}")
        if self.use_trajectory_prediction:
            rospy.loginfo(f"      Method: {self.prediction_method}")
        rospy.loginfo("="*70)
    
    def setup_ros_interface(self):
        """ROS通信の設定"""
        # Publishers
        self.pub_cmd = rospy.Publisher(self.cmd_topic, Vector3, queue_size=1)
        self.pub_status = rospy.Publisher("/mppi/status", String, queue_size=1, latch=True)
        self.pub_debug = rospy.Publisher("/mppi/debug", Float32MultiArray, queue_size=1)
        
        # Subscribers
        self.sub_theta = rospy.Subscriber(
            self.theta_topic, JointState, self.cb_theta, queue_size=10
        )
        self.sub_target = rospy.Subscriber(
            self.target_topic, Float32, self.cb_target, queue_size=1
        )
        self.sub_trajectory = rospy.Subscriber(
            self.trajectory_topic, Float32MultiArray, self.cb_trajectory, queue_size=1
        )
        self.sub_pressure = rospy.Subscriber(
            self.pressure_topic, Vector3, self.cb_pressure, queue_size=50
        )
    
    # ========== Model Loading ==========
    
    def load_model(self):
        """モデルとメタデータをロード"""
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
            rospy.logwarn("[MPPI] Running on CPU - performance will be limited!")

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
    
    # ========== ROS Callbacks ==========
    
    def cb_theta(self, msg: JointState):
        """関節角度のコールバック"""
        if self.theta_index < len(msg.position):
            theta_raw = float(msg.position[self.theta_index])
            with self.lock:
                self.theta_rad = self.theta_filter.update(theta_raw)
                self.hist_theta.appendleft(self.theta_rad)
    
    def cb_target(self, msg: Float32):
        """目標角度のコールバック"""
        target_rad = math.radians(float(msg.data))
        self.theta_ref_rad = target_rad
        self.target_history.append((rospy.get_time(), target_rad))
    
    def cb_trajectory(self, msg: Float32MultiArray):
        """予測軌道のコールバック（外部から与える場合）"""
        if self.use_trajectory_prediction:
            self.theta_trajectory = np.array(msg.data)
    
    def cb_pressure(self, msg: Vector3):
        """圧力測定値のコールバック"""
        with self.lock:
            self.p1_meas = float(msg.x)
            self.p2_meas = float(msg.y)
    
    # ========== B-spline Functions ==========
    
    def generate_bspline_trajectory(self, control_points):
        """B-spline制御点から滑らかな軌道を生成"""
        n_pts = self.bspline_control_points
        degree = min(self.bspline_degree, n_pts - 1)
        
        # 時間パラメータ
        t_ctrl = np.linspace(0, 1, n_pts)
        t_eval = np.linspace(0, 1, self.H)
        
        # 各次元でB-spline補間
        U_smooth = np.zeros((self.H, 2))
        for dim in range(2):
            # scipy.interpolateを使用
            spl = make_interp_spline(t_ctrl, control_points[:, dim], k=degree)
            U_smooth[:, dim] = spl(t_eval)
        
        return U_smooth
    
    def sample_bspline_controls(self, K):
        """B-spline制御点をサンプリング"""
        # 制御点に対するノイズ
        noise = np.random.normal(
            0, self.sigma_u, 
            (K, self.bspline_control_points, 2)
        ).astype(np.float32)
        
        # 各サンプルについてB-spline軌道を生成
        U_batch = np.zeros((K, self.H, 2), dtype=np.float32)
        for k in range(K):
            ctrl_pts = self.control_points + noise[k]
            U_smooth = self.generate_bspline_trajectory(ctrl_pts)
            # 差分制御に変換
            U_batch[k, :, :] = np.diff(
                np.vstack([np.zeros((1, 2)), U_smooth]), axis=0
            )
        
        return U_batch
    
    # ========== Adaptive Sigma ==========
    
    def update_adaptive_sigma(self):
        """エラーに基づいてsigmaを適応的に調整"""
        if not self.use_adaptive_sigma:
            return
        
        error = abs(self.theta_ref_rad - self.theta_rad)
        
        # エラーが大きいときは探索範囲を拡大
        if error > np.radians(10):
            self.sigma_u = min(
                self.sigma_u * (1 + self.sigma_adaptation_rate),
                self.sigma_u_max
            )
        # エラーが小さいときは探索範囲を縮小
        elif error < np.radians(2):
            self.sigma_u = max(
                self.sigma_u * (1 - self.sigma_adaptation_rate * 0.5),
                self.sigma_u_min
            )
    
    # ========== Trajectory Prediction ==========
    
    def predict_reference_trajectory(self):
        """将来の参照軌道を予測"""
        if not self.use_trajectory_prediction:
            # 予測なし：現在の目標値を維持
            return np.full(self.H, self.theta_ref_rad)
        
        if self.theta_trajectory is not None and len(self.theta_trajectory) >= self.H:
            # 外部から与えられた軌道を使用
            return self.theta_trajectory[:self.H]
        
        # 内部での予測
        if self.prediction_method == "linear":
            # 線形予測
            if len(self.target_history) >= 2:
                times = np.array([t for t, _ in list(self.target_history)[-10:]])
                values = np.array([v for _, v in list(self.target_history)[-10:]])
                
                if len(times) > 1:
                    # 線形フィッティング
                    coeffs = np.polyfit(times - times[0], values, 1)
                    future_times = np.arange(self.H) * self.dt
                    predicted = np.polyval(coeffs, future_times)
                    return predicted
            
        elif self.prediction_method == "spline":
            # スプライン補間による予測
            if len(self.target_history) >= 3:
                times = np.array([t for t, _ in list(self.target_history)[-5:]])
                values = np.array([v for _, v in list(self.target_history)[-5:]])
                
                try:
                    spl = make_interp_spline(times, values, k=min(3, len(times)-1))
                    future_times = times[-1] + np.arange(self.H) * self.dt
                    predicted = spl(future_times)
                    return predicted
                except:
                    pass
        
        # デフォルト：現在の目標値を維持
        return np.full(self.H, self.theta_ref_rad)
    
    # ========== Core MPPI Functions ==========
    
    def enforce_constraints(self, p1, p2, p1_prev, p2_prev, dt):
        """物理制約を適用"""
        dp_max_step = self.dp_max * dt
        p1 = np.clip(p1, p1_prev - dp_max_step, p1_prev + dp_max_step)
        p2 = np.clip(p2, p2_prev - dp_max_step, p2_prev + dp_max_step)
        p1 = np.clip(p1, 0.0, self.p_max)
        p2 = np.clip(p2, 0.0, self.p_max)
        return p1, p2
    
    def cost_function(self, theta, theta_ref, p1, p2, p1_prev, p2_prev,
                      dp1, dp2, k, H):
        """コスト関数"""
        err = theta_ref - theta
        cost = self.w_tracking * (err ** 2)
        
        if k == H - 1:
            cost += self.w_terminal * (err ** 2)
        
        cost += self.w_smooth * (dp1 ** 2 + dp2 ** 2)
        cost += self.w_effort * (p1 ** 2 + p2 ** 2)
        
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
    
    def rollout_batch(self, theta0, p1_0, p2_0, U, ref_trajectory):
        """バッチ推論によるroll-out"""
        K, H = U.shape[0], U.shape[1]
        dt = self.dt

        theta_seq = np.zeros((K, H), dtype=np.float32)
        p1_seq = np.zeros((K, H), dtype=np.float32)
        p2_seq = np.zeros((K, H), dtype=np.float32)

        theta_k = np.full(K, theta0, dtype=np.float32)
        p1_k = np.full(K, p1_0, dtype=np.float32)
        p2_k = np.full(K, p2_0, dtype=np.float32)

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

        theta_hist = np.tile(np.array(theta_hist0, dtype=np.float32), (K, 1))
        p1_hist = np.tile(np.array(p1_hist0, dtype=np.float32), (K, 1))
        p2_hist = np.tile(np.array(p2_hist0, dtype=np.float32), (K, 1))
        dp1_hist = np.tile(np.array(dp1_hist0, dtype=np.float32), (K, 1))
        dp2_hist = np.tile(np.array(dp2_hist0, dtype=np.float32), (K, 1))

        for h in range(H):
            dp1 = U[:, h, 0]
            dp2 = U[:, h, 1]

            p1_prev = p1_k.copy()
            p2_prev = p2_k.copy()

            p1_k = p1_k + dp1
            p2_k = p2_k + dp2

            for i in range(K):
                p1_k[i], p2_k[i] = self.enforce_constraints(
                    p1_k[i], p2_k[i], p1_prev[i], p2_prev[i], dt
                )

            dp1_dt = (p1_k - p1_prev) / dt
            dp2_dt = (p2_k - p2_prev) / dt

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

            X_chunks = []
            for k in range(self.lags):
                X_chunks.append(theta_hist[:, k][:, None])
                X_chunks.append(p1_hist[:, k][:, None])
                X_chunks.append(p2_hist[:, k][:, None])
                X_chunks.append(dp1_hist[:, k][:, None])
                X_chunks.append(dp2_hist[:, k][:, None])
            
            X_batch = np.concatenate(X_chunks, axis=1).astype(np.float32)
            X_norm = (X_batch - self.mu) / (self.std + 1e-8)

            with torch.no_grad():
                Y_batch = self.model(torch.from_numpy(X_norm).to(self.device))
            theta_k = Y_batch.cpu().numpy().flatten()

            theta_seq[:, h] = theta_k
            p1_seq[:, h] = p1_k
            p2_seq[:, h] = p2_k

        return theta_seq, p1_seq, p2_seq
    
    def mppi_step(self):
        """拡張版MPPI制御ステップ"""
        t_start = time.time()
        
        with self.lock:
            theta = self.theta_rad
            p1_prev = self.p1_cmd
            p2_prev = self.p2_cmd
        
        # Safety check
        is_safe, msg = self.safety.check(theta)
        if not is_safe:
            rospy.logerr(f"[MPPI] Safety violation: {msg}")
            self.emergency_stop = True
            self.publish_cmd(0.0, 0.0)
            return
        
        # Adaptive sigma update
        self.update_adaptive_sigma()
        
        # Reference trajectory prediction
        ref_trajectory = self.predict_reference_trajectory()
        
        # Control sampling
        if self.use_bspline:
            U = self.sample_bspline_controls(self.K)
        else:
            U = np.random.normal(
                loc=0.0,
                scale=self.sigma_u,
                size=(self.K, self.H, 2)
            ).astype(np.float32)
        
        # Rollout
        theta_seq, p1_seq, p2_seq = self.rollout_batch(
            theta, p1_prev, p2_prev, U, ref_trajectory
        )
        
        # Cost computation
        dt = self.dt
        J = np.zeros(self.K, dtype=np.float32)
        
        for i in range(self.K):
            cost = 0.0
            p1_h, p2_h = p1_prev, p2_prev
            for h in range(self.H):
                dp1 = U[i, h, 0]
                dp2 = U[i, h, 1]
                cost += self.cost_function(
                    theta_seq[i, h], ref_trajectory[h],
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
        
        dU = np.sum(w[:, None, None] * U, axis=0) / w_sum
        
        # Update B-spline control points if used
        if self.use_bspline:
            weighted_ctrl = np.sum(
                w[:, None, None] * (self.control_points + 
                np.random.normal(0, self.sigma_u, 
                                (self.K, self.bspline_control_points, 2))),
                axis=0
            ) / w_sum
            self.control_points = 0.9 * self.control_points + 0.1 * weighted_ctrl
        
        # Apply first control
        dp1_cmd, dp2_cmd = dU[0, 0], dU[0, 1]
        p1_cmd = p1_prev + dp1_cmd
        p2_cmd = p2_prev + dp2_cmd
        
        p1_cmd, p2_cmd = self.enforce_constraints(p1_cmd, p2_cmd, p1_prev, p2_prev, dt)
        
        self.publish_cmd(p1_cmd, p2_cmd)
        
        # Update history
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
        tracking_error = abs(ref_trajectory[0] - theta)
        self.tracking_error_buf.append(tracking_error)
        
        # Debug output
        if rospy.get_param("~debug", False):
            debug_msg = Float32MultiArray()
            debug_msg.data = [
                float(np.min(J)), float(np.mean(J)), 
                float(self.sigma_u), float(tracking_error),
                comp_time * 1000
            ]
            self.pub_debug.publish(debug_msg)
        
        # Logging
        if self.log_path:
            self.log_buffer.append({
                't': rospy.get_time(),
                'theta': theta,
                'theta_ref': ref_trajectory[0],
                'error': tracking_error,
                'p1_cmd': p1_cmd,
                'p2_cmd': p2_cmd,
                'p1_meas': self.p1_meas,
                'p2_meas': self.p2_meas,
                'J_min': float(np.min(J)),
                'J_mean': float(np.mean(J)),
                'sigma_u': self.sigma_u,
                'comp_time_ms': comp_time * 1000.0
            })
    
    def publish_cmd(self, p1, p2):
        """圧力指令を出力"""
        msg = Vector3()
        msg.x = float(p1) * 4096.0 / 0.9
        msg.y = float(p2) * 4096.0 / 0.9
        msg.z = 0.0
        self.pub_cmd.publish(msg)
    
    def setup_logging(self):
        """ロギングのセットアップ"""
        import csv
        
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        
        self.log_file = open(self.log_path, 'w', newline='')
        self.log_writer = csv.DictWriter(self.log_file, fieldnames=[
            't', 'theta', 'theta_ref', 'error',
            'p1_cmd', 'p2_cmd', 'p1_meas', 'p2_meas',
            'J_min', 'J_mean', 'sigma_u', 'comp_time_ms'
        ])
        self.log_writer.writeheader()
        
        self.log_thread = threading.Thread(target=self.logging_worker, daemon=True)
        self.log_thread.start()
    
    def logging_worker(self):
        """バックグラウンドログ書き込み"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if len(self.log_buffer) > 0:
                batch = []
                while len(self.log_buffer) > 0 and len(batch) < 100:
                    batch.append(self.log_buffer.popleft())
                try:
                    self.log_writer.writerows(batch)
                    self.log_file.flush()
                except Exception as e:
                    rospy.logerr(f"[MPPI] Logging error: {e}")
            rate.sleep()
    
    def spin(self):
        """メインループ"""
        rate = rospy.Rate(self.rate_hz)
        frame_count = 0
        
        rospy.loginfo("[Enhanced MPPI] Starting control loop...")
        self.pub_status.publish(String("running"))
        
        # Warmup
        warmup_duration = 2.0
        warmup_start = rospy.get_time()
        
        rospy.loginfo(f"[MPPI] Warmup for {warmup_duration}s...")
        while not rospy.is_shutdown():
            if rospy.get_time() - warmup_start > warmup_duration:
                break
            self.publish_cmd(0.0, 0.0)
            rate.sleep()
        
        rospy.loginfo("[MPPI] Control active!")
        
        try:
            while not rospy.is_shutdown():
                if self.emergency_stop:
                    self.publish_cmd(0.0, 0.0)
                    rospy.logerr("[MPPI] Emergency stop active")
                    rate.sleep()
                    continue
                
                if frame_count % self.frame_skip == 0:
                    self.mppi_step()
                
                frame_count += 1
                
                # Performance report
                if frame_count % int(self.rate_hz * 10) == 0:
                    if len(self.comp_time_buf) > 0:
                        avg_time = np.mean(self.comp_time_buf)
                        max_time = np.max(self.comp_time_buf)
                        rospy.loginfo(
                            f"[MPPI] Comp: avg={avg_time*1000:.1f}ms, "
                            f"max={max_time*1000:.1f}ms | "
                            f"Sigma: {self.sigma_u:.3f} | "
                            f"Error: {np.mean(self.tracking_error_buf)*180/np.pi:.2f}°"
                        )
                
                rate.sleep()
        
        except rospy.ROSInterruptException:
            pass
        
        finally:
            rospy.loginfo("[MPPI] Shutting down...")
            self.publish_cmd(0.0, 0.0)
            self.pub_status.publish(String("stopped"))
            if self.log_file:
                self.log_file.close()


def main():
    controller = EnhancedMPPIController()
    controller.spin()

if __name__ == '__main__':
    main()