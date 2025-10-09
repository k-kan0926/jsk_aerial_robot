#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
exp_mpa_dataset_maker.py

ROS 実験ノード（学習用データ収集 完全版）
- p1/p2 の上限・可行領域を厳密に尊重（p1,p2 ∈ [0,p_max], |Δ| ≤ min(Σ, 2*p_max-Σ)）
- 離散グリッド + 連続励起（サイン/マルチサイン/チャープ）をスケジュール化
- ランプ（和圧/差圧の独立 slew 限界）→ ホールド → セトル（任意）
- PRBS をホールド中/連続中に微小注入（任意）
- /mpa_cmd(Vector3) に p1,p2 を publish（x=p1*4096/0.9, y=p2*4096/0.9, z=0）
- 和圧/差圧や派生量も ~ 名前空間で publish
- CSV ログ（任意）: 時刻, p1, p2, Σ, Δ, モード, 目標, etc.

想定: ROS Noetic など（Python3）。
"""

import os, csv, math, random, itertools
import numpy as np
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32, String

# ============ ユーティリティ ============

def psi_to_p12(ps, pd):
    """Σ,Δ -> (p1,p2)"""
    return 0.5*(ps + pd), 0.5*(ps - pd)

def feasible_clip(ps, pd, p_max):
    """可行領域へ射影: p1,p2∈[0,p_max] と等価に |Δ|≤min(Σ,2p_max-Σ) を守る"""
    ps = float(np.clip(ps, 0.0, 2.0*p_max))
    pd_max = max(0.0, min(ps, 2.0*p_max - ps))
    pd = float(np.clip(pd, -pd_max, +pd_max))
    # 念のため個別もクリップ
    p1, p2 = psi_to_p12(ps, pd)
    p1 = float(np.clip(p1, 0.0, p_max))
    p2 = float(np.clip(p2, 0.0, p_max))
    # 個別から戻してΣΔ整合
    ps = p1 + p2
    pd = p1 - p2
    return ps, pd

def ramp_step(ps_now, pd_now, ps_tgt, pd_tgt, dt, Ssum, Sdiff, p_max):
    dps = np.clip(ps_tgt - ps_now, -Ssum*dt, +Ssum*dt)
    dpd = np.clip(pd_tgt - pd_now, -Sdiff*dt, +Sdiff*dt)
    ps_new = ps_now + dps
    pd_new = pd_now + dpd
    return feasible_clip(ps_new, pd_new, p_max)

# ============ ノード本体 ============

class ExpMPADataMaker:
    def __init__(self):
        # --- 基本設定 ---
        self.p_max = rospy.get_param("~p_max_MPa", 0.70)
        self.p_sum_min = rospy.get_param("~p_sum_min_MPa", 0.0)
        self.p_sum_max = min(rospy.get_param("~p_sum_max_MPa", 1.00), 2.0*self.p_max)
        self.rate_hz = rospy.get_param("~rate_hz", 40.0)
        self.ramp_sec = rospy.get_param("~ramp_sec", 0.3)
        self.hold_sec = rospy.get_param("~hold_sec", 0.7)
        self.settle_sec = rospy.get_param("~settle_sec", 1.0)
        self.cycles = rospy.get_param("~cycles", 1)
        self.randomize_order = rospy.get_param("~randomize_order", True)
        self.seed = int(rospy.get_param("~seed", 123))
        random.seed(self.seed)

        # --- グリッド設定 ---
        self.enable_grid = rospy.get_param("~enable_grid", True)
        self.grid_sum_pts = rospy.get_param("~grid_sum_pts", 12)
        self.grid_diff_pts = rospy.get_param("~grid_diff_pts", 13)
        self.slew_sum_list = rospy.get_param("~slew_sum_list_MPa_s", [0.06, 0.12])
        self.slew_diff_list = rospy.get_param("~slew_diff_list_MPa_s", [0.12, 0.25])
        self.dir_modes = rospy.get_param("~dir_modes", ["up","down"])  # 進入方向
        self.grid_margin = rospy.get_param("~grid_margin_MPa", 0.02)

        # --- 連続励起設定（サイン/マルチサイン/チャープ） ---
        self.enable_sine = rospy.get_param("~enable_sine", True)
        self.enable_multisine = rospy.get_param("~enable_multisine", True)
        self.enable_chirp = rospy.get_param("~enable_chirp", True)
        # 中心ΣΔの候補（各セグメントで一つ選ぶ）
        self.center_sum_list = rospy.get_param("~center_sum_list_MPa", [0.30, 0.50, 0.70])
        self.center_diff_list = rospy.get_param("~center_diff_list_MPa", [0.00, 0.10, -0.10])
        # 振幅候補（可行領域内で後でさらにクリップ）
        self.amp_sum_list = rospy.get_param("~amp_sum_list_MPa", [0.05, 0.10])
        self.amp_diff_list = rospy.get_param("~amp_diff_list_MPa", [0.05, 0.10])
        # 周波数（Hz）リスト
        self.sine_freqs = rospy.get_param("~sine_freqs_Hz", [0.05, 0.1, 0.3, 0.5])
        self.multi_freqs = rospy.get_param("~multi_freqs_Hz", [0.07, 0.23, 0.41])
        # チャープ設定
        self.chirp_f0 = rospy.get_param("~chirp_f0_Hz", 0.05)
        self.chirp_f1 = rospy.get_param("~chirp_f1_Hz", 1.00)
        self.cont_seg_sec = rospy.get_param("~cont_seg_sec", 12.0)  # 1セグメント長

        # --- PRBS 注入 ---
        self.probe_enable = rospy.get_param("~probe_enable", True)
        self.probe_bits = int(rospy.get_param("~probe_bits", 20))
        self.probe_bit_sec = float(rospy.get_param("~probe_bit_sec", 0.1))
        self.probe_amp_sum = float(rospy.get_param("~probe_amp_sum_MPa", 0.01))
        self.probe_amp_diff = float(rospy.get_param("~probe_amp_diff_MPa", 0.01))

        # --- CSV ログ ---
        self.csv_path = rospy.get_param("~csv_path", "")  # 空なら書かない
        self.csv_fp = None
        self.csv_writer = None

        # --- Publisher ---
        self.pub_mpa = rospy.Publisher("/mpa_cmd", Vector3, queue_size=10)
        self.pub_sum = rospy.Publisher("~p_sum_MPa", Float32, queue_size=10)
        self.pub_diff = rospy.Publisher("~p_diff_MPa", Float32, queue_size=10)
        self.pub_mode = rospy.Publisher("~mode", String, queue_size=10)
        self.pub_dsum  = rospy.Publisher("~dp_sum_MPa_s", Float32, queue_size=10)
        self.pub_ddiff = rospy.Publisher("~dp_diff_MPa_s", Float32, queue_size=10)

        # 状態
        self.prev_ps = 0.0
        self.prev_pd = 0.0
        self.prev_p1 = 0.0
        self.prev_p2 = 0.0
        self.prev_t = rospy.Time.now().to_sec()

        # スケジュール
        self.schedule = self._make_schedule()
        rospy.loginfo("[exp_mpa_dataset_maker] %d segments generated.", len(self.schedule))

        # CSV 開く
        if self.csv_path:
            os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
            self.csv_fp = open(self.csv_path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_fp)
            self.csv_writer.writerow(["t","mode","ps","pd","p1","p2","ps_ref","pd_ref","note"])  # 最小

    # ---------- スケジュール生成 ----------
    def _make_schedule(self):
        seq = []
        if self.enable_grid:
            seq += self._make_grid_segments()
        if self.enable_sine:
            seq += self._make_sine_segments(kind="sine")
        if self.enable_multisine:
            seq += self._make_sine_segments(kind="multisine")
        if self.enable_chirp:
            seq += self._make_chirp_segments()
        if self.randomize_order:
            random.shuffle(seq)
        seq = seq * max(1, int(self.cycles))
        return seq

    def _make_grid_segments(self):
        sums = np.linspace(self.p_sum_min, self.p_sum_max, int(self.grid_sum_pts))
        segs = []
        for ps in sums:
            pd_max = min(ps, 2.0*self.p_max - ps)
            diffs = [0.0] if pd_max < 1e-6 else np.linspace(-pd_max, +pd_max, int(self.grid_diff_pts))
            for pd in diffs:
                for Ssum in self.slew_sum_list:
                    for Sdiff in self.slew_diff_list:
                        for dm in self.dir_modes:
                            # 進入方向のプリスタート
                            margin = float(self.grid_margin)
                            if dm == "up":
                                ps_start = max(self.p_sum_min, min(ps, self.p_sum_max) - margin)
                            else:
                                ps_start = min(self.p_sum_max, max(ps, self.p_sum_min) + margin)
                            ps_start, pd_start = feasible_clip(ps_start, pd, self.p_max)
                            segs.append({
                                "type":"grid", "ps":float(ps), "pd":float(pd),
                                "ps_start":float(ps_start), "pd_start":float(pd_start),
                                "Ssum":float(Ssum), "Sdiff":float(Sdiff)
                            })
        return segs

    def _make_sine_segments(self, kind="sine"):
        segs = []
        # 各セグメントごとに中心/振幅/周波数/位相をサンプル
        for cs in self.center_sum_list:
            for cd in self.center_diff_list:
                for As in self.amp_sum_list:
                    for Ad in self.amp_diff_list:
                        # 可行中心へクリップ
                        ps0, pd0 = feasible_clip(cs, cd, self.p_max)
                        if kind == "sine":
                            for f in self.sine_freqs:
                                phs = random.uniform(0, 2*math.pi)
                                phd = random.uniform(0, 2*math.pi)
                                segs.append({
                                    "type":"sine", "ps0":ps0, "pd0":pd0, "As":float(As), "Ad":float(Ad),
                                    "freqs":[float(f)], "phs":[phs, phd], "T":float(self.cont_seg_sec)
                                })
                        elif kind == "multisine":
                            phs = [random.uniform(0, 2*math.pi) for _ in self.multi_freqs]
                            phd = [random.uniform(0, 2*math.pi) for _ in self.multi_freqs]
                            segs.append({
                                "type":"multisine", "ps0":ps0, "pd0":pd0, "As":float(As), "Ad":float(Ad),
                                "freqs":[float(ff) for ff in self.multi_freqs], "phs":[phs, phd], "T":float(self.cont_seg_sec)
                            })
        return segs

    def _make_chirp_segments(self):
        segs = []
        for cs in self.center_sum_list:
            for cd in self.center_diff_list:
                for As in self.amp_sum_list:
                    for Ad in self.amp_diff_list:
                        ps0, pd0 = feasible_clip(cs, cd, self.p_max)
                        segs.append({
                            "type":"chirp", "ps0":ps0, "pd0":pd0, "As":float(As), "Ad":float(Ad),
                            "f0":float(self.chirp_f0), "f1":float(self.chirp_f1), "T":float(self.cont_seg_sec)
                        })
        return segs

    # ---------- 実行ループ ----------
    def run(self):
        rate = rospy.Rate(self.rate_hz)
        dt = 1.0/float(self.rate_hz)
        i = 0
        mode = "idle"
        t0 = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            if i >= len(self.schedule):
                self._publish(0.0, 0.0, mode="done")
                break

            seg = self.schedule[i]
            stype = seg["type"]

            if stype == "grid":
                # ランプ → ホールド → セトル
                mode = "grid_ramp"
                # 初期化
                ps, pd = float(seg["ps_start"]), float(seg["pd_start"])
                ps_tgt, pd_tgt = float(seg["ps"]), float(seg["pd"])
                Ssum, Sdiff = float(seg["Ssum"]), float(seg["Sdiff"])
                t_state = rospy.Time.now().to_sec()

                # ramp
                while not rospy.is_shutdown():
                    now = rospy.Time.now().to_sec()
                    ps, pd = ramp_step(ps, pd, ps_tgt, pd_tgt, dt, Ssum, Sdiff, self.p_max)
                    self._publish_from_pspd(ps, pd, mode)
                    if (now - t_state) >= self.ramp_sec:
                        break
                    rate.sleep()

                # hold (+PRBS)
                mode = "grid_hold"
                t_state = rospy.Time.now().to_sec()
                while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - t_state) < self.hold_sec:
                    ps_cmd, pd_cmd = ps_tgt, pd_tgt
                    if self.probe_enable:
                        k = int(((rospy.Time.now().to_sec() - t_state) // self.probe_bit_sec) % 2)*2 - 1
                        ps_cmd += k * self.probe_amp_sum
                        pd_cmd += k * self.probe_amp_diff
                        ps_cmd, pd_cmd = feasible_clip(ps_cmd, pd_cmd, self.p_max)
                    self._publish_from_pspd(ps_cmd, pd_cmd, mode)
                    rate.sleep()

                # settle
                mode = "grid_settle"
                t_state = rospy.Time.now().to_sec()
                while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - t_state) < self.settle_sec:
                    self._publish_from_pspd(ps_tgt, pd_tgt, mode)
                    rate.sleep()

                i += 1

            elif stype in ("sine","multisine","chirp"):
                # 継続時間 T の連続励起
                ps0, pd0 = float(seg["ps0"]), float(seg["pd0"])
                As, Ad = float(seg["As"]), float(seg["Ad"])
                T = float(seg["T"])
                t_state = rospy.Time.now().to_sec()
                mode = f"{stype}"

                while not rospy.is_shutdown():
                    t = rospy.Time.now().to_sec() - t_state
                    if t >= T:
                        break
                    if stype == "sine":
                        f = seg["freqs"][0]
                        phs, phd = seg["phs"][0], seg["phs"][1]
                        ps_cmd = ps0 + As*math.sin(2*math.pi*f*t + phs)
                        pd_cmd = pd0 + Ad*math.sin(2*math.pi*f*t + phd)
                    elif stype == "multisine":
                        freqs = seg["freqs"]
                        phs, phd = seg["phs"][0], seg["phs"][1]
                        ps_cmd = ps0 + sum((As/len(freqs))*math.sin(2*math.pi*ff*t + phs[j]) for j,ff in enumerate(freqs))
                        pd_cmd = pd0 + sum((Ad/len(freqs))*math.sin(2*math.pi*ff*t + phd[j]) for j,ff in enumerate(freqs))
                    else:  # chirp 線形掃引
                        f0, f1 = float(seg["f0"]), float(seg["f1"])
                        f = f0 + (f1 - f0)*(t/T)
                        ps_cmd = ps0 + As*math.sin(2*math.pi*f*t)
                        pd_cmd = pd0 + Ad*math.sin(2*math.pi*f*t + math.pi/3.0)

                    # 可行領域へ
                    ps_cmd, pd_cmd = feasible_clip(ps_cmd, pd_cmd, self.p_max)
                    # 小PRBS を重畳（任意）
                    if self.probe_enable:
                        k = int((t // self.probe_bit_sec) % 2)*2 - 1
                        ps_cmd += k * self.probe_amp_sum
                        pd_cmd += k * self.probe_amp_diff
                        ps_cmd, pd_cmd = feasible_clip(ps_cmd, pd_cmd, self.p_max)

                    self._publish_from_pspd(ps_cmd, pd_cmd, mode)
                    rate.sleep()

                # セトル
                mode = f"{stype}_settle"
                t_state = rospy.Time.now().to_sec()
                while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - t_state) < self.settle_sec:
                    self._publish_from_pspd(ps0, pd0, mode)
                    rate.sleep()

                i += 1

            else:
                rospy.logwarn("Unknown segment type: %s", stype)
                i += 1

        # 終了処理
        if self.csv_fp:
            self.csv_fp.flush(); self.csv_fp.close(); self.csv_fp = None
        rospy.loginfo("[exp_mpa_dataset_maker] Finished.")

    # ---------- Publish & Logging ----------
    def _publish_from_pspd(self, ps, pd, mode="run"):
        ps, pd = feasible_clip(ps, pd, self.p_max)
        p1, p2 = psi_to_p12(ps, pd)
        self._publish(p1, p2, mode=mode, ps_ref=ps, pd_ref=pd)

    def _publish(self, p1, p2, mode="run", ps_ref=None, pd_ref=None):
        # 個別クリップ
        p1 = float(np.clip(p1, 0.0, self.p_max))
        p2 = float(np.clip(p2, 0.0, self.p_max))
        # ΣΔ
        ps = p1 + p2
        pd = p1 - p2
        # 速度（近似）
        now = rospy.Time.now().to_sec()
        dt = max(1e-9, now - self.prev_t)
        dsum = (ps - self.prev_ps)/dt
        ddiff = (pd - self.prev_pd)/dt
        self.prev_ps, self.prev_pd, self.prev_t = ps, pd, now

        # publish
        msg = Vector3()
        msg.x = p1 * 4096.0 / 0.9
        msg.y = p2 * 4096.0 / 0.9
        msg.z = 0.0
        self.pub_mpa.publish(msg)
        self.pub_sum.publish(Float32(ps))
        self.pub_diff.publish(Float32(pd))
        self.pub_mode.publish(String(mode))
        self.pub_dsum.publish(Float32(dsum))
        self.pub_ddiff.publish(Float32(ddiff))

        # log
        if self.csv_writer is not None:
            self.csv_writer.writerow([
                now, mode, ps, pd, p1, p2,
                (ps_ref if ps_ref is not None else ps),
                (pd_ref if pd_ref is not None else pd),
                ""
            ])

# ============ main ============
if __name__ == "__main__":
    rospy.init_node("exp_mpa_dataset_maker")
    node = ExpMPADataMaker()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
