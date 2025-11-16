#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tune_mppi_gains.py
MPPIゲイン自動チューニング

Features:
- グリッドサーチ
- ベイズ最適化（optuna）
- インタラクティブ調整

Usage:
  python tune_mppi_gains.py --mode interactive
  python tune_mppi_gains.py --mode grid
  python tune_mppi_gains.py --mode bayesian
"""
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from typing import Dict, Tuple
import subprocess
import time
import os

class MPPITuner:
    def __init__(self):
        self.default_gains = {
            'w_tracking': 30.0,
            'w_smooth': 0.05,
            'w_effort': 0.01,
            'w_constraint': 500.0,
            'K': 32,
            'horizon': 15,
            'lambda': 2.0,
            'sigma_u': 0.10
        }
        
        # 探索範囲
        self.search_space = {
            'w_tracking': (10.0, 100.0),
            'w_smooth': (0.01, 0.20),
            'w_effort': (0.001, 0.05),
            'w_constraint': (100.0, 1000.0),
            'lambda': (0.5, 5.0),
            'sigma_u': (0.05, 0.20)
        }
        
        self.results = []
    
    def evaluate_gains(self, gains: Dict) -> Tuple[float, Dict]:
        """
        ゲインセットを評価
        
        Returns:
            score: 総合スコア（小さいほど良い）
            metrics: 詳細メトリクス
        """
        print(f"\n[Evaluating Gains]")
        for k, v in gains.items():
            print(f"  {k:15s}: {v}")
        
        # ROSlaunchを起動（バックグラウンド）
        log_path = f"logs/tune_{int(time.time())}.csv"
        
        launch_args = [
            'roslaunch', 'kinikun', 'narx_mppi2.launch',
            f'w_tracking:={gains["w_tracking"]}',
            f'w_smooth:={gains["w_smooth"]}',
            f'w_effort:={gains["w_effort"]}',
            f'w_constraint:={gains["w_constraint"]}',
            f'lambda:={gains["lambda"]}',
            f'sigma_u:={gains["sigma_u"]}',
            f'K:={gains.get("K", 32)}',
            f'horizon:={gains.get("horizon", 15)}',
            f'log_csv:={log_path}'
        ]
        
        proc = subprocess.Popen(launch_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        # テスト実行（30秒）
        print("  Running test (30s)...")
        time.sleep(5)  # warmup
        
        # 目標値送信（ステップ応答テスト）
        subprocess.run(['rostopic', 'pub', '-1', '/theta_target_deg',
                       'std_msgs/Float32', 'data: 20.0'])
        time.sleep(10)
        
        subprocess.run(['rostopic', 'pub', '-1', '/theta_target_deg',
                       'std_msgs/Float32', 'data: -20.0'])
        time.sleep(10)
        
        subprocess.run(['rostopic', 'pub', '-1', '/theta_target_deg',
                       'std_msgs/Float32', 'data: 0.0'])
        time.sleep(5)
        
        # 停止
        proc.terminate()
        proc.wait()
        
        # ログ解析
        if not os.path.exists(log_path):
            print("  ERROR: Log file not found")
            return float('inf'), {}
        
        df = pd.read_csv(log_path)
        
        # メトリクス計算
        rmse = np.sqrt(np.mean(df['error']**2))
        mae = np.mean(np.abs(df['error']))
        max_error = np.max(np.abs(df['error']))
        
        # 圧力変化率
        dt = np.median(np.diff(df['t'].values))
        dp1_dt = np.abs(np.diff(df['p1_cmd'].values) / dt)
        dp2_dt = np.abs(np.diff(df['p2_cmd'].values) / dt)
        mean_dp_dt = np.mean(np.concatenate([dp1_dt, dp2_dt]))
        
        # 計算時間
        mean_comp_time = df['comp_time_ms'].mean()
        
        metrics = {
            'rmse': float(rmse),
            'mae': float(mae),
            'max_error': float(max_error),
            'mean_dp_dt': float(mean_dp_dt),
            'mean_comp_time': float(mean_comp_time)
        }
        
        # スコア計算（重み付き）
        score = (
            10.0 * rmse +           # 追従性重視
            5.0 * mae +
            2.0 * max_error +
            0.5 * mean_dp_dt +      # 滑らかさ
            0.1 * mean_comp_time    # 計算時間
        )
        
        print(f"  RMSE: {rmse:.4f}, MAE: {mae:.4f}, Score: {score:.2f}")
        
        return score, metrics
    
    def interactive_tuning(self):
        """インタラクティブチューニング"""
        print("\n" + "="*70)
        print(" Interactive MPPI Gain Tuning")
        print("="*70)
        
        gains = self.default_gains.copy()
        
        while True:
            print("\n[Current Gains]")
            for i, (k, v) in enumerate(gains.items(), 1):
                print(f"  {i}. {k:15s}: {v}")
            
            print("\nOptions:")
            print("  1-8: Modify gain")
            print("  t: Test current gains")
            print("  r: Reset to default")
            print("  q: Quit")
            
            choice = input("\nChoice: ").strip()
            
            if choice == 'q':
                break
            elif choice == 'r':
                gains = self.default_gains.copy()
                print("Reset to default.")
            elif choice == 't':
                score, metrics = self.evaluate_gains(gains)
                self.results.append({**gains, 'score': score, **metrics})
            elif choice.isdigit() and 1 <= int(choice) <= 8:
                idx = int(choice) - 1
                key = list(gains.keys())[idx]
                new_val = input(f"New value for {key} [{gains[key]}]: ").strip()
                try:
                    gains[key] = type(gains[key])(new_val)
                except:
                    print("Invalid value")
        
        # 結果保存
        if self.results:
            df_results = pd.DataFrame(self.results)
            df_results.to_csv('tuning_results.csv', index=False)
            print(f"\nResults saved to tuning_results.csv")
            print(f"Best score: {df_results['score'].min():.2f}")
    
    def grid_search(self):
        """グリッドサーチ"""
        print("\n" + "="*70)
        print(" Grid Search MPPI Gain Tuning")
        print("="*70)
        
        # 簡易版: w_tracking と w_smooth のみ
        tracking_range = np.linspace(10, 80, 5)
        smooth_range = np.linspace(0.01, 0.15, 4)
        
        print(f"\nSearching:")
        print(f"  w_tracking: {tracking_range}")
        print(f"  w_smooth: {smooth_range}")
        print(f"  Total: {len(tracking_range) * len(smooth_range)} evaluations")
        
        for w_track in tracking_range:
            for w_sm in smooth_range:
                gains = self.default_gains.copy()
                gains['w_tracking'] = w_track
                gains['w_smooth'] = w_sm
                
                score, metrics = self.evaluate_gains(gains)
                self.results.append({**gains, 'score': score, **metrics})
        
        # ベスト表示
        df_results = pd.DataFrame(self.results)
        df_results.to_csv('grid_search_results.csv', index=False)
        
        best_idx = df_results['score'].idxmin()
        best = df_results.loc[best_idx]
        
        print("\n[Best Configuration]")
        print(f"  w_tracking: {best['w_tracking']:.2f}")
        print(f"  w_smooth: {best['w_smooth']:.4f}")
        print(f"  Score: {best['score']:.2f}")
        print(f"  RMSE: {best['rmse']:.4f}")
    
    def bayesian_optimization(self, n_trials=20):
        """ベイズ最適化（optuna）"""
        try:
            import optuna
        except ImportError:
            print("ERROR: optuna not installed. Run: pip install optuna")
            return
        
        print("\n" + "="*70)
        print(" Bayesian Optimization (Optuna)")
        print("="*70)
        
        def objective(trial):
            gains = {
                'w_tracking': trial.suggest_float('w_tracking', 10.0, 100.0),
                'w_smooth': trial.suggest_float('w_smooth', 0.01, 0.20),
                'w_effort': trial.suggest_float('w_effort', 0.001, 0.05),
                'w_constraint': trial.suggest_float('w_constraint', 100.0, 1000.0),
                'lambda': trial.suggest_float('lambda', 0.5, 5.0),
                'sigma_u': trial.suggest_float('sigma_u', 0.05, 0.20),
                'K': self.default_gains['K'],
                'horizon': self.default_gains['horizon']
            }
            
            score, metrics = self.evaluate_gains(gains)
            return score
        
        study = optuna.create_study(direction='minimize')
        study.optimize(objective, n_trials=n_trials)
        
        print("\n[Best Trial]")
        print(f"  Score: {study.best_value:.2f}")
        print("  Parameters:")
        for k, v in study.best_params.items():
            print(f"    {k:15s}: {v:.4f}")
        
        # 結果保存
        df_trials = study.trials_dataframe()
        df_trials.to_csv('bayesian_optimization_results.csv', index=False)
        print("\nResults saved to bayesian_optimization_results.csv")
        
        # 可視化
        try:
            from optuna.visualization import plot_optimization_history, plot_param_importances
            
            fig1 = plot_optimization_history(study)
            fig1.write_html('optuna_history.html')
            
            fig2 = plot_param_importances(study)
            fig2.write_html('optuna_importance.html')
            
            print("\nVisualization saved:")
            print("  optuna_history.html")
            print("  optuna_importance.html")
        except Exception as e:
            print(f"Visualization failed: {e}")

def main():
    parser = argparse.ArgumentParser(description='MPPI Gain Tuning')
    parser.add_argument('--mode', type=str, default='interactive',
                        choices=['interactive', 'grid', 'bayesian'],
                        help='Tuning mode')
    parser.add_argument('--n_trials', type=int, default=20,
                        help='Number of trials for bayesian optimization')
    args = parser.parse_args()
    
    tuner = MPPITuner()
    
    if args.mode == 'interactive':
        tuner.interactive_tuning()
    elif args.mode == 'grid':
        tuner.grid_search()
    elif args.mode == 'bayesian':
        tuner.bayesian_optimization(args.n_trials)

if __name__ == '__main__':
    main()