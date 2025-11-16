#!/usr/bin/env python3
"""
monitor_mppi.py
MPPI制御のリアルタイムモニタリング
"""
import rospy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np

class MPPIMonitor:
    def __init__(self):
        rospy.init_node('mppi_monitor', anonymous=True)
        
        self.max_points = 500
        self.t_buf = deque(maxlen=self.max_points)
        self.theta_buf = deque(maxlen=self.max_points)
        self.theta_ref_buf = deque(maxlen=self.max_points)
        self.p1_buf = deque(maxlen=self.max_points)
        self.p2_buf = deque(maxlen=self.max_points)
        self.error_buf = deque(maxlen=self.max_points)
        
        self.theta_ref = 0.0
        self.t_start = rospy.get_time()
        
        # Subscribers
        rospy.Subscriber("/kinikun1/joint_states", JointState, self.cb_theta)
        rospy.Subscriber("/theta_target_deg", Float32, self.cb_target)
        rospy.Subscriber("/mpa_cmd", Vector3, self.cb_cmd)
        
        # Setup plot
        self.fig, self.axes = plt.subplots(3, 1, figsize=(12, 10))
        self.fig.suptitle('NARX-MPPI Real-time Monitor', fontsize=14, fontweight='bold')
        
        self.setup_plot()
    
    def cb_theta(self, msg):
        if len(msg.position) > 2:
            t = rospy.get_time() - self.t_start
            theta = msg.position[2]
            
            self.t_buf.append(t)
            self.theta_buf.append(theta)
            self.theta_ref_buf.append(self.theta_ref)
            self.error_buf.append(self.theta_ref - theta)
    
    def cb_target(self, msg):
        self.theta_ref = np.radians(msg.data)
    
    def cb_cmd(self, msg):
        self.p1_buf.append(msg.x)
        self.p2_buf.append(msg.y)
    
    def setup_plot(self):
        for ax in self.axes:
            ax.grid(True, alpha=0.3)
        
        self.axes[0].set_ylabel('Angle [rad]')
        self.axes[0].set_title('Tracking Performance')
        
        self.axes[1].set_ylabel('Error [rad]')
        self.axes[1].set_title('Tracking Error')
        self.axes[1].axhline(0, color='k', linestyle='--', alpha=0.3)
        
        self.axes[2].set_ylabel('Pressure [MPa]')
        self.axes[2].set_xlabel('Time [s]')
        self.axes[2].set_title('Control Inputs')
        self.axes[2].set_ylim([0, 0.75])
    
    def update_plot(self, frame):
        if len(self.t_buf) < 2:
            return
        
        t = np.array(self.t_buf)
        theta = np.array(self.theta_buf)
        theta_ref = np.array(self.theta_ref_buf)
        error = np.array(self.error_buf)
        p1 = np.array(self.p1_buf)
        p2 = np.array(self.p2_buf)
        
        # Clear
        for ax in self.axes:
            ax.clear()
            ax.grid(True, alpha=0.3)
        
        # Plot 1: Tracking
        self.axes[0].plot(t, theta_ref, 'b--', label='Reference', linewidth=2, alpha=0.7)
        self.axes[0].plot(t, theta, 'r-', label='Actual', linewidth=1.5)
        self.axes[0].set_ylabel('Angle [rad]')
        self.axes[0].legend(loc='upper right')
        self.axes[0].set_title('Tracking Performance')
        
        # Plot 2: Error
        self.axes[1].plot(t, error, 'r-', linewidth=1.5)
        self.axes[1].axhline(0, color='k', linestyle='--', alpha=0.3)
        self.axes[1].fill_between(t, -0.05, 0.05, color='g', alpha=0.2)
        self.axes[1].set_ylabel('Error [rad]')
        self.axes[1].set_title(f'Error (current: {error[-1]:.3f} rad = {np.degrees(error[-1]):.2f}°)')
        
        # Plot 3: Control
        if len(p1) >= len(t):
            self.axes[2].plot(t, p1[:len(t)], 'b-', label='p1', linewidth=1.5)
            self.axes[2].plot(t, p2[:len(t)], 'r-', label='p2', linewidth=1.5)
        self.axes[2].set_ylabel('Pressure [MPa]')
        self.axes[2].set_xlabel('Time [s]')
        self.axes[2].legend(loc='upper right')
        self.axes[2].set_ylim([0, 0.75])
        self.axes[2].set_title('Control Inputs')
        
        plt.tight_layout()
    
    def run(self):
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100, cache_frame_data=False)
        plt.show()

if __name__ == '__main__':
    try:
        monitor = MPPIMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass