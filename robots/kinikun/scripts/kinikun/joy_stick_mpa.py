#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion

class JoyMPAController:
    def __init__(self):
        rospy.init_node('joy_mpa_controller')
        
        # --- MPA values (MPa) ---
        self.v1_mpa = 0.2
        self.v2_mpa = 0.2
        self.v3_mpa = 0.2
        self.v4_mpa = 0.2
        self.mpa_step = 0.02
        self.v_min, self.v_max = 0.0, 0.7
        
        # --- Target angles (deg) ---
        self.target1_deg = 0.0
        self.target2_deg = 0.0
        self.angle_step = 0.5
        self.angle_limit = 35.0
        self.active_pair = 1
        self.prev_axes = []


        
        # --- Button indices (PS4 controller) ---
        # 標準的なPS4マッピング
        self.BTN_SQUARE = 0   # □ (v4)
        self.BTN_CROSS = 1    # × (v1)
        self.BTN_CIRCLE = 2   # ○ (v2)
        self.BTN_TRIANGLE = 3 # △ (v3)
        # D-Padが入っているaxesのインデックス（例）
        self.DPAD_V_AXIS = 10  # 上/下（これでペア切り替え）
        self.BTN_L1 = 4
        self.BTN_R1 = 5
        self.BTN_L2 = 6
        self.BTN_R2 = 7
        
        # --- Publishers ---
        self.mpa_pub = rospy.Publisher('/mpa_cmd', Quaternion, queue_size=1)
        self.target1_pub = rospy.Publisher('/mppi/theta_target_deg', Float32, queue_size=1)
        self.target2_pub = rospy.Publisher('/mppi/theta_target_deg_2', Float32, queue_size=1)
        
        # --- Subscriber ---
        self.joy_sub = rospy.Subscriber('/kinikun1/joy', Joy, self.joy_callback)
        
        # Previous button states for edge detection
        self.prev_buttons = []
        
        rospy.loginfo("Joy MPA Controller initialized")
        rospy.loginfo("Button mapping:")
        rospy.loginfo("  MPA increase: L2 + (×:v1, ○:v2, △:v3, □:v4)")
        rospy.loginfo("  MPA decrease: R2 + (×:v1, ○:v2, △:v3, □:v4)")
        rospy.loginfo("  Target1 increase: L1 + □")
        rospy.loginfo("  Target1 decrease: R1 + □")
        rospy.loginfo("  Target2 increase: L1 + ○")
        rospy.loginfo("  Target2 decrease: R1 + ○")
        
        # Publish initial MPA values
        rospy.sleep(0.1)
        self.publish_mpa_cmd()
    
    def mpa_to_dac(self, p_mpa):
        """Convert MPa to DAC count (0-4095)"""
        p_mpa = max(0.0, min(0.9, p_mpa))
        return int(p_mpa * 4096.0 / 0.9)
    
    def publish_mpa_cmd(self):
        """Publish current MPA values"""
        q = Quaternion()
        q.x = self.mpa_to_dac(self.v1_mpa)
        q.y = self.mpa_to_dac(self.v2_mpa)
        q.z = self.mpa_to_dac(self.v3_mpa)
        q.w = self.mpa_to_dac(self.v4_mpa)
        self.mpa_pub.publish(q)
    
    def clamp_mpa(self, value):
        """Clamp MPA value to valid range"""
        return max(self.v_min, min(self.v_max, value))
    
    def clamp_angle(self, value):
        """Clamp angle to valid range"""
        return max(-self.angle_limit, min(self.angle_limit, value))
    
    def button_pressed(self, btn_idx, buttons):
        """Check if button was just pressed (rising edge)"""
        if btn_idx >= len(buttons):
            return False
        if not self.prev_buttons or btn_idx >= len(self.prev_buttons):
            return False
        return buttons[btn_idx] == 1 and self.prev_buttons[btn_idx] == 0
    
    def button_is_held(self, btn_idx, buttons):
        """Check if button is currently held down"""
        if btn_idx >= len(buttons):
            return False
        return buttons[btn_idx] == 1
    def joy_callback(self, msg):
        buttons = msg.buttons
        axes = msg.axes

        # 初回は前回状態を保存して終了
        if not self.prev_buttons:
            self.prev_buttons = list(buttons)
            self.prev_axes = list(axes)
            return

        # ========= 十字キー（D-Pad）処理 =========
        # 上下左右は axes[self.DPAD_V_AXIS], axes[self.DPAD_H_AXIS] の ±1.0 前後で出る想定
        dpad_v = axes[self.DPAD_V_AXIS] if self.DPAD_V_AXIS < len(axes) else 0.0
        prev_dpad_v = (
            self.prev_axes[self.DPAD_V_AXIS]
            if self.prev_axes and self.DPAD_V_AXIS < len(self.prev_axes)
            else 0.0
        )

        # 「ニュートラル(≈0) → 上方向(>0.5)」に遷移した瞬間を「十字上が押された」とみなす
        # ※もし「上」が -1.0 の場合は、条件を「prev_dpad_v >= -0.5 and dpad_v < -0.5」に変えてください
        dpad_up_pressed = (prev_dpad_v <= 0.5 and dpad_v > 0.5)

        if dpad_up_pressed:
            # MPAペア切り替え: 1:(v1,v2) <-> 2:(v3,v4)
            self.active_pair = 2 if self.active_pair == 1 else 1
            rospy.loginfo(f"[JoyMPA] Active MPA pair switched to {self.active_pair}")

        # ========= MPA圧力制御 =========
        # L2 + □/○ でアクティブペアの各MPAを増加
        if self.button_is_held(self.BTN_L2, buttons):
            # 1本目 (v1 or v3) を増加: L2 + □
            if self.button_pressed(self.BTN_SQUARE, buttons):
                if self.active_pair == 1:
                    self.v1_mpa = self.clamp_mpa(self.v1_mpa + self.mpa_step)
                    dac = self.mpa_to_dac(self.v1_mpa)
                    rospy.loginfo(f"[JoyMPA] v1 ++ -> {self.v1_mpa:.3f} MPa (DAC {dac})")
                else:
                    self.v3_mpa = self.clamp_mpa(self.v3_mpa + self.mpa_step)
                    dac = self.mpa_to_dac(self.v3_mpa)
                    rospy.loginfo(f"[JoyMPA] v3 ++ -> {self.v3_mpa:.3f} MPa (DAC {dac})")
                self.publish_mpa_cmd()

            # 2本目 (v2 or v4) を増加: L2 + ○
            if self.button_pressed(self.BTN_CIRCLE, buttons):
                if self.active_pair == 1:
                    self.v2_mpa = self.clamp_mpa(self.v2_mpa + self.mpa_step)
                    dac = self.mpa_to_dac(self.v2_mpa)
                    rospy.loginfo(f"[JoyMPA] v2 ++ -> {self.v2_mpa:.3f} MPa (DAC {dac})")
                else:
                    self.v4_mpa = self.clamp_mpa(self.v4_mpa + self.mpa_step)
                    dac = self.mpa_to_dac(self.v4_mpa)
                    rospy.loginfo(f"[JoyMPA] v4 ++ -> {self.v4_mpa:.3f} MPa (DAC {dac})")
                self.publish_mpa_cmd()

        # R2 + □/○ でアクティブペアの各MPAを減少
        if self.button_is_held(self.BTN_R2, buttons):
            # 1本目 (v1 or v3) を減少: R2 + □
            if self.button_pressed(self.BTN_SQUARE, buttons):
                if self.active_pair == 1:
                    self.v1_mpa = self.clamp_mpa(self.v1_mpa - self.mpa_step)
                    dac = self.mpa_to_dac(self.v1_mpa)
                    rospy.loginfo(f"[JoyMPA] v1 -- -> {self.v1_mpa:.3f} MPa (DAC {dac})")
                else:
                    self.v3_mpa = self.clamp_mpa(self.v3_mpa - self.mpa_step)
                    dac = self.mpa_to_dac(self.v3_mpa)
                    rospy.loginfo(f"[JoyMPA] v3 -- -> {self.v3_mpa:.3f} MPa (DAC {dac})")
                self.publish_mpa_cmd()

            # 2本目 (v2 or v4) を減少: R2 + ○
            if self.button_pressed(self.BTN_CIRCLE, buttons):
                if self.active_pair == 1:
                    self.v2_mpa = self.clamp_mpa(self.v2_mpa - self.mpa_step)
                    dac = self.mpa_to_dac(self.v2_mpa)
                    rospy.loginfo(f"[JoyMPA] v2 -- -> {self.v2_mpa:.3f} MPa (DAC {dac})")
                else:
                    self.v4_mpa = self.clamp_mpa(self.v4_mpa - self.mpa_step)
                    dac = self.mpa_to_dac(self.v4_mpa)
                    rospy.loginfo(f"[JoyMPA] v4 -- -> {self.v4_mpa:.3f} MPa (DAC {dac})")
                self.publish_mpa_cmd()

        # ========= 目標角度制御 =========
        # target1: L1/R1 + □
        # target2: L1/R1 + ○

        # 増加側: L1 + □/○
        if self.button_is_held(self.BTN_L1, buttons):
            # target1++
            if self.button_pressed(self.BTN_SQUARE, buttons):
                self.target1_deg = self.clamp_angle(self.target1_deg + self.angle_step)
                self.target1_pub.publish(Float32(self.target1_deg))
                rospy.loginfo(f"[JoyMPA] target1 ++ -> {self.target1_deg:.2f} deg")

            # target2++
            if self.button_pressed(self.BTN_CIRCLE, buttons):
                self.target2_deg = self.clamp_angle(self.target2_deg + self.angle_step)
                self.target2_pub.publish(Float32(self.target2_deg))
                rospy.loginfo(f"[JoyMPA] target2 ++ -> {self.target2_deg:.2f} deg")

        # 減少側: R1 + □/○
        if self.button_is_held(self.BTN_R1, buttons):
            # target1--
            if self.button_pressed(self.BTN_SQUARE, buttons):
                self.target1_deg = self.clamp_angle(self.target1_deg - self.angle_step)
                self.target1_pub.publish(Float32(self.target1_deg))
                rospy.loginfo(f"[JoyMPA] target1 -- -> {self.target1_deg:.2f} deg")

            # target2--
            if self.button_pressed(self.BTN_CIRCLE, buttons):
                self.target2_deg = self.clamp_angle(self.target2_deg - self.angle_step)
                self.target2_pub.publish(Float32(self.target2_deg))
                rospy.loginfo(f"[JoyMPA] target2 -- -> {self.target2_deg:.2f} deg")

        # ========= 前回状態を更新 =========
        self.prev_buttons = list(buttons)
        self.prev_axes = list(axes)
    

if __name__ == '__main__':
    try:
        controller = JoyMPAController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass