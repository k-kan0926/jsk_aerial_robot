#!/usr/bin/env python

import rospy

rospy.init_node("kinikun_joy_stick_instruction")

msg = """\

KINIKUN JOYSTICK INSTRUCTION
    Manipulation Air1 : L1 + up
    Manipulation Air2 : L1 + down
    Manipulation Air3 : L1 + left
    Manipulation Air4 : L1 + right
"""

print(msg)