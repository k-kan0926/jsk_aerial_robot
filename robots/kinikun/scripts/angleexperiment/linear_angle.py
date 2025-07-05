#!/usr/bin/env python3
"""
interactive_param_loader.py
--------------------------------
A ROS node that interactively accepts target values (p1, p2) and a duration
from the command line, linearly interpolates the values, publishes them to the
`mpa_cmd` topic as voltage commands (Vector3) and updates ROS parameters
`/p1_value`, `/p2_value` on every step. After reaching the target, it holds for
5 s, resets both values to zero, and prompts for new input.
"""
import sys
import rospy
from geometry_msgs.msg import Vector3

FREQUENCY = 10  # [Hz] publishing frequency
RESET_HOLD = 5.0  # [s] time to hold at target before reset


def linspace(start: float, end: float, duration: float, freq: int):
    """Yield evenly spaced values from `start` to `end` over `duration` seconds."""
    steps = max(1, int(duration * freq))
    for i in range(steps + 1):
        yield start + (end - start) * (i / steps)


def publish_zero(pub):
    """Publish zero voltage and reset ROS params."""
    zero_msg = Vector3()
    pub.publish(zero_msg)
    rospy.set_param('/p1_value', 0.0)
    rospy.set_param('/p2_value', 0.0)
    rospy.loginfo("p1_value and p2_value have been reset to 0.0")


def prompt_user():
    """Prompt the user for target p1, p2, and duration until valid input is given."""
    while True:
        try:
            raw = input("Enter target p1 p2 duration (e.g. '0.3 0.4 8'): ")
        except (EOFError, KeyboardInterrupt):
            return None  # Signal shutdown

        parts = raw.strip().split()
        if len(parts) != 3:
            print("Please enter **three** numbers: p1 p2 duration")
            continue
        try:
            p1, p2, duration = map(float, parts)
            if duration <= 0:
                print("Duration must be positive.")
                continue
            return p1, p2, duration
        except ValueError:
            print("Invalid numbers. Try again.")
        


def main():
    rospy.init_node("interactive_param_loader", anonymous=True)
    pub = rospy.Publisher("mpa_cmd", Vector3, queue_size=10)
    rate = rospy.Rate(FREQUENCY)

    # Initialise ROS parameters if they don't exist
    if not rospy.has_param('/p1_value'):
        rospy.set_param('/p1_value', 0.0)
    if not rospy.has_param('/p2_value'):
        rospy.set_param('/p2_value', 0.0)

    while not rospy.is_shutdown():
        user_input = prompt_user()
        if user_input is None or rospy.is_shutdown():
            break  # Shutdown requested

        target_p1, target_p2, duration = user_input
        current_p1 = rospy.get_param('/p1_value')
        current_p2 = rospy.get_param('/p2_value')

        rospy.loginfo(f"Moving p1: {current_p1} -> {target_p1}, p2: {current_p2} -> {target_p2} over {duration}s")

        p1_seq = linspace(current_p1, target_p1, duration, FREQUENCY)
        p2_seq = linspace(current_p2, target_p2, duration, FREQUENCY)

        for p1, p2 in zip(p1_seq, p2_seq):
            if rospy.is_shutdown():
                break
            v1 = p1 * 4096 / 0.9
            v2 = p2 * 4096 / 0.9
            pub.publish(Vector3(x=v1, y=v2, z=0.0))
            rospy.set_param('/p1_value', p1)
            rospy.set_param('/p2_value', p2)
            rate.sleep()

        if rospy.is_shutdown():
            break

        rospy.loginfo("Target reached. Holding for %s seconds…", RESET_HOLD)
        rospy.sleep(RESET_HOLD)
        publish_zero(pub)

    rospy.loginfo("Shutting down interactive_param_loader.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
