#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import pygame
import sys
import os

os.environ["SDL_VIDEODRIVER"] = "dummy"


class GamepadTeleop:
    def __init__(self):
        rospy.init_node("gamepad_teleop")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(20)  # 20 Hz for smoother control

        # Movement parameters
        self.max_linear_speed = 1.0
        self.max_angular_speed = 1.5

        # Deadzone for analog sticks (prevent drift)
        self.deadzone = 0.15

        # Initialize pygame
        pygame.init()
        pygame.joystick.init()

        # Check for connected controllers
        if pygame.joystick.get_count() == 0:
            rospy.logerr("No game controller found!")
            sys.exit(1)

        # Use first controller
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        rospy.loginfo("Connected to: {}".format(self.joystick.get_name()))
        rospy.loginfo("Number of axes: {}".format(self.joystick.get_numaxes()))
        rospy.loginfo("Number of buttons: {}".format(self.joystick.get_numbuttons()))

    def apply_deadzone(self, value):
        """Apply deadzone to analog stick values"""
        if abs(value) < self.deadzone:
            return 0.0
        # Scale the value so deadzone maps to 0 and 1.0 stays at 1.0
        return (
            (abs(value) - self.deadzone)
            / (1.0 - self.deadzone)
            * (1.0 if value > 0 else -1.0)
        )

    def run(self):
        rospy.loginfo("Gamepad teleop started.")
        rospy.loginfo("Left stick: forward/back, Right stick: turn")
        rospy.loginfo("Press button 0 (usually A/X) to quit")

        while not rospy.is_shutdown():
            pygame.event.pump()  # Process event queue

            twist = Twist()

            # Read analog sticks
            # Axis mapping varies by controller, these are common:
            # Axis 0: Left stick horizontal
            # Axis 1: Left stick vertical
            # Axis 2: Right stick horizontal
            # Axis 3: Right stick vertical

            # Left stick Y-axis for forward/backward (inverted because down is positive)
            if self.joystick.get_numaxes() >= 2:
                left_y = -self.joystick.get_axis(1)
                twist.linear.x = self.apply_deadzone(left_y) * self.max_linear_speed

            # Right stick X-axis for turning
            if self.joystick.get_numaxes() >= 3:
                right_x = self.joystick.get_axis(2)
                twist.angular.z = self.apply_deadzone(right_x) * self.max_angular_speed

            # Check for quit button (button 0, usually A on Xbox or X on PlayStation)
            if self.joystick.get_numbuttons() > 0:
                if self.joystick.get_button(0):
                    rospy.loginfo("Quit button pressed")
                    break

            self.pub.publish(twist)
            self.rate.sleep()

        pygame.quit()
        rospy.loginfo("Gamepad teleop stopped.")


if __name__ == "__main__":
    try:
        teleop = GamepadTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass
