import Jetson.GPIO as GPIO
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np

# GPIO setup
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

right_motor_channels = [38, 40]  # in1, in2
left_motor_channels = [19, 21]  # in3, in4

GPIO.setup(right_motor_channels, GPIO.OUT)
GPIO.setup(left_motor_channels, GPIO.OUT)

class GPIOController:
    def __init__(self):
        rospy.init_node('gpio_controller', anonymous=True)
        rospy.Subscriber('/cmd_vel', Twist, self.handle_twist_msg)

        # GPIO setup
        #GPIO.setmode(GPIO.BOARD)
        #GPIO.setwarnings(False)

        #right_motor_channels = [38, 40]  # in1, in2
        #left_motor_channels = [19, 21]  # in3, in4

        #GPIO.setup(self.right_motor_channels, GPIO.OUT)
        #GPIO.setup(self.left_motor_channels, GPIO.OUT)

        rospy.loginfo("GPIO setup")

    def update_GPIO(self, msg):
        if msg.angular.z == 0:
          if msg.linear.x == 1:
              GPIO.output(40, GPIO.LOW)
              GPIO.output(19, GPIO.HIGH)
              GPIO.output(38, GPIO.HIGH)
              GPIO.output(21, GPIO.LOW)
          elif msg.linear.x == -1:
              GPIO.output(40, GPIO.HIGH)
              GPIO.output(19, GPIO.LOW)
              GPIO.output(38, GPIO.LOW)
              GPIO.output(21, GPIO.HIGH)
          elif msg.linear.x == 0:
              GPIO.output(40, GPIO.LOW)
              GPIO.output(19, GPIO.LOW)
              GPIO.output(38, GPIO.LOW)
              GPIO.output(21, GPIO.LOW)
        else:
          if msg.angular.z > 0:
            GPIO.output(40, GPIO.LOW)
            GPIO.output(19, GPIO.LOW)
            GPIO.output(38, GPIO.HIGH)
            GPIO.output(21, GPIO.HIGH)
          elif msg.angular.z < 0:
            GPIO.output(40, GPIO.HIGH)
            GPIO.output(19, GPIO.HIGH)
            GPIO.output(38, GPIO.LOW)
            GPIO.output(21, GPIO.LOW)

    def handle_twist_msg(self, msg):
        left_cmd = np.clip(msg.linear.x + msg.angular.z/2, -1, 1)
        right_cmd = np.clip(msg.linear.x - msg.angular.z/2, -1,1)
        self.update_motor(left_cmd, left_motor_channels)
        self.update_motor(right_cmd, right_motor_channels)

    def update_motor(self, cmd, motor_channels):
        if cmd > 0.0:
          GPIO.output(motor_channels[0], GPIO.HIGH)
          GPIO.output(motor_channels[1], GPIO.LOW)
        elif cmd < 0.0:
          GPIO.output(motor_channels[0], GPIO.LOW)
          GPIO.output(motor_channels[1], GPIO.HIGH)
        else:
          GPIO.output(motor_channels[0], GPIO.LOW)
          GPIO.output(motor_channels[1], GPIO.LOW)

if __name__ == "__main__":
    gpio_controller = GPIOController()
    try:
        rospy.loginfo("Running")
        rospy.spin()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
