import Jetson.GPIO as GPIO
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np

# GPIO setup
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

motor_1_channels = [40, 38]  # in1, in2
motor_2_channels = [19, 21]  # in3, in4

GPIO.setup(motor_1_channels, GPIO.OUT)
GPIO.setup(motor_2_channels, GPIO.OUT)

class GPIOController:
    def __init__(self):
        rospy.init_node('gpio_controller', anonymous=True)
        rospy.Subscriber('/cmd_vel', Twist, self.update_GPIO)

        self.linear_cmd = 0 # [-1,0,1]
        self.angular_cmd = 0 # [-pi/2,pi/2]
        motor_pwm_channel = 33
        GPIO.setup(motor_pwm_channel, GPIO.OUT, initial=GPIO.HIGH)
        # self.p = GPIO.PWM(motor_pwm_channel, 50)
        # self.val = 10
        # self.incr = 10
        # self.p.start(self.val)

    # def __del__(self):
        # self.p.stop()

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

    def update_command(self, incoming_cmd):
        self.linear_cmd = incoming_cmd["linear"]
        self.angular_cmd = incoming_cmd["angular"]
        # self.linear_cmd = np.clip(self.linear_cmd + incoming_cmd["linear"], -1, 1)
        # self.angular_cmd = np.clip(self.angular_cmd + incoming_cmd["angular"], -np.pi/2, np.pi/2)

    #def handle_key_input(self, msg):
        #key = msg.data

        #new_cmd = {'linear':0, 'angular':0}
        #if key == 'right':
        #    print("Right arrow pressed!")
        #    new_cmd['linear'] = 0
        #    new_cmd['angular'] = -np.pi/2
        #elif key == 'left':
        #    print("Left arrow pressed!")
        #    new_cmd['linear'] = 0
        #    new_cmd['angular'] = np.pi/2
        #elif key == 'up':
        #    print("Up arrow pressed!")
        #    new_cmd['linear'] = 1
        #    new_cmd['angular'] = 0
        #elif key == 'down':
        #    print("Down arrow pressed!")
        #    new_cmd['linear'] = -1
        #    new_cmd['angular'] = 0
        #elif key == 'space':
        #    print("STOP!!!!")
        #    new_cmd['linear'] = 0
        #    new_cmd['angular'] = 0
        #
        #self.update_command(new_cmd)
        #self.update_GPIO(msg)
        # elif key == "w":
        #     if (self.val + self.incr) <= 100:
        #         self.val += self.incr
        #         self.p.ChangeDutyCycle(self.val)
        #         print(f"speed: {self.val}")
        # elif key == "s":
        #     if (self.val - self.incr) >= 0:
        #         self.val -= self.incr
        #         self.p.ChangeDutyCycle(self.val)
        #         print(f"speed: {self.val}")

if __name__ == "__main__":
    gpio_controller = GPIOController()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
