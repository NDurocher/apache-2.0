import rospy
import math
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import mpu6050

def euler_to_quaternion(eulers):

	roll, pitch, yaw = eulers

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

class IMUPublisher:
    def __init__(self):
        # Initialize the MPU6050 sensor
        self.mpu6050 = mpu6050.mpu6050(0x68)
        self.mpu6050.set_filter_range()

        # Initialize ROS components
        self.pub = rospy.Publisher('/sensor/imu', Imu, queue_size=10)
        rospy.init_node('IMU')

        # Store offsets as member variables
        self.linear_offset = {"x": 0, "y": 0, "z": 0}
        self.angular_offset = {"x": 0, "y": 0, "z": 0}

	self.pub_frequency = 30.0 # Hz
        self.rate = rospy.Rate(self.pub_frequency)

	self.alpha = 0.987 # complementary filter gain

        # Perform calibration
        self.calibrate_sensor()
	rospy.loginfo("sensor calibrated")
	
	self.eulers = np.zeros(3,)

    def read_sensor_data(self):
        imu_msg = Imu()

 	# Read accelerometer data and apply linear offset correction
 	accelerometer_data = self.mpu6050.get_accel_data()
 	imu_msg.linear_acceleration.x = accelerometer_data['x'] - self.linear_offset["x"]
	imu_msg.linear_acceleration.y = accelerometer_data['y'] - self.linear_offset["y"]
	imu_msg.linear_acceleration.z = accelerometer_data['z'] - self.linear_offset["z"]

 	# Read gyroscope data and apply angular offset correction
 	gyroscope_data = self.mpu6050.get_gyro_data()
	imu_msg.angular_velocity.x = gyroscope_data['x'] - self.angular_offset["x"]
	imu_msg.angular_velocity.y = gyroscope_data['y'] - self.angular_offset["y"]
  	imu_msg.angular_velocity.z = gyroscope_data['z'] - self.angular_offset["z"]

	# Convert to Rad/s
	imu_msg.angular_velocity.x *= np.pi / 180.0
        imu_msg.angular_velocity.y *= np.pi / 180.0
        imu_msg.angular_velocity.z *= np.pi / 180.0

	return imu_msg

    def calibrate_sensor(self):
        linear_calibration_buffer = []
        angular_calibration_buffer = []

        # Collect 100 samples for calibration
        while len(linear_calibration_buffer) < 100:
            imu_msg= self.read_sensor_data()
            linear_calibration_buffer.append({
                "x": imu_msg.linear_acceleration.x,
                "y": imu_msg.linear_acceleration.y,
                "z": imu_msg.linear_acceleration.z
            })
            angular_calibration_buffer.append({
                "x": imu_msg.angular_velocity.x,
                "y": imu_msg.angular_velocity.y,
                "z": imu_msg.angular_velocity.z
            })
            self.rate.sleep()

        # Compute linear offsets as the average of the calibration samples
        self.linear_offset["x"] = sum(sample["x"] for sample in linear_calibration_buffer) / len(linear_calibration_buffer)
        self.linear_offset["y"] = sum(sample["y"] for sample in linear_calibration_buffer) / len(linear_calibration_buffer)
        self.linear_offset["z"] = sum(sample["z"] for sample in linear_calibration_buffer) / len(linear_calibration_buffer)

        # Compute angular offsets as the average of the calibration samples
        self.angular_offset["x"] = sum(sample["x"] for sample in angular_calibration_buffer) / len(angular_calibration_buffer)
        self.angular_offset["y"] = sum(sample["y"] for sample in angular_calibration_buffer) / len(angular_calibration_buffer)
        self.angular_offset["z"] = sum(sample["z"] for sample in angular_calibration_buffer) / len(angular_calibration_buffer)

    def publish_data(self):
        while not rospy.is_shutdown():
            # Read sensor data
            imu_msg = self.read_sensor_data()

            # Orientation
            phi = math.atan(imu_msg.linear_acceleration.y / (math.sqrt(pow(imu_msg.linear_acceleration.x,2) + pow(imu_msg.linear_acceleration.z,2))))
            theta = math.atan(imu_msg.linear_acceleration.x / (math.sqrt(pow(imu_msg.linear_acceleration.y,2) + pow(imu_msg.linear_acceleration.z,2))))
            psi = math.atan(math.sqrt(pow(imu_msg.linear_acceleration.x,2) + pow(imu_msg.linear_acceleration.y,2)) / imu_msg.linear_acceleration.z)

	    ang_vels = np.array([imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z])

	    self.eulers = self.alpha * (self.eulers + ang_vels * 1/self.pub_frequency) + (1 - self.alpha) * np.array([phi, theta, psi])
#	    self.eulers += ang_vels * 1/self.pub_frequency

            quaternion = euler_to_quaternion(self.eulers)
            imu_msg.orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

            # Publish the corrected message
            self.pub.publish(imu_msg)
	    rospy.loginfo(theta)

            # Sleep to maintain the desired rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        imu_publisher = IMUPublisher()
	rospy.loginfo("Running")
        imu_publisher.publish_data()
    except rospy.ROSInterruptException:
        pass
