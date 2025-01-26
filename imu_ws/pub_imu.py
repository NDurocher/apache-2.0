import rospy
import numpy as np
from geometry_msgs.msg import AccelStamped
import mpu6050

class IMUPublisher:
    def __init__(self):
        # Initialize the MPU6050 sensor
        self.mpu6050 = mpu6050.mpu6050(0x68)
        self.mpu6050.set_filter_range()

        # Initialize ROS components
        self.pub = rospy.Publisher('imu_accel', AccelStamped, queue_size=10)
        rospy.init_node('Accel')

        # Store offsets as member variables
        self.linear_offset = {"x": 0, "y": 0, "z": 0}
        self.angular_offset = {"x": 0, "y": 0, "z": 0}
        self.rate = rospy.Rate(30)  # 30 Hz

        # Perform calibration
        self.calibrate_sensor()

    def read_sensor_data(self):
        accel_msg = AccelStamped()

        # Read accelerometer data
        accelerometer_data = self.mpu6050.get_accel_data()
        accel_msg.accel.linear.x = accelerometer_data['x']
        accel_msg.accel.linear.y = accelerometer_data['y']
        accel_msg.accel.linear.z = accelerometer_data['z']

        # Read gyroscope data
        gyroscope_data = self.mpu6050.get_gyro_data()
        accel_msg.accel.angular.x = gyroscope_data['x']
        accel_msg.accel.angular.y = gyroscope_data['y']
        accel_msg.accel.angular.z = gyroscope_data['z']

        # Read temperature (not used directly here)
        temperature = self.mpu6050.get_temp()

        return accel_msg, temperature

    def calibrate_sensor(self):
        linear_calibration_buffer = []
        angular_calibration_buffer = []

        # Collect 100 samples for calibration
        while len(linear_calibration_buffer) < 100:
            accel_msg, _ = self.read_sensor_data()
            linear_calibration_buffer.append({
                "x": accel_msg.accel.linear.x,
                "y": accel_msg.accel.linear.y,
                "z": accel_msg.accel.linear.z
            })
            angular_calibration_buffer.append({
                "x": accel_msg.accel.angular.x,
                "y": accel_msg.accel.angular.y,
                "z": accel_msg.accel.angular.z
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
            accel_msg, _ = self.read_sensor_data()

            # Apply linear offset correction
            accel_msg.accel.linear.x -= self.linear_offset["x"]
            accel_msg.accel.linear.y -= self.linear_offset["y"]
            accel_msg.accel.linear.z -= self.linear_offset["z"]

            # Apply angular offset correction
            accel_msg.accel.angular.x -= self.angular_offset["x"]
            accel_msg.accel.angular.y -= self.angular_offset["y"]
            accel_msg.accel.angular.z -= self.angular_offset["z"]

            # Publish the corrected message
            self.pub.publish(accel_msg)

            # Sleep to maintain the desired rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        imu_publisher = IMUPublisher()
        imu_publisher.publish_data()
    except rospy.ROSInterruptException:
        pass
