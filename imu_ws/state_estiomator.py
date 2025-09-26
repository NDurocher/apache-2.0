import rospy
import sys
from geometry_msgs.msg import AccelStamped, Quaternion
from tf.transformations import quaternion_from_euler

class StateEstimator:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('state_estimator', anonymous=True)

        # Initialize velocity and position variables
        self.velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.angular_velocity = {"x": 0, "y": 0, "z": 0}
        self.orientation = {"roll": 0, "pitch": 0, "yaw": 0}
        self.quaternion = Quaternion()

        # Time tracking for integration
        self.last_time = rospy.Time.now()

        self.rec_buffer = []

        # Subscribe to the imu_accel topic
        rospy.Subscriber("imu_accel", AccelStamped, self.imu_callback)

        # Log initialization
        rospy.loginfo("IMU Processor initialized and subscribed to 'imu_accel'")

    def imu_callback(self, msg):
        """
        Callback function to process incoming IMU data.
        Calculates velocity and position based on acceleration.
        """

        self.rec_buffer.append(msg)
        if len(self.rec_buffer) > 9:
            self.rec_buffer = self.rec_buffer[1:]

        filtered_msg = self.moving_average()

        # Get the current time and compute time delta (dt)
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()  # Time in seconds
        self.last_time = current_time

        # Extract acceleration values
        accel_x = filtered_msg.accel.linear.x
        accel_y = filtered_msg.accel.linear.y
        accel_z = filtered_msg.accel.linear.z

        # Integrate acceleration to calculate velocity
        self.velocity['x'] += accel_x * dt
        self.velocity['y'] += accel_y * dt
        self.velocity['z'] += accel_z * dt

        # Integrate velocity to calculate position
        self.position['x'] += self.velocity['x'] * dt
        self.position['y'] += self.velocity['y'] * dt
        self.position['z'] += self.velocity['z'] * dt

        # Extract angular velocity from the message
        self.angular_velocity["x"] += filtered_msg.accel.angular.x * dt
        self.angular_velocity["y"] += filtered_msg.accel.angular.y * dt
        self.angular_velocity["z"] += filtered_msg.accel.angular.z * dt

        # Integrate angular velocities to update orientation (in radians)
        self.orientation["roll"] += self.angular_velocity["x"] * dt
        self.orientation["pitch"] += self.angular_velocity["y"] * dt
        self.orientation["yaw"] += self.angular_velocity["z"] * dt

        # Convert orientation to quaternion
        self.quaternion = quaternion_from_euler(
            self.orientation["roll"],
            self.orientation["pitch"],
            self.orientation["yaw"]
        )
        
        # Log the computed values
        # rospy.loginfo("Acceleration - x: {:.2f}, y: {:.2f}, z: {:.2f}".format(accel_x, accel_y, accel_z))
        # rospy.loginfo("Velocity     - x: {:.2f}, y: {:.2f}, z: {:.2f}".format(self.velocity['x'], self.velocity['y'], self.velocity['z']))
        print("\rPosition     - x: {:.2f}, y: {:.2f}, z: {:.2f}".format(self.position['x'], self.position['y'], self.position['z'])),
	sys.stdout.flush()
	# rospy.loginfo("Orientation  - r: {:.2f}, p: {:.2f}, y: {:.2f}".format(self.orientation['roll'], self.orientation['pitch'], self.orientation['yaw']))

    def moving_average(self):
        msg_out = AccelStamped()
        
        msg_out.accel.linear.x = sum(msg.accel.linear.x for msg in self.rec_buffer)/len(self.rec_buffer)
        msg_out.accel.linear.y = sum(msg.accel.linear.y for msg in self.rec_buffer)/len(self.rec_buffer)
        msg_out.accel.linear.z = sum(msg.accel.linear.z for msg in self.rec_buffer)/len(self.rec_buffer)

        msg_out.accel.angular.x = sum(msg.accel.angular.x for msg in self.rec_buffer)/len(self.rec_buffer)
        msg_out.accel.angular.y = sum(msg.accel.angular.y for msg in self.rec_buffer)/len(self.rec_buffer)
        msg_out.accel.angular.z = sum(msg.accel.angular.z for msg in self.rec_buffer)/len(self.rec_buffer)

        return msg_out


    def start(self):
        """
        Start the ROS node and keep it alive.
        """
        rospy.spin()

if __name__ == "__main__":
    try:
        state_estimator = StateEstimator()
        state_estimator.start()
    except rospy.ROSInterruptException:
        pass
