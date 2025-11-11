#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from carla_msgs.msg import CarlaEgoVehicleControl


def talker(topic, msg_type, freq=10):
    pub = rospy.Publisher(topic, msg_type, queue_size=10)
    rospy.init_node("talker", anonymous=True)
    rate = rospy.Rate(freq)  # 10hz
    for i in range(11):
        # hello_str = "hello world %s" % rospy.get_time()
        controls = CarlaEgoVehicleControl()
        controls.throttle = 0.8
        controls.steer = 0.0
        # rospy.loginfo(hello_str)
        pub.publish(controls)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker("/carla/ego/vehicle_control_cmd", CarlaEgoVehicleControl)
    except rospy.ROSInterruptException:
        pass
