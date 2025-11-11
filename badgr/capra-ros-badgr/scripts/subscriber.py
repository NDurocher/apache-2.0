#!/usr/bin/env python
import rospy
import torch
from std_msgs.msg import String
from sensor_msgs.msg import Image
from carla_msgs.msg import CarlaEgoVehicleControl
from cv_bridge import CvBridge

def callback():
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.throttle)
    image = rospy.wait_for_message('/carla/ego/front/image', Image)
    bridge = CvBridge()
    img = torch.tensor(bridge.imgmsg_to_cv2(image, desired_encoding='passthrough'))
    rospy.loginfo(img.shape)
    return Get_ImageResponse(img)
    
def cam_service():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/carla/ego/front/image', Image, callback)
    # s = rospy.Service('image_server', Get_Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    cam_service()