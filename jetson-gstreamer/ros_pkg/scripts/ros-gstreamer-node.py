#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import numpy as np

class GstreamerCamera:
    def __init__(self):
        Gst.init(None)
        
        # Configure GStreamer pipeline for CSI camera
        # Modify this pipeline based on your specific camera setup
        sensor_id=0
        capture_width=1920
        capture_height=1080
        display_width=960
        display_height=540
        framerate=30
        flip_method=0

        pipeline_str = (
                       "nvarguscamerasrc sensor-id=%d ! "
                       "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
                       " nvvidconv flip-method=%d ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
                       "videoconvert ! video/x-raw, format=(string)BGR ! "
                       " appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false" % (sensor_id,capture_width, capture_height,framerate, flip_method, display_width, display_height)
                       )
        
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.sink = self.pipeline.get_by_name("sink")

        self.sink.connect("new-sample", self.on_new_sample)
        self.latest_frame = None

    def on_new_sample(self,sink):
        sample = sink.emit("pull-sample")
        if sample:
            buf = sample.get_buffer()
            caps = sample.get_caps()
            width = caps.get_structure(0).get_value("width")
            height = caps.get_structure(0).get_value("height")

            buffer = buf.extract_dup(0, buf.get_size())
            self.latest_frame = np.ndarray(
                (height, width, 3),
                buffer=buffer,
                dtype=np.uint8
            )
        return Gst.FlowReturn.OK

    def start(self):
        self.pipeline.set_state(Gst.State.PLAYING)
        
    def read(self):
        if self.latest_frame is not None:
            return True, self.latest_frame.copy()
        return False, None

def main():
    rospy.init_node('gstreamer_camera_publisher')
    pub = rospy.Publisher('camera/image_raw', Image, queue_size=10)
    bridge = CvBridge()
    camera = GstreamerCamera()
    camera.start()
    
    rate = rospy.Rate(30)  # 30Hz
    
    while not rospy.is_shutdown():
        ret, frame = camera.read()
        if ret:
            msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
