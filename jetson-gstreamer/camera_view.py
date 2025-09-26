#!/usr/bin/env python2

import rospy
import cv2
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys

class CameraViewer:
    def __init__(self, image_topic="/camera/image_raw"):
        self.bridge = CvBridge()
        self.image_topic = image_topic
        
        # Initialize the ROS node
        rospy.init_node('camera_viewer', anonymous=True)
        
        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        
        print("Subscribed to {}".format(self.image_topic))
        print("Press 'q' to quit, 's' to save current frame")
        
        # Keep track of frame count for saving
        self.frame_count = 0
	self.time_diff = None
	self.fps = 0.0
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            if msg.encoding == "rgb8":
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                
        except Exception as e:
            print("Error converting image: {}".format(e))
            return
            
        # Display image info in the terminal (optional)
        height, width = cv_image.shape[:2]
        
	now = time.time()
	if self.time_diff is not None:
	  dt = now - self.time_diff
	  if dt > 0:
	    self.fps = 1.0 / dt
	self.time_diff = time.time()

        # Add some text overlay on the image
        cv2.putText(cv_image, "Size: {}x{}".format(width,height), (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(cv_image, "Topic: {}".format(self.image_topic), (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
	cv2.putText(cv_image, "FPS: {} Hz".format(self.fps), (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        
        # Show the image
        cv2.imshow("ROS Camera Feed", cv_image)
        
        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("Quitting...")
            rospy.signal_shutdown("User requested shutdown")
            cv2.destroyAllWindows()
        elif key == ord('s'):
            filename = "captured_frame_{:04d}.jpg".format(self.frame_count)
            cv2.imwrite(filename, cv_image)
            print("Saved frame as {}".format(filename))
            self.frame_count += 1

def main():
    # Default topic, but allow command line argument
    image_topic = "/camera/image_raw"
    if len(sys.argv) > 1:
        image_topic = sys.argv[1]
    
    try:
        viewer = CameraViewer(image_topic)
        rospy.spin()
    except rospy.ROSInterruptException:
        print("ROS node interrupted")
    except KeyboardInterrupt:
        print("Keyboard interrupt")
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
