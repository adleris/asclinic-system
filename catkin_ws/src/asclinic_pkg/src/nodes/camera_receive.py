#!/usr/bin/env python3
import cv2
# Import the ROS-Python package
import rospy
#import rospkg

# Import the standard message types
from std_msgs.msg import UInt32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraReceive:

    def __init__(self):
        self.cv_bridge = CvBridge()
        rospy.Subscriber("/asc"+"/camera_image", Image, self.callback)

    def callback(self, image):
        # print("received")
        # print(msg)
        # cv_bridge = CvBridge()
        cv2.imshow('frame', self.cv_bridge.imgmsg_to_cv2(image, desired_encoding='passthrough'))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return 
    
# def listener():

#     # In ROS, nodes are uniquely named. If two nodes with the same
#     # name are launched, the previous one is kicked off. The
#     # anonymous=True flag means that rospy will choose a unique
#     # name for our 'listener' node so that multiple listeners can
#     # run simultaneously.
#     node_name = "camera_receive"
#     rospy.init_node(node_name)
    
#     # Setup subscriber (<topic_name>, <message_type>, Queue size)
#     rospy.Subscriber("/asc"+"/camera_image", Image, callback)
#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()
#     camera.release()


if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "camera_receive"
    rospy.init_node(node_name)
    camera_capture_object = CameraReceive()
    # Spin as a single-threaded node
    rospy.spin()

    # Release the camera
    # camera_capture_object.cam.release()
    # Close any OpenCV windows
    cv2.destroyAllWindows()