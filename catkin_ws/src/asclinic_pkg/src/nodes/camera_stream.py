#!/usr/bin/env python3

import cv2
import os
import rospy
from sensor_msgs.msg import Image

def talker():
    camera = cv2.VideoCapture(0)
    # Setup publisher (<topic_name>, <message_type>, Queue size)
    pub = rospy.Publisher("/asc"+"/camera_image", Image, queue_size=10)
    # Init node
    rospy.init_node('video_stream', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        return_value, image = camera.read()
        # rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

# define a video capture object
camera = cv2.VideoCapture(0)
return_value, image = camera.read()
filename = 'savedImage.jpg'
cv2.imwrite(filename, image)
# After the loop release the cap object
camera.release()
# Destroy all the windows
# cv2.destroyAllWindows()