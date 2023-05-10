#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from asclinic_pkg.msg import ServoPulseWidth

class CameraServo:

    def __init__(self):
        self.servo_pub = rospy.Publisher("/asc"+"/set_servo_pulse_width", ServoPulseWidth, queue_size=10)

def camera_publisher_setup():
    # Setup publisher (<topic_name>, <message_type>, Queue size)
    pub = rospy.Publisher('set_servo_pulse_width', ServoPulseWidth, queue_size=10)
    # Init node
    rospy.init_node('set_servo_pulse_width', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "Matt Pub: %s" % rospy.get_time()
        # rospy.loginfo(hello_str)

        # pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':

    global node_name
    node_name = "servo_control"
    rospy.init_node(node_name, anonymous=True)
    servo_control_object = CameraServo()
    rospy.spin()