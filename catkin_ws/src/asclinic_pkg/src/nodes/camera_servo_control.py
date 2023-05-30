#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import UInt32

from asclinic_pkg.msg import ServoPulseWidth
import numpy as np
from numpy.polynomial import Chebyshev

class CameraServo:

    def __init__(self):
        # Setup publisher to talk with i2c_for_servos
        self.servo_pub = rospy.Publisher("/asc"+"/set_servo_pulse_width", ServoPulseWidth, queue_size=10)
        # Subscriber for pan control
        rospy.Subscriber("/asc"+"/pan_deg", Int32, self.adjust_pan_angle)
        # Subscriber for tilt control
        rospy.Subscriber("/asc"+"/tilt_deg", Int32, self.adjust_tilt_angle)
        # Setup publisher to track 
        self.pub_pan = rospy.Publisher("/asc"+"/pose_camera_pan", Int32, queue_size=10)

    def adjust_pan_angle(self, data):
        # pan 90 deg = 2150
        # pan -90 deg = 800
        # convert angle to PWM
        pwm_points = np.array([800, 1480, 2150])
        deg_points = np.array([-90, 0, 90])
        angle = data.data
        convertion_factor = Chebyshev.fit(deg_points, pwm_points, deg=1)

        # package data to be published
        PWM_string = ServoPulseWidth()
        PWM_string.channel = 3
        PWM_string.pulse_width_in_microseconds = int(convertion_factor(angle))

        self.servo_pub.publish(PWM_string)
        self.pub_pan.publish(angle)
    
    def adjust_tilt_angle(self, data):
        # tilt 0 deg = 1360
        # tilt 45 deg = 1930
        # convert angle to PWM
        pwm_points = np.array([1360, 1930])
        deg_points = np.array([0, 45])
        angle = data.data
        convertion_factor = Chebyshev.fit(deg_points, pwm_points, deg=1)

        # package data to be published
        PWM_string = ServoPulseWidth()
        PWM_string.channel = 4
        PWM_string.pulse_width_in_microseconds = int(convertion_factor(angle))

        self.servo_pub.publish(PWM_string)



# def camera_publisher_setup():



#     # pan 90 deg = 2150
#     # pan -90 deg = 800
#     # tilt 0 deg = 1360
#     # tilt 45 deg = 1930
#     # Setup publisher (<topic_name>, <message_type>, Queue size)
#     pub = rospy.Publisher('set_servo_pulse_width', ServoPulseWidth, queue_size=10)
#     # Init node
#     rospy.init_node('set_servo_pulse_width', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         # hello_str = "Matt Pub: %s" % rospy.get_time()
#         # rospy.loginfo(hello_str)

#         # pub.publish(hello_str)
#         rate.sleep()

if __name__ == '__main__':

    global node_name
    node_name = "servo_control"
    rospy.init_node(node_name)
    servo_control_object = CameraServo()
    rospy.spin()
