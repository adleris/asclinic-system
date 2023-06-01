#!/usr/bin/env python

import rospy
from asclinic_pkg.msg import PoseFloat32, LeftRightFloat32
from math import cos, sin 

NAMESPACE = "control"
NODE_NAME = "wheel_odometry"

# constants in m:
WHEEL_BASE_METERS = 0.218
WHEEL_RADIUS_METERS = 0.5 * 0.144
SAMPLE_PERIOD_SEC = 0.05

class wheel_odometry:
    def __init__(self):

        # ROS set up of publishing and subscribing
        self.currPhi = 0.0
        self.change_pos_publisher = rospy.Publisher(f"{NAMESPACE}/change_to_pose", PoseFloat32,  queue_size=1)
        rospy.Subscriber(f"{NAMESPACE}/wheel_angular_speeds", LeftRightFloat32, self.convert_wheel_speeds_to_pose_differences, queue_size=1)
        rospy.Subscriber(f"{NAMESPACE}/curr_pose", PoseFloat32, self.get_current_pose, queue_size=1)

    def convert_wheel_speeds_to_pose_differences(self, event):
        
        # needed for further calculations
        delta_s     = WHEEL_RADIUS_METERS * SAMPLE_PERIOD_SEC * (event.left + event.right) / 2
        delta_phi   = WHEEL_RADIUS_METERS * SAMPLE_PERIOD_SEC * (-event.left + event.right) / (2 * WHEEL_BASE_METERS)

        # publising the change in pose to be published
        changeToPose = PoseFloat32()
        # add the phi of the previous phi
        changeToPose.x   = delta_s * cos(self.currPhi + 0.5 * delta_phi)
        changeToPose.y   = delta_s * sin(self.currPhi + 0.5 * delta_phi)
        changeToPose.phi = delta_phi

        # rospy.loginfo(f"delta_x: {changeToPose.x} delta_y: {changeToPose.y} delta_phi: {changeToPose.phi}")

        self.change_pos_publisher.publish(changeToPose)


    def get_current_pose(self, event):
        # will just grab phi from the current phi
        self.currPhi = event.phi

if __name__ == "__main__":

    # setup of node
    rospy.init_node(NODE_NAME)
    odometry = wheel_odometry()
    rospy.spin()