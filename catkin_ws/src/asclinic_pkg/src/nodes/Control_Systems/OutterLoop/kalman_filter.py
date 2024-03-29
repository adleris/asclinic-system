#!/usr/bin/env python3

from asclinic_pkg.msg import PoseFloat32
from std_msgs.msg import Int32, Bool
from math import pi
import rospy

NAMESPACE = "control"
NODE_NAME = "kalman_filter"

class kalman_filter:
    def __init__(self):
        # TODO could make an initial pos topic that is sub to
        self.currPose = PoseFloat32(0, 0, 0)
        self.initial_pose_set = False
        self.inRotation = False

        # ROS subs and publihing set up
        self.pose_publisher = rospy.Publisher(f"{NAMESPACE}/curr_pose", PoseFloat32, queue_size=1)
        rospy.Subscriber(f"{NAMESPACE}/change_to_pose", PoseFloat32, self.update_pose, queue_size=1)
        rospy.Subscriber(f"initial_pose", PoseFloat32, self.set_initial_pose, queue_size=1)
        rospy.Subscriber(f"pose_camera_estimation", PoseFloat32, self.set_pose, queue_size=1)
        rospy.Subscriber("/asc/control/in_rotation", Bool, self.set_inRotation, queue_size=1)

        

    def set_initial_pose(self, event):
        # sets the a new pose once, used for the intial pose
        rospy.loginfo("[Kalman] Received Initial Pose")
        if not self.initial_pose_set:
            self.set_pose(event)
            self.initial_pose_set = True
    
    def set_inRotation(self, event):
        self.inRotation = event.data

    def set_pose(self, event):
        # This takes the change to pose and adds them to the current pose to be published
        if not self.initial_pose_set:
            self.initial_pose_set = True
        print(self.inRotation)
        if self.inRotation:
            return
        
        self.currPose.x     = event.x
        self.currPose.y     = event.y
        self.currPose.phi   = event.phi

        # Correcting theta to keep in range 
        if self.currPose.phi <= -pi:
            self.currPose.phi += 2 * pi
        elif self.currPose.phi > pi:
            self.currPose.phi -= 2 * pi

        # Publish pose
        self.pose_publisher.publish(self.currPose)

    def update_pose(self, event):
        # This takes the change to pose and adds them to the current pose to be published
        self.currPose.x     += event.x
        self.currPose.y     += event.y
        self.currPose.phi   += event.phi

        # Correcting theta to keep in range 
        if self.currPose.phi <= -pi:
            self.currPose.phi += 2 * pi
        elif self.currPose.phi > pi:
            self.currPose.phi -= 2 * pi

        # Set up and publishing pose
        # rospy.loginfo(f"x: {self.currPose.x} y: {self.currPose.y} phi: {self.currPose.phi}")
        self.pose_publisher.publish(self.currPose)

if __name__ == "__main__":

    rospy.init_node(NODE_NAME)
    kFilter = kalman_filter()
    rospy.spin()

