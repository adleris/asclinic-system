#!/usr/bin/env python

from asclinic_pkg.msg import PoseFloat32
import rospy

NAMESPACE = "asc/control"
NODE_NAME = f"{NAMESPACE}/kalman_filter"

# Constants for easy access to variables
X = 0
Y = 1
PHI = 2


class kalman_filter:
    def __init__(self):
        # TODO could make an initial pos topic that is sub to
        self.currPose = [0] * 3

        # ROS subs and publihing set up
        self.pose_publisher = rospy.Publisher(f"{NAMESPACE}/sys_pose", PoseFloat32, queue_size=1)
        rospy.Subscriber(f"{NAMESPACE}/change_to_pose", PoseFloat32, self.update_pose, queue_size=1)

    def update_pose(self, event):
        # This takes the change to pose and adds them to the current pose to be published
        self.currPose[X]     += event.x
        self.currPose[Y]     += event.y
        self.currPose[PHI]   += event.phi

        # Set up and publishing pose
        current_pose = PoseFloat32()
        current_pose.x = self.currPose[X]
        current_pose.y = self.currPose[X]
        current_pose.phi = self.currPose[PHI]
        self.pose_publisher.publish(current_pose)

if __name__ == "__main__":
    global NODE_NAME

    rospy.init_node(NODE_NAME)
    kFilter = kalman_filter()
    rospy.spin()

