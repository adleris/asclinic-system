#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from asclinic_pkg.msg import PoseFloat32
from std_msgs.msg import Float32

from utilities import point_distance

NODE_NAME = "pose_converter"

class PoseConverter():
    def __init__(self):
        rospy.Subscriber("/asc/control/curr_pose", PoseFloat32, self.received_pose)
        self.pub : rospy.Publisher = rospy.Publisher("planner/curr_pose", Pose, queue_size=10)
    
        self.target : Pose = None
        self.dist_pub = rospy.Publisher("planner/dist_to_node", Float32, queue_size=10)
        rospy.Subscriber("/planner/next_target", Point, self.recd_target)
    
    def received_pose(self, event : PoseFloat32)        :
        """Convert PoseFloat32 to Pose and republish"""
        next_pose : Pose = Pose(Point(event.x, event.y, 0), 
                                Quaternion(*quaternion_from_euler(0,0,event.phi)))
        self.pub.publish(next_pose)
        if self.target is not None:
            self.dist_pub.publish(point_distance(next_pose.position, self.target))

    def recd_target(self, event):
        self.target = event


if __name__ == "__main__":
    rospy.init_node("pose_converter", anonymous=False)
    pc = PoseConverter()
    rospy.spin()
