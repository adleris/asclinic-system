#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from asclinic_pkg.msg import PoseFloat32

NODE_NAME = "pose_converter"

class PoseConverter():
    def __init__(self):
        rospy.Subscriber("/asc/control/curr_pose", PoseFloat32, self.received_pose)
        self.pub : rospy.Publisher = rospy.Publisher("planner/curr_pose", Pose, queue_size=10)
    
    def received_pose(self, event : PoseFloat32) -> None:
        """Convert PoseFloat32 to Pose and republish"""
        next_pose : Pose = Pose(Point(event.x, event.y, 0), 
                                Quaternion(*quaternion_from_euler(0,0,event.phi)))
        self.pub.publish(next_pose)

if __name__ == "__main__":
    rospy.init_node("pose_converter", anonymous=False)
    pc = PoseConverter()
    rospy.spin()
