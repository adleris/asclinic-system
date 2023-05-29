#!/usr/bin/env python

"""
Test file to mock the controller and generate new pose values according to the
poses requested by the path planner.

TODO: figure out pose/point
"""

import numpy as np
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Pose, Quaternion

class MotionGenerator():
    def __init__(self):
        self.curr_pose : Pose = Pose(Point(5.49216, 0.541588, 0),  Quaternion(0,0,0,1))
        self._point_list_idx : int = 0
        self._point_list : list[Point] = []
        self._POINTS_PER_EDGE : int = 10

        self.has_target : bool = False

        rospy.Subscriber("planner/next_target", Point, self.new_target_cb)
        rospy.Subscriber("planner/at_global_target", Bool, self.at_global_target_cb)

        self.pub : rospy.Publisher = rospy.Publisher("/control/curr_pose", Pose, queue_size=10)

        # publish new point every 1Hz
        rospy.Timer(rospy.Duration(1), self.publish_next_point_cb)

    def new_target_cb(self, data : Point) -> None:
        """interpolate a new path b/w current point and next target"""
        rospy.loginfo("Received new target")
        self._gen_new_points(data)
        self.has_target = True
        rospy.loginfo(self._point_list)

    def at_global_target_cb(self, data : Bool) -> None:
        """callback for arrival at global target"""
        rospy.loginfo("arrived at global target")
        self.has_target = False

    def publish_next_point_cb(self, timer_event) -> None:
        """publish new point location"""
        if not self.has_target:
            rospy.loginfo("pub: no target")
            return

        next_point : Point = self._get_next_point()
        if next_point is not None:
            rospy.loginfo("pub: point")
            self.pub.publish(Pose(next_point, Quaternion(0,0,0,1)))
            return
        else:
            rospy.loginfo("pub: End of path")
            return

    def _get_next_point(self) -> Point:
        """Get the next point in the list"""
        if not (self._point_list_idx < self._POINTS_PER_EDGE):
            return None
        else:
            self._point_list_idx += 1
            return self._point_list[self._point_list_idx]

    def _gen_new_points(self, target : Point) -> None:
        """Generate new list of points to publish"""
       
        X = np.linspace(self.curr_pose.position.x, target.x, self._POINTS_PER_EDGE)
        Y = np.linspace(self.curr_pose.position.y, target.y, self._POINTS_PER_EDGE)
        self._point_list = [Point(X[i],Y[i],0) for i in range(self._POINTS_PER_EDGE)]
        self._point_list_idx = 0

if __name__ == "__main__":
    rospy.init_node("test_motion_generator", anonymous=False)
    tm = MotionGenerator()
    rospy.spin()
