#!/usr/bin/env python
import rospy

from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Point

from utilities import point_distance

class ObstacleAvoidance:
    """
    Set up an obstacle avoidance node.

    # Process

    - If an obstacle is detected by the lidar parser, this node will publish
    the new detection to rest of the pathplanning system. It will log the
    position the robot was in when the obstacle was detected.
    - until we have travelled a certain distance from the obstacle position,
    do not publish any other detections. This is implemented in the code via
    the `detection_lock`. 
    - Once we have travelled this distance, release the `detection_lock`

    # ROS Subscribers

    - current pose
    - `/asc/sensors/lidarScan` : `LaserScan`

    # ROS Publishers

    - `/asc/path/obstacle_avoidance/detection`: `Bool` 
    """

    OBSTACLE_AVOIDANCE_LOCK_DISTANCE = 0.5 # metres

    def __init__(self, node_name: str):
        
        self.detection_lock: bool = False
        # The current position
        self.curr_pos: Point = Point(0,0,0)
        """
        `obstacle_pos` tracks the position of the current obstacle we are
        avoiding. If set to None, there is no obstacle being tracked.
        If `detection_lock` is False then `obstacle_pos` should be None
        """
        self.obstacle_pos: Point = None

        self.detection_publisher = rospy.Publisher(node_name+"/detection", Bool, queue_size=10)
        rospy.Subscriber("planner/curr_pose", Pose, self.poseReceivedCallback)
        rospy.Subscriber("sensors/obstacle_scan", Bool, self.obstacleInRangeCallback)
        rospy.loginfo("Obstacle avoidance started.")


    def poseReceivedCallback(self, msg):
        """
        Subscriber for pose update. Handles the detection lock/unlock.

        :param msg: {Pose}
        """
        self.curr_pos = msg.position
        rospy.logdebug("pose update:\n" + str(msg.position))

        # we are not in avoidance
        if not self.detection_lock:
            return

        # release the lock if we've travelled far enough
        # since obstacle pos is checked first, this should hopefully? prevent issues on other threads
        if (obstacle_point is not None) and (point_distance(obstacle_point, self.pose.position) > self.OBSTACLE_AVOIDANCE_LOCK_DISTANCE):
            self.obstacle_pos = None
            self.detection_lock = False
            rospy.loginfo("Releasing detection lock")


    def obstacleInRangeCallback(self, msg):
        """
        Subscriber for an obstacle detection.

        :param msg: {Bool}

        Publishes 'True' when an obstacle is observed in range.
        
        *Should* only publish once per obstacle, as it will wait for the robot
        to move out of a radius around the last obstacle before it will consider
        new obstacles.
        """
        if not msg.data:
            return
        
        # if this obstacle is considered a new detection
        if not self.detection_lock:
            self.detection_publisher.publish(msg)
            self.obstacle_pos = self.curr_pos
            self.detection_lock = True
            rospy.loginfo("publishing detection and creating lock")
        else:
            rospy.logdebug("ignoring detection due to lock")
            return


if __name__ == '__main__':

    node_name : str = "obstacle_avoidance"
    rospy.init_node(node_name, anonymous=False)
    obstacle_avoidance = ObstacleAvoidance()
    
    rospy.spin()
