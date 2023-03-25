#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy
# import rospy
# from std_msgs.msg import String
# from geometry_msgs.msg import Pose

class PathVisualiser():
    """This class manages path visualisation for debugging paths.

    It will overlay the position of the robot, the global goal, and the path the robot will take to get there.
    Depending on the data structure being used, map nodes could also be printed."""

    ROOM_IMAGE_FILE = "room-map.png"
    
    def __init__(self, image_file: str = ROOM_IMAGE_FILE):
        """Set up a path visualiser
        @param image_file File with map of room"""

        self.image = ImageManager(image_file)

        # subscribers to collate info
        self.global_target_sub = rospy.Subcriber("/Pathplanning/Goal", Pose, self._goal_target_sub)
        self.pose_sub = rospy.Subcriber("/DriveController/Pose", Pose, self._pose_update_sub)

        # publisher for new images
        # self.image_pub = rospy.Publisher("/Pathplanning/image_dummy", <<what data type??>>)
        # >> Soft publish by saving the files?
        # while not rospy.is_shutdown():
        #     image_pub.publish(image.new_image(self.global_target, self.pose))

        rospy.spin()

    def _global_target_sub(self, data):
        """Callback for update of global target"""
        self.global_target = (data.Point.x, data.Point.y)

    def _pose_update_sub(self, data):
        """callback for update of current pose"""
        self.pose = (data.Point.x, data.Point.y)


class ImageManager():

    ROOM_X_LEN = 20 # metres
    ROOM_Y_LEN = 10 # metres

    def __init__(self, image_file: str):
        """Set up ImageManager"""
        self.image = plt.imread(image_file)
        self.x_pixels = self.image.shape[1]
        self.y_pixels = self.image.shape[0]

    def new_image(self, goal: tuple[int, int], pose: tuple[int, int]) -> None:
        """Take in a goal location and current pose (as [x,y] list) and
        render them to an image
        TODO: Add path visualisation"""
        
        implot = plt.imshow(self.image)
        
        # plot global target
        goal_px = self.xy_to_pixels(goal)
        plt.scatter([goal_px[0]], [goal_px[1]], color='g', marker='.')

        # plot current pose
        pose_px = self.xy_to_pixels(pose)
        plt.scatter([pose_px[0]], [pose_px[1]], color='b', marker='.')

        # at some point, also draw in the path

        plt.show()
        
    def xy_to_pixels(self, point: tuple[int, int]) -> tuple[int, int]:
        """convert and (x,y) coordinate in the space to a pixel location on the image
        Note the orientation of the image axes:
        
        0 -------> x
        |
        |
        v 
        y
        """
        return (point[0] / self.ROOM_X_LEN * self.x_pixels,
                point[1] / self.ROOM_Y_LEN * self.y_pixels)

if __name__ == "__main__":
    PathVisualiser()