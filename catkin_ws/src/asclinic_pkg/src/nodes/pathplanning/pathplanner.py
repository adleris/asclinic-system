#!/usr/bin/env python

"""
This file contains the driver node for the whole path-planning system
"""

import numpy as np
import rospy

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Point, Quaternion

from dijkstra import Graph, Vertex, Edge
import node_planning 
from obstacle_avoidance import ObstacleAvoidance
from visualise_path import PathVisualiser, ImageManager

class PathPlanner():

    NODE_NAME = "/asc/path/planner"

    def __init__(self):
        # data stores for generic data
        self.verts : list[Vertex]
        self.edges : list[Edge]
        self.graph : Graph
        self.init_graph()
        self.path : list[tuple[float, float]]
        self.path_idx : int = 0


        # data stores for current state
        self.curr_pose : Pose = Pose(Point(0,0,0), Quaternion(0,0,0,1))
        self.global_target : Point = None    ##### TODO: should this be a vertex?


        # set up subscribers
        rospy.Subscriber("/asc/control/curr_pose", Pose, self.received_curr_pose)
        rospy.Subscriber("/asc/main/global_target", Point, self.received_global_target)
        rospy.Subscriber("/asc/path/obstacle_avoidance/detection", Bool, self.received_obstacle_detection)

        rospy.Publisher(self.NODE_NAME + "/next_target", Point, queue_size=10)

    def init_graph(self) -> None:
        self.verts = node_planning.load_coords()
        self.edges = node_planning.load_edges()
        self.graph = Graph(self.verts, self.edges)

    def recalculate_path(self, start: int, end: int):
        self.path = self.graph.dijkstra(start, end)
        self.path_idx = 0

    def received_curr_pose(self, msg) -> None:
        return
    def received_global_target(self, msg) -> None:
        return
    def received_obstacle_detection(self, msg) -> None:
        return

if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=False)
    pp = PathPlanner()
    rospy.spin()



