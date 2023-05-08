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

# send a global target at the start

class PathPlanner():

    NODE_NAME = "/path/planner"

    def __init__(self):
        # data stores for generic map data
        self.verts : list[Vertex]
        self.edges : list[Edge]
        self.graph : Graph
        self.init_graph()


        # hold information about the path
        self.path : list[tuple[float, float]]
        self.path_idx : int = 0

        # holds the current edge we are traversing
        self.current_edge : Edge


        # data stores for current state
        self.curr_pose : Pose = Pose(Point(0,0,0), Quaternion(0,0,0,1))
        self.global_target : Point = None    
        ##### TODO: should this be a vertex? # I think point is fine, global target can import the coordinate list


        # set up subscribers
        rospy.Subscriber("/control/curr_pose", Pose, self.received_curr_pose)
        rospy.Subscriber("/main/global_target", Point, self.received_global_target)
        rospy.Subscriber("/obstacle_avoidance/detection", Bool, self.received_obstacle_detection)

        self.local_target_pub: rospy.Publisher = rospy.Publisher(self.NODE_NAME + "/next_target", Point, queue_size=10)

        rospy.loginfo("Initialised Path Planner.")

    def init_graph(self) -> None:
        """
        Initialises a graph with initial vertices and edges from node_planning
        """
        self.verts = node_planning.load_coords()
        self.edges = node_planning.load_edges()
        self.graph = Graph(self.verts, self.edges)

    def _recalculate_path(self, start: int, end: int) -> None:
        """
        Recalculate a path from start to end.

        NOTE: Assumes that we are at start.
        """

        self.path = self.graph.dijkstra(start, end)
        #TODO possible handling here for no path found
        self.path_idx = 0

    def publish_next_local_target(self):
        """
        Publish the next target node down the line for traversal
        """
        self.path_idx += 1
        self.current_edge = None #TODO update

        next_path_point = self.path[self.path_idx]
        next_point = Point(next_path_point[0], next_path_point[1], 0)
        self.local_target_pub.publish(next_point)

    def received_curr_pose(self, msg: Pose) -> None:
        """
        Update our knowledge of the current Pose

        :param msg: Pose
        """
        self.curr_pose = msg

    def received_global_target(self, msg: Point) -> None:
        """
        Update our global target. 
        
        Triggers a re-calculation of the path.

        :param msg: New target as Point
        """
        self.global_target = msg
        target_vertex_id : int = self._vertex_id_lookup(self.global_target)
        current_vertex_id : int = self.path[self.path_idx]
        self._recalculate_path(current_vertex_id, target_vertex_id)

        self.publish_next_local_target()

    def received_obstacle_detection(self, msg: Bool) -> None:
        """
        Run the 'detected obstacle' state transition.
        
        Triggers a re-calculation of the path.

        :param msg: Bool (always True)
        """
        # remove the current edge we are traversing from the map
        self.edges.remove(self.get_current_edge)
        # add a new vertex at the current pose
        # add an edge from new vertex back to previous vertex
        # recreate a new map

        self._recalculate_path(0,0) #TODO

    def _vertex_id_from_xy(self, point: tuple[float, float]) -> int:
        """
        Look up a vertex id in the vertex list based on its coordinates and return the ID

        :param point: 
        :return id: ID of the vertex being looked up 
        """
        for vertex in self.verts:
            # floating point equality is no good, but these should be generated form the same source
            if vertex.x == point.x and vertex.y == point.y:
                return vertex.id
        rospy.logerror("Could not find vertex with coords: " + str(point))
        return 0

    def _point_from_vertex_id(self, id: int) -> Point:
        """
        Look up Point coordinates of a Vertex based on its `id`.

        :param point: 
        :return id: ID of the vertex being looked up 
        """
        for vertex in self.verts:
            if vertex.id == id:
                return Point(x=vertex.x, y=vertex.y, z=0)
        rospy.logerror("Could not find vertex with ID: " + str(id))
        return Point(0,0,0)


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=False)
    pp = PathPlanner()
    rospy.spin()


#####
# TODO Need to add a path-parsing function which can figure out a mapping back to nodes/edges
#####
