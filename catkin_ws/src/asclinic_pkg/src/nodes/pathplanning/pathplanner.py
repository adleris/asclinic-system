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
from obstacle_avoidance import ObstacleAvoidance, point_distance
#from visualise_path import PathVisualiser, ImageManager

# send a global target at the start

NODE_NAME = "planner"

class PathPlanner():

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
        self.global_target_id : int = -1


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

        # TODO: add handling for arriving at global target!

    def received_curr_pose(self, msg: Pose) -> None:
        """
        Update our knowledge of the current Pose

        :param msg: Pose
        """
        self.curr_pose = msg

        # check if we had arrived at the next local vertex and should 
        # publish the next segment of the path
        if self._check_if_at_target_node():
            self.publish_next_local_target()

    def received_global_target(self, msg: Point) -> None:
        """
        Update our global target. 
        
        Triggers a re-calculation of the path.

        :param msg: New target as Point
        """
        self.global_target = msg
        self.global_target_id : int = self._vertex_id_lookup(self.global_target)
        current_vertex_id : int = self.path[self.path_idx]
        self._recalculate_path(current_vertex_id, self.global_target_id)

        self.publish_next_local_target()

    def received_obstacle_detection(self, msg: Bool) -> None:
        """
        Run the 'detected obstacle' state transition.
        
        Triggers a re-calculation of the path.

        :param msg: Bool (always True)
        """
        # pull out important values from the path
        old_start : int = self.path[self.path_idx-1]
        old_end   : int = self.path[self.path_idx]
        self._remove_edge(old_start, old_end)

        # add a new vert at the current pose, and an edge from that vert back to the last vert
        new_vert : Vertex(self.curr_pose.position.x, self.curr_pose.position.y)
        self.verts.append(new_vert)
        new_edge : Edge(new_vert.id, old_start, 1.0)
        self.edges.append(new_edge)

        # recalculate the full path from this new point
        self._recalculate_path(new_vert.id, self.global_target_id)

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


    def _remove_edge(self, start: int, end: int):
        """
        Remove the edge between start vertex id and end vertex id
        """
        for e in range(len(self.edges)):
            if self.edges[e].start == start and self.edges[e].end == end:
                self.edges.pop(e)

    def _check_if_at_target_node(self) -> bool:
        """
        Check if we are within a radius of the target position.

        For a first implementation, uses a very simple check on radial distance from target.

        This is used to trigger transitions to a new segment of the path.
        """
        target_position : Point = self._point_from_vertex_id(self.path[self.path_idx])
        return point_distance(self.curr_pose.position, target_position) < 0.2

if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=False)
    pp = PathPlanner()
    rospy.spin()


#####
# TODO Need to add a path-parsing function which can figure out a mapping back to nodes/edges
#####

