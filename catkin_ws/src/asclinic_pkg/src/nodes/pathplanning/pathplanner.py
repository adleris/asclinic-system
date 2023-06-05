#!/usr/bin/env python3

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
#from visualise_path import PathVisualiser, ImageManager
from utilities import _mark_unused, point_distance

# send a global target at the start

NODE_NAME = "planner"

HARDCODED_START_POINT = Point(4.55, 2.25, 0)
HARDCODED_START_POSE  = Pose(HARDCODED_START_POINT, Quaternion(0,0,0,1))
AT_VERTEX_DISTANCE = 0.15 # metres

class PathPlanner():

    def __init__(self):
        # data stores for generic map data
        self.verts : list[Vertex]
        self.edges : list[Edge]
        self.graph : Graph
        self.init_graph()


        # hold information about the path
        self.path : list[tuple[float, float]] = []
        self.path_idx : int = 0


        # data stores for current state
        # for the first case of the problem, always start at a given location (node id 2 here)
        self.curr_pose : Pose = None
        self.global_target : Point = None    
        ##### TODO: should this be a vertex? # I think point is fine, global target can import the coordinate list
        self.global_target_id : int = -1


        # set up subscribers
        rospy.Subscriber("/planner/curr_pose", Pose, self.received_curr_pose)
        rospy.Subscriber("/main/global_target", Point, self.received_global_target)
        rospy.Subscriber("/obstacle_avoidance/detection", Bool, self.received_obstacle_detection)

        self.local_target_pub: rospy.Publisher = rospy.Publisher(NODE_NAME + "/next_target", Point, queue_size=10)
        self.at_global_target_pub: rospy.Publisher = rospy.Publisher(NODE_NAME + "/at_global_target", Bool, queue_size=1)

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

        rospy.loginfo(f"\033[33mCalculating path from {start} to {end}\033[0m")

        self.path = self.graph.dijkstra(start, end)
        #TODO possible handling here for no path found
        self.path_idx = 0

    def publish_next_local_target(self):
        """
        Publish the next target node down the line for traversal
        """
        self.path_idx += 1
        if self.path_idx >= len(self.path):
            rospy.loginfo("\033[32mReached the global target!\033[0m")
            self.at_global_target_pub.publish(True)
            return

        next_path_point = self.path[self.path_idx]
        next_point = Point(next_path_point[0], next_path_point[1], 0)
        self.local_target_pub.publish(next_point)

        rospy.loginfo(f"\033[33mNext local target: {self._vertex_id_from_tuple(next_path_point)} @ {next_path_point}\033[0m")

    def received_curr_pose(self, msg: Pose) -> None:
        """
        Update our knowledge of the current Pose

        :param msg: Pose
        """
        self.curr_pose = msg

        # rospy.loginfo(f"next pose received: ({msg.position.x}, {msg.position.y})")

        # check if we had arrived at the next local vertex and should 
        # publish the next segment of the path
        if self._check_if_at_target_node():
            rospy.loginfo("At target node!")
            self.publish_next_local_target()

    def received_global_target(self, msg: Point) -> None:
        """
        Update our global target. 
        
        Triggers a re-calculation of the path.

        :param msg: New target as Point

        If a vertex at those coordinates cannot be found, does nothing
        """
        self.global_target = msg

        rospy.loginfo("Received new global target ({}, {})".format(msg.x, msg.y))

        try:
            self.global_target_id : int = self._vertex_id_from_point(self.global_target, exact=True)
        except VertexNotFoundException:
            # couldn't find a vertex at those coordinates. Do nothing.
            rospy.logerr("Could not lookup global target!")
            return

        # if we haven't set a path yet, then we need to set up our first start location
        # TODO: This could be quite a complex calculation.
        # 1. The easy process is to always start at a known location
        # 2. The next process is to mark our current PP vertex as the nearest vertex. We will then generate trajectories based on thinking
        #    we were at that node. Controller will see some large error signals in this case, but it should work as a first pass.
        # 4. Drive around until we localise by a marker, then add a an edge to the nearest vertex (and assume we can navigate to it!)
        if len(self.path) == 0:
            current_vertex_id : int = self._vertex_id_from_point(self.curr_pose.position, exact=False)

        else:
            # use the last point of our path (= previous global target) as
            # current position to feed into dijkstra
            current_vertex_id : int = self._vertex_id_from_tuple(self.path[-1])

        rospy.logdebug("\033[33mabout to recalculate path from {} to {}\033[0m".format(current_vertex_id, self.global_target_id))
        self._recalculate_path(current_vertex_id, self.global_target_id)

        self.publish_next_local_target()

    def received_obstacle_detection(self, msg: Bool) -> None:
        """
        Run the 'detected obstacle' state transition.
        
        Triggers a re-calculation of the path.

        :param msg: Bool (always True)
        """
        # if we haven't initalised a path yet, do nothing
        if len(self.path) == 0:
            return

        # pull out important values from the path
        old_start : int = self.path[self.path_idx-1]
        old_end   : int = self.path[self.path_idx]
        self._remove_edge(old_start, old_end)

        # add a new vert at the current pose, and an edge from that vert back to the last vert
        new_vert : Vertex = Vertex(self.curr_pose.position.x, self.curr_pose.position.y)
        self.verts.append(new_vert)
        new_edge : Edge = Edge(new_vert.id, old_start, 1.0)
        self.edges.append(new_edge)

        # recalculate the full path from this new point
        self._recalculate_path(new_vert.id, self.global_target_id)

    def _vertex_id_from_point(self, point: Point, exact: bool = False) -> int:
        """
        Look up a vertex id in the vertex list based on its coordinates and return the ID.

        Point: The point to search for 
        Returns: ID of the vertex being looked up 

        Exact: True: Searches for a vertex exactly matching point (faster)
        Exact: False: Returns nearest vertex to point
        """
        if exact:
            for vertex in self.verts:
                # floating point equality is no good, but these should be generated form the same source
                if vertex.x == point.x and vertex.y == point.y:
                    return vertex.id

            rospy.logerr("Could not find vertex with coords:\n" + str(point))
            raise(VertexNotFoundException(point.x, point.y))
            return 0
        else:
            closest_id = -1
            min_dist = 10000000
            for vertex in self.verts:
                dist = point_distance(point, self._point_from_vertex_tuple(vertex.to_tuple()))
                if dist < min_dist:
                    closest_id = vertex.id
                    min_dist = dist
            return closest_id

    def _vertex_id_from_tuple(self, coord : Tuple[float, float], exact: bool = False) -> int:
        """
        Look up a vertex id in the vertex list based on tuple coordinates (as
        used in self.path) and return the ID.

        Returns ID of the vertex being looked up 
        """
        return self._vertex_id_from_point(self._point_from_vertex_tuple(coord), exact=exact)

    @staticmethod
    def _point_from_vertex_tuple(coord: tuple[float, float]) -> Point:
        """Convert an (x,y) from a path into a Point object"""
        return Point(coord[0], coord[1], 0)

    @_mark_unused
    def _point_from_vertex_id(self, id: int) -> Point:
        """
        Look up Point coordinates of a Vertex based on its `id`.

        :param point: 
        :return id: ID of the vertex being looked up 
        """
        for vertex in self.verts:
            if vertex.id == id:
                return Point(x=vertex.x, y=vertex.y, z=0)
        rospy.logerr("Could not find vertex with ID: " + str(id))
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
        if len(self.path) == 0:
            return False

        if self.path_idx == len(self.path):
            return

        target_position : Point = self._point_from_vertex_tuple(self.path[self.path_idx])
        return point_distance(self.curr_pose.position, target_position) < AT_VERTEX_DISTANCE


class VertexNotFoundException(Exception):
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=False)
    pp = PathPlanner()
    rospy.spin()


#####
# TODO Need to add a path-parsing function which can figure out a mapping back to nodes/edges
#####

