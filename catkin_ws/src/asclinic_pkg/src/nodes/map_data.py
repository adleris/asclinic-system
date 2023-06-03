#!/usr/bin/env python3

import numpy as np
from geometry_msgs.msg import Point
from asclinic_pkg.msg import PoseFloat32

class MapData:

    def __init__(self):

        self.aruco_marker_pose = {
        # Marker ID : [x, y, phi (in degrees)]
            "23" :  [5.35,  3.56,   -90],
            "28" :  [6.97,  2.09,   -135],
            "29" :  [9.98,  5.62,  -180],
            "26" :  [6.63,  5.25,   -90],
            "27"  :  [6.97,  2.09,   -135]
            # "13" :  [0.5,   9,      0],
            # "19" :  [6.82,  9.88,   -135]
        }

        self.global_target_positions =[
        # Plant position: [x, y, z]
        # z not used, should be set to zero
        # For now, based on Alex's node positions
            [9.15, 5.12],
            [4.58, 3.41]
        ]


    def get_point(self, index):
        point = Point(self.global_target_positions[index][0], self.global_target_positions[index][1],0)
        return point

    def get_target_pose(self, index):
        pose = PoseFloat32(self.aruco_marker_pose[index][0], self.aruco_marker_pose[index][1], self.aruco_marker_pose[index][2])
        return pose

    def translation_matrix(self, index):
        x = self.aruco_marker_pose[index][0]
        y = self.aruco_marker_pose[index][1]
        phi = np.radians(self.aruco_marker_pose[index][2])
        matrix = np.matrix([
            [np.cos(phi), -np.sin(phi), x],
            [np.sin(phi), np.cos(phi),  y],
            [0            , 0            ,  1]
        ])

        return matrix
