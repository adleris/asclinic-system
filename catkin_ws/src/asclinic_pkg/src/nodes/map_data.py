#!/usr/bin/env python3

import numpy as np
from geometry_msgs.msg import Point

class MapData:

    def __init__(self):

        self.aruco_marker_pose = {
        # Marker ID : [x, y, phi]
            "13" :   [0.5,     9,      0],
            "19" :   [6.82,    9.88,   -135]
        }

        self.global_target_positions =[
        # Plant position: [x, y, z]
        # z not used, should be set to zero
        # For now, based on Alex's node positions
            [2.06, 0.54],
            [6.35, 0.87],
            [8.47, 4.51]
        ]

    def get_point(self, index):
        point = Point(self.global_target_positions[index][0], self.global_target_positions[index][1],0)
        return point

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
