#!/usr/bin/env python3

import numpy as np

class MapData:

    def __init__(self):

        self.aruco_marker_pose = {
        # Marker ID : [x, y, phi]
            "7" :   [0,     0.5,      -90],
        }

        self.global_target_positions = {
        # Plant position: [x, y]
        # For now, based on Alex's node positions
            "0" : [1.03, 0.76],
            "1" : [2.52, 4.58],
            "2" : [8.47, 5.49]
        }

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