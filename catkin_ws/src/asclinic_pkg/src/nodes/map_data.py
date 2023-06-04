#!/usr/bin/env python3

import numpy as np
from geometry_msgs.msg import Point
from asclinic_pkg.msg import PoseFloat32

class MapData:

    def __init__(self):

        self.aruco_marker_pose = {
        # Marker ID : [x, y, phi (in degrees)]
            "2"  :  [5.46,  8.17,   -90], # DNL GROUP 2
            "4"  :  [6.04,  5.21,   45], # MAW 4
            "6"  :  [0.46,  0.45,   60],
            "7"  :  [3.84,  0.5,    -180],
            "8"  :  [0.8,   3.19,   45],
            "12" :  [2.82,  4.01,   90], # DNL GROUP 12
            "14" :  [8.49, 6.50,    -120], # DNL GROUP 14
            "15" :  [0.48, 5.08, 0],
            "18" :  [6.60,     9.92,   -90],
            "20" :  [8.1,   0.17,    -90], # DNL GROUP 20 but backwards
            # "22" :  [4.50, 4.98,    180], # DNL GROUP 22
            "23" :  [5.34,  3.57,   -90], # DNL GROUP 23
            # "28" :  [6.97,  2.09,   -135],
            "28" :  [9.98,  5.62,  -180], # DNL GROUP 28
            # "26" :  [6.63,  5.25,   -90],
            "24" :  [5.19, 6.09, 90],
            "27"  : [6.97,  2.09,   180] # MAW 27

            # "13" :  [0.5,   9,      0],
            # "19" :  [6.82,  9.88,   -135]
        }

        self.global_target_positions =[
        # Plant position: [x, y, z]
        # z not used, should be set to zero
        # For now, based on Alex's node positions
            [8.47, 4.51],
            [9.15, 5.12],
            [6.54, 7.97],
            [2.52, 5.42],
            [1.33, 2.37], 
            [1.05, 0.96],
            [4.55, 2.25]
        ]

        self.plant_camera_pose = [
        # Plant pose: [pan, tilt, phi]
            [0, 5, 15],
            [-35, 15, 25],
            [10, 0, 90],
            [15, 25, 175],
            [-10,  5,  -90],
            [-17,  15, -90],
            [0,  30, 0],
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
