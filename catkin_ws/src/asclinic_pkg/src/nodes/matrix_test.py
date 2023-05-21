import numpy as np 
import math
from map_data import MapData

# import rospy

# from std_msgs.msg import UInt32
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import radians

# Import the standard message types
# from geometry_msgs.msg import Pose, Point, Quaternion
marker_data = MapData()

# a = np.matrix([
#     [math.cos(phi), -math.sin(phi), 0],
#     [math.sin(phi), math.cos(phi),  5],
#     [0            , 0            ,  1]
# ])

# b = np.matrix([
#     [0],
#     [0],
#     [1]
# ])

# print(np.matmul(np.linalg.inv(a), b))


phi_c = np.pi

# Translation matrix from marker frame to camera frame
T_mc = np.matrix([
    [np.cos(phi_c), -np.sin(phi_c), 1],
    [np.sin(phi_c), np.cos(phi_c),  -0.5],
    [0            , 0            ,  1]
])

# Origin vector
origin = np.matrix([
    [0],
    [0],
    [1]
])

# Translation matrix from world frame to marker frame
marker_position = marker_data.translation_matrix(str(13))

# Invert Translation matrix from marker frame to camera frame
T_mc_inv = np.linalg.inv(T_mc)

# Calculate position vector via matrix multiplication
position_vector = np.matmul(marker_position, np.matmul(T_mc_inv, origin))

print(position_vector[0,0])
print(position_vector[1,0])
# c = np.matrix([
#     [math.cos(phi), -math.sin(phi)],
#     [math.sin(phi), math.cos(phi)]
# ])s

# d = np.matrix([
#     [0],
#     [5]
# ])

# print(np.matmul(c, d))

class ArucoData:

    def __init__(self):

        self.aruco_marker_pose = {
        # Marker ID : [x, y, phi]
            "7" :   [0,     0,      0],
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


test = ArucoData()

# print(test.translation_matrix(str(7)))
