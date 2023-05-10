import numpy as np 
import math

import rospy

from std_msgs.msg import UInt32
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import radians

# Import the standard message types
from geometry_msgs.msg import Pose, Point, Quaternion

phi = -(math.asin(4/5)+math.pi/2)
print(math.degrees(phi))

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

a = np.matrix([
    [math.cos(phi), -math.sin(phi), 0],
    [math.sin(phi), math.cos(phi),  5],
    [0            , 0            ,  1]
])


b = np.matrix([
    [0],
    [0],
    [1]
])

c = np.matrix([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
])

temp = np.matmul(c, np.linalg.inv(a))
print(temp)
print(np.matmul(temp, b))

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
            "7" :   [0.5,     1,      90],
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

print(test.translation_matrix(str(7)))
