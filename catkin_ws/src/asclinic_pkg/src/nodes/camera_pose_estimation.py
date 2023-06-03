#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (C) 2021, The University of Melbourne, Department of Electrical and Electronic Engineering (EEE)
#
# This file is part of ASClinic-System.
#    
# See the root of the repository for license details.
#
# ----------------------------------------------------------------------------
#     _    ____   ____ _ _       _          ____            _                 
#    / \  / ___| / ___| (_)____ (_) ___    / ___| _   _ ___| |_ ___ ________  
#   / _ \ \___ \| |   | | |  _ \| |/ __|___\___ \| | | / __| __/ _ \  _   _ \ 
#  / ___ \ ___) | |___| | | | | | | (_|_____|__) | |_| \__ \ ||  __/ | | | | |
# /_/   \_\____/ \____|_|_|_| |_|_|\___|   |____/ \__, |___/\__\___|_| |_| |_|
#                                                 |___/                       
#
# DESCRIPTION:
# Python node to detect ArUco markers in the camera images
#
# ----------------------------------------------------------------------------

# ----------------------------------------------------------------------------
# A FEW USEFUL LINKS ABOUT ARUCO MARKER DETECTION
#
# > This link is most similar to the code used in this node:
#   https://aliyasineser.medium.com/aruco-marker-tracking-with-opencv-8cb844c26628
#
# > This online tutorial provdes very detailed explanations:
#   https://www.pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
#
# > This link is the main Aruco website:
#   https://www.uco.es/investiga/grupos/ava/node/26
# > And this is the documentation as linked on the Aruco website
#   https://docs.google.com/document/d/1QU9KoBtjSM2kF6ITOjQ76xqL7H0TEtXriJX5kwi9Kgc/edit
#
# > This link is an OpenCV tutorial for detection of ArUco markers:
#   https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html
#
# > This link is an OpenCV explanation of the "solvePnP" function:
#   https://docs.opencv.org/4.7.0/d5/d1f/calib3d_solvePnP.html
#
# > As starting point for details about Rodrigues representation of rotations
#   https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
#
# ----------------------------------------------------------------------------

# Import the ROS-Python package
import rospy
#import rospkg

# Import the standard message types
from std_msgs.msg import UInt32
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from asclinic_pkg.msg import PoseFloat32

# Import numpy
import numpy as np

# Import opencv
import cv2

# Import aruco
import cv2.aruco as aruco
from map_data import MapData

# Package to convert between ROS and OpenCV Images
from cv_bridge import CvBridge

# PARAMETER DEFINITION
USB_CAMERA_DEVICE_NUMBER = 0

# Camera properties
DESIRED_CAMERA_FRAME_HEIGHT = 1080
DESIRED_CAMERA_FRAME_WIDTH = 1920
DESIRED_CAMERA_FPS = 5

# Size of Aruco marker
MARKER_SIZE = 0.250

BLUR_THRESHOLD = 50

class ArucoDetector:

    def __init__(self):

        self.marker_data = MapData()

        self.pose_c_pub = rospy.Publisher("/asc"+"/pose_camera_estimation", PoseFloat32, queue_size=10)

        # Setup Subscriber to get camera pan pose
        rospy.Subscriber("/asc"+"/pose_camera_pan", Int32, self.adjust_pan_angle)
        self.phi_servo = 0

        # > Put the desired video capture properties into local variables
        self.camera_frame_width  = DESIRED_CAMERA_FRAME_WIDTH
        self.camera_frame_height = DESIRED_CAMERA_FRAME_HEIGHT
        self.camera_fps = DESIRED_CAMERA_FPS

        self.distancearray = []
        self.camera_setup = USB_CAMERA_DEVICE_NUMBER
        self.cam=cv2.VideoCapture(self.camera_setup)

        # Display the properties of the camera upon initialisation
        # > A list of all the properties available can be found here:
        #   https://docs.opencv.org/4.x/d4/d15/group__videoio__flags__base.html#gaeb8dd9c89c10a5c63c139bf7c4f5704d
        print("\n[ARUCO DETECTOR] Camera properties upon initialisation:")
        print("CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        print("CV_CAP_PROP_FRAME_WIDTH :  '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH)))
        print("CAP_PROP_FPS :             '{}'".format(self.cam.get(cv2.CAP_PROP_FPS)))
        print("CAP_PROP_FOCUS :           '{}'".format(self.cam.get(cv2.CAP_PROP_FOCUS)))
        print("CAP_PROP_AUTOFOCUS :       '{}'".format(self.cam.get(cv2.CAP_PROP_AUTOFOCUS)))
        print("CAP_PROP_BRIGHTNESS :      '{}'".format(self.cam.get(cv2.CAP_PROP_BRIGHTNESS)))
        print("CAP_PROP_CONTRAST :        '{}'".format(self.cam.get(cv2.CAP_PROP_CONTRAST)))
        print("CAP_PROP_SATURATION :      '{}'".format(self.cam.get(cv2.CAP_PROP_SATURATION)))
        #print("CAP_PROP_HUE :             '{}'".format(self.cam.get(cv2.CAP_PROP_HUE)))
        #print("CAP_PROP_CONVERT_RGB :     '{}'".format(self.cam.get(cv2.CAP_PROP_CONVERT_RGB)))
        #print("CAP_PROP_POS_MSEC :        '{}'".format(self.cam.get(cv2.CAP_PROP_POS_MSEC)))
        #print("CAP_PROP_FRAME_COUNT :     '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_COUNT)))
        print("CAP_PROP_BUFFERSIZE :      '{}'".format(self.cam.get(cv2.CAP_PROP_BUFFERSIZE)))

        # Set the camera properties to the desired values
        # > Frame height and  width, in [pixels]
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_frame_height)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH,  self.camera_frame_width)
        # > Frame rate, in [fps]
        self.cam.set(cv2.CAP_PROP_FPS, self.camera_fps)
        # > Auto focus, [bool: 0=off, 1=on]
        self.cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        # > Focus absolute, [int: min=0 max=250 step=5 default=0]
        #   0 corresponds to focus at infinity
        self.cam.set(cv2.CAP_PROP_FOCUS, 0)
        # > Buffer size, [int: min=1]
        #   Setting the buffer to zero ensures that we get that
        #   most recent frame even when the "timerCallbackForCameraRead"
        #   function takes longer than (1.self.camera_fps) seconds
        self.cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Display the properties of the camera after setting the desired values
        print("\n[ARUCO DETECTOR] Camera properties upon initialisation:")
        print("CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        print("CV_CAP_PROP_FRAME_WIDTH :  '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH)))
        print("CAP_PROP_FPS :             '{}'".format(self.cam.get(cv2.CAP_PROP_FPS)))
        print("CAP_PROP_FOCUS :           '{}'".format(self.cam.get(cv2.CAP_PROP_FOCUS)))
        print("CAP_PROP_AUTOFOCUS :       '{}'".format(self.cam.get(cv2.CAP_PROP_AUTOFOCUS)))
        print("CAP_PROP_BRIGHTNESS :      '{}'".format(self.cam.get(cv2.CAP_PROP_BRIGHTNESS)))
        print("CAP_PROP_CONTRAST :        '{}'".format(self.cam.get(cv2.CAP_PROP_CONTRAST)))
        print("CAP_PROP_SATURATION :      '{}'".format(self.cam.get(cv2.CAP_PROP_SATURATION)))
        #print("CAP_PROP_HUE :             '{}'".format(self.cam.get(cv2.CAP_PROP_HUE)))
        #print("CAP_PROP_CONVERT_RGB :     '{}'".format(self.cam.get(cv2.CAP_PROP_CONVERT_RGB)))
        #print("CAP_PROP_POS_MSEC :        '{}'".format(self.cam.get(cv2.CAP_PROP_POS_MSEC)))
        #print("CAP_PROP_FRAME_COUNT  :    '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_COUNT)))
        print("CAP_PROP_BUFFERSIZE :      '{}'".format(self.cam.get(cv2.CAP_PROP_BUFFERSIZE)))

        # The frame per second (fps) property cannot take any value,
        # hence compare the actural value and display any discrepancy
        camera_actual_fps = self.cam.get(cv2.CAP_PROP_FPS)
        if not(camera_actual_fps==self.camera_fps):
            rospy.logwarn("[ARUCO DETECTOR] The camera is running at " + str(camera_actual_fps) + " fps, even though " + str(self.camera_fps) + " fps was requested.")
            rospy.logwarn("[ARUCO DETECTOR] The fps discrepancy is normal behaviour as most cameras cannot run at arbitrary fps rates.")
            rospy.logwarn("[ARUCO DETECTOR] Due to the fps discrepancy, updated the value: self.camera_fps = " + str(camera_actual_fps))
            self.camera_fps = camera_actual_fps

        # Initlaise the OpenCV<->ROS bridge
        self.cv_bridge = CvBridge()

        # Get the ArUco dictionary to use
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

        # Create an parameter structure needed for the ArUco detection
        self.aruco_parameters = aruco.DetectorParameters()
        # > Specify the parameter for: corner refinement
        self.aruco_parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

        # Create an Aruco detector object
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_parameters)

        # Note: For OpenCV versions <=4.6, the above functions were:
        #self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        #self.aruco_parameters = aruco.DetectorParameters_create()
        #results = aruco.detectMarkers(current_frame_gray, self.aruco_dict, parameters=self.aruco_parameters)

        # Define the marker corner point in the marker frame
        marker_size_half = 0.5 * MARKER_SIZE
        self.single_marker_object_points = np.array([  \
                [-marker_size_half, marker_size_half, 0.0], \
                [ marker_size_half, marker_size_half, 0.0], \
                [ marker_size_half,-marker_size_half, 0.0], \
                [-marker_size_half,-marker_size_half, 0.0]  \
                ], dtype=np.float32 )

        # Specify the intrinsic parameters of the camera
        # > These parameters could either be hardcoded here;
        # > Or you can load then from a file that you may have
        #   saved during the calibration procedure.
        # > Note the that values hardcoded here may give
        #   meaningless results for your camera
        self.intrinic_camera_matrix = np.array( [[1405,0,966] , [0,1399,605] , [0,0,1]], dtype=float)
        self.intrinic_camera_distortion  = np.array( [[ 0.08885923, -0.19933783,  0.0082785,   0.00237787, -0.01476223]], dtype=float)

        # Read the a camera frame as a double check of the properties
        # > Read the frame
        return_flag , current_frame = self.cam.read()
        # > Get the dimensions of the frame
        dimensions = current_frame.shape
        # > Display the dimensions
        rospy.loginfo("[ARUCO DETECTOR] As a double check of the camera properties set, a frame captured just now has dimensions = " + str(dimensions))
        # > Also check the values
        if (not(dimensions[0]==self.camera_frame_height) or not(dimensions[1]==self.camera_frame_width)):
            rospy.logerr("[ARUCO DETECTOR] ERROR: frame dimensions do NOT match the desired values.")
            # Update the variables
            self.camera_frame_height = dimensions[0]
            self.camera_frame_width  = dimensions[1]

        # Display the status
        rospy.loginfo("[ARUCO DETECTOR] Initialisation complete")

        # Initialise a timer for capturing the camera frames
        rospy.Timer(rospy.Duration(1/self.camera_fps), self.timerCallbackForCameraRead)


    def adjust_pan_angle(self, data):
        pass

    # Respond to timer callback
    def timerCallbackForCameraRead(self, event):
        # Read the camera frame
        return_flag , current_frame = self.cam.read()

        # Check if the camera frame was successfully read
        if (return_flag == True):
            # Convert the image to gray scale
            current_frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
            # Detect ArUco markers from the frame
            aruco_corners_of_all_markers, aruco_ids, aruco_rejected_img_points = self.aruco_detector.detectMarkers(current_frame_gray)

            # Process any ArUco markers that were detected
            if aruco_ids is not None:

                current_frame_with_marker_outlines = aruco.drawDetectedMarkers(current_frame.copy(), aruco_corners_of_all_markers, aruco_ids, borderColor=(0, 220, 0))

                pose_info = None

                # Iterate over the markers detected
                for i_marker_id in range(len(aruco_ids)):
                    # Get the ID for this marker
                    this_id = aruco_ids[i_marker_id][0]
                    # Get the corners for this marker
                    corners_of_this_marker = np.asarray(aruco_corners_of_all_markers[i_marker_id][0], dtype=np.float32)
                    # Estimate the pose of this marker
                    solvepnp_method = cv2.SOLVEPNP_IPPE_SQUARE
                    success_flag, rvec, tvec = cv2.solvePnP(self.single_marker_object_points, corners_of_this_marker, self.intrinic_camera_matrix, self.intrinic_camera_distortion, flags=solvepnp_method)
                    
                    # Test to see if this is a marker we know about
                    if (str(this_id) in self.marker_data.aruco_marker_pose):
                        
                        if ((pose_info == None) or (tvec[2][0]**2 + tvec[0][0]**2 > pose_info["e_dist"])):
                            pose_info = {
                                "id":       str(this_id),
                                "phi_c":    (-np.linalg.norm(rvec)*np.sign(rvec)[1])[0],
                                "tvec_z":   tvec[2][0],
                                "tvec_x":   -tvec[0][0],
                                "e_dist":   tvec[2][0]**2 + tvec[0][0]**2
                            }
                
                if pose_info == None:
                    return

                if cv2.Laplacian(current_frame_gray, cv2.CV_64F).var() <= BLUR_THRESHOLD:
                    return

                # Translation matrix from marker frame to camera frame
                phi_c = pose_info["phi_c"]
                T_mc = np.matrix([
                    [np.cos(phi_c), -np.sin(phi_c), pose_info["tvec_z"]],
                    [np.sin(phi_c), np.cos(phi_c),  pose_info["tvec_x"]],
                    [0            , 0            ,  1]
                ])

                # Origin vector
                origin = np.matrix([
                    [0],
                    [0],
                    [1]
                ])

                # Translation matrix from world frame to marker frame
                marker_position = self.marker_data.translation_matrix(str(pose_info["id"]))

                # Invert Translation matrix from marker frame to camera frame
                T_mc_inv = np.linalg.inv(T_mc)

                # Calculate position vector via matrix multiplication
                position_vector = np.matmul(marker_position, np.matmul(T_mc_inv, origin))

                x_c = position_vector[0,0]
                y_c = position_vector[1,0]

                # Phi calculations
                phi_m = np.radians(self.marker_data.aruco_marker_pose[str(pose_info["id"])][2])
                # print(np.degrees(phi_c))
                # print(np.degrees(phi_m))
                
                phi = phi_m - phi_c # negative to make the math work out
                # print(np.degrees(phi))
                if (abs(phi) > np.pi):
                    phi = 2*np.pi - np.sign(phi)*phi
                # print("x_c: " + str(x_c) + " y_c: " + str(y_c) + " phi: " + str(np.degrees(phi)))
                data_string = PoseFloat32()
                data_string.x = x_c
                data_string.y = y_c
                data_string.phi = phi
                self.pose_c_pub.publish(data_string)

        else:
            # Display an error message
            rospy.loginfo("[ARUCO DETECTOR] ERROR occurred during \"self.cam.read()\"")


if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "aruco_detector"
    rospy.init_node(node_name)
    aruco_detector_object = ArucoDetector()
    # Spin as a single-threaded node
    rospy.spin()

    # Release the camera
    aruco_detector_object.cam.release()
    # Close any OpenCV windows
    cv2.destroyAllWindows()
