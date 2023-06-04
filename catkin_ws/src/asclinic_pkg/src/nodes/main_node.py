#!/usr/bin/env python3
import rospy
import time

# Import the standard message types
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Pose, Point, Quaternion
from asclinic_pkg.msg import PoseFloat32

# Import numpy
import numpy as np

from map_data import MapData

# ---------------------------------------------------------------------------------
# PARAMETER DEFINITION

MAIN_NODE_FREQ  = 5 # Hz
INITIAL_NODE    = 0 # node index
INITIAL_POSE    = PoseFloat32(5.36, 2.59, np.pi/2)

# ---------------------------------------------------------------------------------

class MainNode:

    def __init__(self):

        # Frequency of main function
        self.main_node_freq = MAIN_NODE_FREQ

        # High level state machine
        # STATE OPTIONS:
        #   "Idle"
        #   "Driving"
        #   "Taking_Photo"
        self.s_main_state = "Idle"

        # Interrupt flags
        self.f_system_start     = False
        self.f_at_global_target = False
        self.f_photo_taken      = False
        self.f_emergency_idle   = False
        self.f_in_rotation      = False

        # Global Target
        self.map_data               = MapData()
        self.global_target_index    = INITIAL_NODE
        self.global_target          = self.map_data.get_point(self.global_target_index)

        # Setup Publishers
        self.pub_main_state          = rospy.Publisher("/asc"+"/main_state", String, queue_size=10)
        self.pub_global_target       = rospy.Publisher("/main"+"/global_target", Point, queue_size=10)
        self.pub_enable_drive        = rospy.Publisher("/asc"+"/enable_drive", Bool, queue_size=10)
        self.pub_enable_photo        = rospy.Publisher("/asc"+"/enable_photo", Bool, queue_size=10)
        self.pub_initial_pose        = rospy.Publisher("/asc"+"/initial_pose", PoseFloat32, queue_size=10)
        self.pub_pan                 = rospy.Publisher("/asc"+"/pan_deg", Int32, queue_size=10)
        self.pub_tilt                = rospy.Publisher("/asc"+"/tilt_deg", Int32, queue_size=10)
        self.pub_enable_camera_track = rospy.Publisher("/asc"+"/enable_camera_track", Bool, queue_size=10)
        self.pub_update_pose_phi     = rospy.Publisher("/asc"+"/update_phi", Int32, queue_size=10)
        
        time.sleep(5)
        self.pub_pan.publish(0)
        self.pub_tilt.publish(0)
        time.sleep(1)
        self.pub_pan.publish(0)
        self.pub_tilt.publish(0)
        self.pub_enable_camera_track.publish(False)
        self.pub_initial_pose.publish(INITIAL_POSE)
        
        # Initialise Publishers
        self.pub_main_state.publish(self.s_main_state)
        self.pub_enable_drive.publish(False)
        # self.pub_global_target.publish(self.global_target)

        # Setup Subscribers
        rospy.Subscriber("/asc"+"/system_start", Bool, self.callbackSystemStart)
        rospy.Subscriber("/planner"+"/at_global_target", Bool, self.callbackAtGlobalTarget)
        rospy.Subscriber("/asc"+"/photo_taken", Bool, self.callbackPhotoTaken)
        rospy.Subscriber("/asc"+"/emergency", Bool, self.callbackEmergency)
        rospy.Subscriber("/asc/control/in_rotation", Bool, self.set_inRotation, queue_size=1)
        
        # Display the status
        rospy.loginfo("[MainNode] Initialisation complete")

        # Initialise a timer for capturing the camera frames
        rospy.Timer(rospy.Duration(1/self.main_node_freq), self.timerCallbackForMainFunction)

    # Respond to /asc/system_start callback
    def callbackSystemStart(self, event):
        rospy.loginfo("[MainNode] /asc/system_start received: " + str(event.data))
        self.f_system_start = event.data
        
    def callbackAtGlobalTarget(self, event):
        rospy.loginfo("[MainNode] /planner/at_global_target received: " + str(event.data))
        self.f_at_global_target = event.data

    def callbackPhotoTaken(self, event):
        rospy.loginfo("[MainNode] /asc/f_photo_taken received: " + str(event.data))
        self.f_photo_taken = event.data

    def callbackEmergency(self, event):
        rospy.loginfo("[MainNode] /asc/f_emergency_idle received: " + str(event.data))
        self.f_emergency_idle = event.data

    def set_inRotation(self, event):
        rospy.loginfo("[MainNode] /asc/f_in_rotation received: " + str(event.data))
        self.inRotation = event.data

    # Respond to timer callback
    def timerCallbackForMainFunction(self, event):

        # If there is an emergency condition
        if (self.f_emergency_idle == True):
            rospy.loginfo("[MainNode] Setting main_state to 'Idle'")
            self.s_main_state       = "Idle"
            self.pub_main_state.publish(self.s_main_state)
            self.pub_enable_drive.publish(False)
            self.pub_enable_camera_track.publish(False)

            self.f_system_start     = False
            self.f_at_global_target = False
            self.f_photo_taken      = False
            self.f_emergency_idle   = False
        
        if (self.s_main_state == "Idle" and self.f_system_start == True):
            self.f_system_start = False
            self.pub_initial_pose.publish(INITIAL_POSE)
            self.pub_enable_camera_track.publish(False)
            self.transitionMainStateToDrive()
            
            

        if (self.s_main_state == "Drive" and self.f_at_global_target == True):
            self.f_at_global_target = False
            # Update Main State
            rospy.loginfo("[MainNode] Setting main_state to 'Taking_Photo'")
            self.s_main_state = "Taking_Photo"
            self.pub_main_state.publish(self.s_main_state)
            # Disable Driving
            self.pub_enable_drive.publish(False)
            # Disable Camera Tracking
            self.pub_enable_camera_track.publish(False)
            # Send camera pose update
            self.pub_update_pose_phi.publish(self.map_data.plant_camera_pose[self.global_target_index][2])
            time.sleep(0.1)


        if (self.s_main_state == "Taking_Photo" and self.f_in_rotation == False):
            # Send camera pose update
            # I'm sorry for bad indexing, I am tired :(
            rospy.loginfo("[MainNode] Sending position")
            for _ in range(3):
                time.sleep(1)
                self.pub_pan.publish(self.map_data.plant_camera_pose[self.global_target_index][0])
                self.pub_tilt.publish(self.map_data.plant_camera_pose[self.global_target_index][1])
            
            # Enable "Taking" a photo
            self.pub_enable_photo.publish(True)
               


        if (self.s_main_state == "Taking_Photo" and self.f_photo_taken == True):
            self.f_photo_taken = False
            # Publish new global target
            self.updateGlobalTarget()
            self.transitionMainStateToDrive()
            

    def transitionMainStateToDrive(self):
        # Update Main State
        rospy.loginfo("[MainNode] Setting main_state to 'Drive'")
        self.s_main_state = "Drive"
        self.pub_main_state.publish(self.s_main_state)
        # Publish the global target
        self.pub_global_target.publish(self.global_target)
        # Enable Driving
        self.pub_enable_drive.publish(True)
        # Disable "Taking" a photo
        self.pub_enable_photo.publish(False)
        # Enable camera tracking
        self.pub_enable_camera_track.publish(True)
        # Set tilt to 0
        self.pub_tilt.publish(0)
    
    def updateGlobalTarget(self):
        self.global_target_index += 1
        if self.global_target_index >= len(self.map_data.global_target_positions):
            self.global_target_index = 0
        self.global_target = self.map_data.get_point(self.global_target_index)


if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "main_node"
    rospy.init_node(node_name)
    amain_node_object = MainNode()
    # Spin as a single-threaded node
    rospy.spin()

