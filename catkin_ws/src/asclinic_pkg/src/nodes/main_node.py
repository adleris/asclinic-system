#!/usr/bin/env python3
import rospy

# Import the standard message types
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, Point, Quaternion

# Import numpy
import numpy as np

from map_data import MapData

# ---------------------------------------------------------------------------------
# PARAMETER DEFINITION

MAIN_NODE_FREQ = 5 # Hz

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

        # Global Target
        self.map_data = MapData()
        self.global_target = self.map_data.global_target_positions["0"]

        # Setup Publishers
        self.pub_main_state          = rospy.Publisher("/asc"+"/main_state", String, queue_size=10)
        self.pub_global_target       = rospy.Publisher("/asc"+"/global_target", Point, queue_size=10)
        self.pub_enable_drive        = rospy.Publisher("/asc"+"/enable_drive", Bool, queue_size=10)
        self.pub_enable_photo        = rospy.Publisher("/asc"+"/enable_photo", Bool, queue_size=10)
        
        # Initialise Publishers
        self.pub_main_state.publish(self.s_main_state)
        # self.pub_global_target.publish(self.global_target)

        # Setup Subscribers
        rospy.Subscriber("/asc"+"/system_start", Bool, self.callbackSystemStart)
        rospy.Subscriber("/asc"+"/at_global_target", Bool, self.callbackAtGlobalTarget)
        rospy.Subscriber("/asc"+"/photo_taken", Bool, self.callbackPhotoTaken)
        rospy.Subscriber("/asc"+"/emergency", Bool, self.callbackEmergency)
        
        # Display the status
        rospy.loginfo("[MainNode] Initialisation complete")

        # Initialise a timer for capturing the camera frames
        rospy.Timer(rospy.Duration(1/self.main_node_freq), self.timerCallbackForMainFunction)

    # Respond to /asc/system_start callback
    def callbackSystemStart(self, event):
        rospy.loginfo("[MainNode] /asc/system_start received: " + str(event.data))
        self.f_system_start = event.data
        
    def callbackAtGlobalTarget(self, event):
        rospy.loginfo("[MainNode] /asc/at_global_target received: " + str(event.data))
        self.f_at_global_target = event.data

    def callbackPhotoTaken(self, event):
        rospy.loginfo("[MainNode] /asc/f_photo_taken received: " + str(event.data))
        self.f_photo_taken = event.data

    def callbackEmergency(self, event):
        rospy.loginfo("[MainNode] /asc/f_emergency_idle received: " + str(event.data))
        self.f_emergency_idle = event.data

    # Respond to timer callback
    def timerCallbackForMainFunction(self, event):

        # If there is an emergency condition
        if (self.f_emergency_idle == True):
            rospy.loginfo("[MainNode] Setting main_state to 'Idle'")
            self.s_main_state       = "Idle"
            self.pub_main_state.publish(self.s_main_state)
            self.pub_enable_drive(False)

            self.f_system_start     = False
            self.f_at_global_target = False
            self.f_photo_taken      = False
            self.f_emergency_idle   = False
        
        if (self.s_main_state == "Idle" and self.f_system_start == True):
            self.f_system_start = False
            transitionMainStateToDrive()

        if (self.s_main_state == "Drive" and self.f_at_global_target == True):
            self.f_at_global_target = False
            # Update Main State
            rospy.loginfo("[MainNode] Setting main_state to 'Taking_Photo'")
            self.s_main_state = "Taking_Photo"
            self.pub_main_state.publish(self.s_main_state)
            # Disable Driving
            self.pub_enable_drive(False)
            # Enable "Taking" a photo
            self.pub_enable_photo(True)

        if (self.s_main_state == "Taking_Photo" and self.f_photo_taken == True):
            self.f_photo_taken = False
            transitionMainStateToDrive()
            


    def transitionMainStateToDrive():
        # Update Main State
        rospy.loginfo("[MainNode] Setting main_state to 'Drive'")
        self.s_main_state = "Drive"
        self.pub_main_state.publish(self.s_main_state)
        # Enable Driving
        self.pub_enable_drive(True)
        # Publish new global target
        #TODO Add funcationality to update global target position
        self.global_target = self.map_data.global_target_positions["0"]
        self.pub_global_target(self.global_target)
        # Disable "Taking" a photo
        self.pub_enable_photo(False)


if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "main_node"
    rospy.init_node(node_name)
    amain_node_object = MainNode()
    # Spin as a single-threaded node
    rospy.spin()

