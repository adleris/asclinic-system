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
        self.f_system_drive = 0

        # Global Target
        self.map_data = MapData()
        self.global_target = self.map_data.global_target_positions["0"]

        # Setup Publishers
        self.pub_main_state      = rospy.Publisher("/asc"+"/main_state", String, queue_size=10)
        self.pub_global_target   = rospy.Publisher("/asc"+"/global_target", Point, queue_size=10)
        
        # Initialise Publishers
        self.pub_main_state.publish(self.s_main_state)
        # self.pub_global_target.publish(self.global_target)

        # Setup Subscribers
        rospy.Subscriber("/asc"+"/system_start", Bool, self.requestSystemStart)
        
        # Display the status
        rospy.loginfo("[MainNode] Initialisation complete")

        # Initialise a timer for capturing the camera frames
        rospy.Timer(rospy.Duration(1/self.main_node_freq), self.timerCallbackForMainFunction)

    # Respond to timer callback
    def requestSystemStart(self, event):
        rospy.loginfo("[MainNode] /asc/system_start received: " + str(event.data))
        # rospy.loginfo("[MainNode] Setting main_state to 'drive'")
        self.f_system_drive = event.data
        

    # Respond to timer callback
    def timerCallbackForMainFunction(self, event):

        if (self.f_system_drive == True):
            rospy.loginfo("[MainNode] Setting main_state to 'Idle'")
            self.s_main_state = "Idle"
        
        if (self.s_main_state == "Idle" and self.f_system_drive == True):
            rospy.loginfo("[MainNode] Setting main_state to 'Drive'")
            self.s_main_state = "Drive"
            self.pub_main_state.publish(self.s_main_state)
            self.f_system_drive = 0
        # rospy.loginfo("[MainNode] Callback function")


if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "main_node"
    rospy.init_node(node_name)
    amain_node_object = MainNode()
    # Spin as a single-threaded node
    rospy.spin()

