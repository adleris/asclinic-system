#!/usr/bin/env python

from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from asclinic_pkg.msg import PoseFloat32, LeftRightFloat32
from math import pi, atan2
import rospy

# TODO add a re orination at some point 

NODE_NAME    = "motion_controller"
NAME_SPACE   = "control"

STATE_IDLE      = "IDLE"
STATE_ROTATE    = "ROTATE"
STATE_STRAIGHT  = "STRAIGHT"
ROTATE_LEFT     = 1
ROTATE_RIGHT    = -1

class motion_controller():
    def __init__(self):
        # will be postive when theta need to increase (i.e turn left) and negative when need to decrease (i.e. turn right)
        self.rotateMultiplier = 1
        
        # constants needed for movment 
        self.rotationSpeed = 0.5
        self.straightLineSpeed = 3
        self.rotationTolarance = (2/180) * pi 
        self.locationTolarance = 0.05 # isnt in use
        
        # state set up
        self.enableDrive = False
        self.stateQueue = []
        self.state = STATE_IDLE
        self.IDEL_stateCounter = 0
        
        self.goal_pose = PoseFloat32(0, 0, 0) 

        # ros setup
        self.RefPublisher = rospy.Publisher(f"{NAME_SPACE}/wheel_speeds_reference", LeftRightFloat32, queue_size=1)
        rospy.Subscriber(f"{NAME_SPACE}/curr_pose", PoseFloat32, self.control_main_loop, queue_size=1)
        rospy.Subscriber("/planner/next_target", Point, self.add_to_location_queue, queue_size=1)
        rospy.Subscriber("/asc/enable_drive", Bool, self.setEnableDrive, queue_size=1)
        
    def add_to_location_queue(self, event):
        rospy.loginfo(f"New Target Location x: {event.x}, y: {event.y}")
        self.IDEL_stateCounter = 0
        self.state = STATE_IDLE
        refSignals = LeftRightFloat32(0,0)
        self.RefPublisher.publish(refSignals)

        # rotate and then move, queue need to be reset as well 
        self.stateQueue = [STATE_ROTATE, STATE_STRAIGHT]
        
        self.goal_point = event
        self.calc_goal_pose()

    def calc_goal_pose(self):
        self.goal_pose.x = self.goal_point.x
        self.goal_pose.y = self.goal_point.y

        # calculating the difference vectors for to calcuate the facing to get their
        diffVector_x = self.goal_point.x - self.current_pose.x
        diffVector_y = self.goal_point.y - self.current_pose.y
        self.goal_pose.phi = atan2(diffVector_y, diffVector_x)

        # This is used to calculate which way to rotate
        # this is for phi in [0, pi]

        if self.current_pose.phi >= 0.0:
            # Catch for [0, pi] as current angle
            if (self.current_pose.phi <= self.goal_pose.phi) or (self.current_pose.phi - pi >= self.goal_pose.phi):
                self.rotateMultiplier = ROTATE_LEFT
            else:
                self.rotateMultiplier = ROTATE_RIGHT
        else:
            # Catch for (-pi, 0] as current angle
            if (self.current_pose.phi >= self.goal_pose.phi) or (self.current_pose.phi + pi <= self.goal_pose.phi):
                self.rotateMultiplier = ROTATE_RIGHT
            else:
                self.rotateMultiplier = ROTATE_LEFT

    def setEnableDrive(self, event):
        rospy.loginfo("[motion_controller] /asc/enable_drive received: " + str(event.data))
        self.enableDrive = event.data

    def _rotationTransition(self):
        # This has all logic for if the system should get out of the rotation state
        
        # This is the catch for if the goal rotation is on the range [pi, pi-tolerance]
        if (self.goal_pose.phi > 0) and (self.goal_pose.phi > pi - self.rotationTolarance):
            otherSideTolerence = self.rotationTolarance - (pi - self.goal_pose.phi)

            # catch for the cyclical nature 
            if (otherSideTolerence - pi < self.current_pose.phi):
                return True

        # This is the catch for if the goal rotation is on the range (-pi, -pi+tolerance]
        elif (self.goal_pose.phi < 0) and (self.goal_pose.phi < -pi + self.rotationTolarance):
            otherSideTolerence = self.rotationTolarance - (pi + self.goal_pose.phi)

            # catch for the cyclical nature
            if (pi - otherSideTolerence) < self.current_pose.phi:
                return True
        
        # This is the general catch for most cases
        if (abs(self.current_pose.phi - self.goal_pose.phi) <= self.rotationTolarance):
            return True
        
        # has yet to meet the conditions
        return False


    def control_main_loop(self, event):
        self.current_pose = event
        
        # Transitions for States:
        if (self.state == STATE_IDLE) and (self.IDEL_stateCounter >= 5):
            if len(self.stateQueue) >= 1:
                self.state = self.stateQueue.pop(0)
        
        if (self.state == STATE_ROTATE) and self._rotationTransition():
            rospy.loginfo(f"Current State: {self.state}")
            self.IDEL_stateCounter = 0
            self.state = STATE_IDLE
        
        # This could just output zero i.e. needs to be true to output a drive signal
        if not self.enableDrive:
            self.state = STATE_IDLE
                    
        # Outputs for the States:
        refSignals = LeftRightFloat32()
        
        if self.state == STATE_ROTATE:
            refSignals.left     =   -self.rotateMultiplier * self.rotationSpeed
            refSignals.right    =    self.rotateMultiplier * self.rotationSpeed
        
        elif self.state == STATE_STRAIGHT:
            #? Maybe add a ramp function 
            #? also think about breaking halfway 
            refSignals.left     = self.straightLineSpeed
            refSignals.right    = self.straightLineSpeed
        else:
            # this is deafult for seafty 
            refSignals.left     = 0
            refSignals.right    = 0

            # only incirments if in IDLE state
            if self.state == STATE_IDLE:
                self.IDEL_stateCounter += 1

        self.RefPublisher.publish(refSignals)

if __name__ == "__main__":
    try:
        rospy.init_node(NODE_NAME)
        controller = motion_controller()
        rospy.spin()
    except KeyboardInterrupt:
        refSignals = LeftRightFloat32(0, 0)
        controller.RefPublisher.publish(refSignals)
        exit()
