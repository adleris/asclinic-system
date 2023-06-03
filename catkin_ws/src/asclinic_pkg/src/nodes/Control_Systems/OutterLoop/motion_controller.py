#!/usr/bin/env python

from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from asclinic_pkg.msg import PoseFloat32, LeftRightFloat32
from math import pi, atan2
from numpy import sign
import rospy

# TODO add a re orination at some point and give matt the ability to roate 180

NODE_NAME    = "motion_controller"
NAME_SPACE   = "control"

STATE_IDEL      = "IDEL"
STATE_ROTATE    = "ROTATE"
STATE_STRAIGHT  = "STRAIGHT"
STATE_STRAIGHT_WITH_P_CONTROL = "STRAIGHT with P control"
DRIVE_STATES = [STATE_STRAIGHT, STATE_STRAIGHT_WITH_P_CONTROL]
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
        self.posePhiTolorance = (30/180) * pi
        self.recalculateGoal = False
        
        # P control
        self.pControl_K = 5
        self.limitOfControl = 0.5

        # state set up
        self.enableDrive = False
        self.stateQueue = []
        self.state = STATE_IDEL
        self.stateCounter = 0
        
        self.goal_pose = PoseFloat32(0, 0, 0) 

        # ros setup
        self.RefPublisher = rospy.Publisher(f"{NAME_SPACE}/wheel_speeds_reference", LeftRightFloat32, queue_size=1)
        rospy.Subscriber(f"{NAME_SPACE}/curr_pose", PoseFloat32, self.control_main_loop, queue_size=1)
        rospy.Subscriber("/planner/next_target", Point, self.add_to_location_queue, queue_size=1)
        rospy.Subscriber("/asc/enable_drive", Bool, self.setEnableDrive, queue_size=1)
        
    def add_to_location_queue(self, event):
        rospy.loginfo(f"New Target Location x: {event.x}, y: {event.y}")
        self.stateCounter = 0
        self.state = STATE_IDEL
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

    def _rotationTransition(self, tolorance):
        # This has all logic for if the system should get out of the rotation state
        
        # This is the catch for if the goal rotation is on the range [pi, pi-tolerance]
        if (self.goal_pose.phi > 0) and (self.goal_pose.phi > pi - tolorance):
            otherSideTolerence = tolorance - (pi - self.goal_pose.phi)

            # catch for the cyclical nature 
            if (otherSideTolerence - pi < self.current_pose.phi):
                return True

        # This is the catch for if the goal rotation is on the range (-pi, -pi+tolerance]
        elif (self.goal_pose.phi < 0) and (self.goal_pose.phi < -pi + tolorance):
            otherSideTolerence = tolorance - (pi + self.goal_pose.phi)

            # catch for the cyclical nature
            if (pi - otherSideTolerence) < self.current_pose.phi:
                return True
        
        # This is the general catch for most cases
        if (abs(self.current_pose.phi - self.goal_pose.phi) <= tolorance) and (sign(self.current_pose.phi) == sign(self.goal_pose.phi)):
            return True
        
        # has yet to meet the conditions
        return False

    def _actionWhenInPControlState(self):
        if (self.current_pose.phi >= 0.5 * pi) and (self.goal_pose <= -0.5 * pi):
            phiError = (self.goal_pose.phi + 2 * pi) - self.current_pose.phi
        elif (self.current_pose.phi <= -0.5 * pi) and (self.goal_pose >= 0.5 * pi):
            phiError = self.goal_pose.phi - (self.current_pose.phi + 2 * pi)
        else:
            phiError = self.goal_pose.phi - self.current_pose.phi
        
        if phiError > 0:
            if (self.pControl_K * phiError > self.limitOfControl):
                leftWheelSpeed = self.straightLineSpeed - self.limitOfControl
            else:
                leftWheelSpeed  = self.straightLineSpeed - (self.pControl_K * phiError > self.limitOfControl)
            rightWheelSpeed = self.straightLineSpeed

        else:
            leftWheelSpeed  = self.straightLineSpeed 
            if (self.pControl_K * abs(phiError) > self.limitOfControl):
                rightWheelSpeed = self.straightLineSpeed - self.limitOfControl
            else:
                rightWheelSpeed = self.straightLineSpeed - self.pControl_K * abs(phiError)
        
        return leftWheelSpeed, rightWheelSpeed

    def control_main_loop(self, event):
        self.current_pose = event
        
        # Transitions for States:
        if (self.state == STATE_IDEL) and (self.stateCounter >= 2) and self.recalculateGoal:
            self.calc_goal_pose()
            self.recalculateGoal = False
            rospy.loginfo(f"[{NODE_NAME}] Current State: {self.state}")

        if (self.state == STATE_IDEL) and (self.stateCounter >= 5):
            self.stateCounter = 0
            if len(self.stateQueue) >= 1:
                self.state = self.stateQueue.pop(0)
                rospy.loginfo(f"[{NODE_NAME}] Current State: {self.state}")
        
        if (self.state == STATE_ROTATE) and self._rotationTransition(self.rotationTolarance):
            self.stateCounter = 0
            self.state = STATE_IDEL
            rospy.loginfo(f"[{NODE_NAME}] Current State: {self.state}")
        
        if (self.state ==  DRIVE_STATES) and (self.stateCounter >= 80):
            self.state = STATE_STRAIGHT_WITH_P_CONTROL
            rospy.loginfo(f"[{NODE_NAME}] Current State: {self.state}")
        
        if (self.state in DRIVE_STATES) and (self.stateCounter % 100 == 0):
            self.calc_goal_pose()

        if (self.state in DRIVE_STATES) and self._rotationTransition(self.posePhiTolorance):
            self.stateQueue = [STATE_ROTATE, STATE_STRAIGHT]
            self.stateCounter = 0
            self.state = STATE_IDEL
            self.recalculateGoal = True
            rospy.loginfo(f"[{NODE_NAME}] Current State: {self.state}")
        
        self.stateCounter += 1 
        
        # Outputs for the States:
        refSignals = LeftRightFloat32()
        
        if self.state == STATE_ROTATE:
            refSignals.left     =   -self.rotateMultiplier * self.rotationSpeed
            refSignals.right    =    self.rotateMultiplier * self.rotationSpeed
        
        elif self.state == STATE_STRAIGHT and self.enableDrive:
            #? Maybe add a ramp function 
            #? also think about breaking halfway
            refSignals.left     = self.straightLineSpeed
            refSignals.right    = self.straightLineSpeed
        
        elif self.state == STATE_STRAIGHT and self.enableDrive:
            refSignals.left, refSignals.right = self._actionWhenInPControlState()
        
        else:
            # this is deafult for seafty 
            refSignals.left     = 0
            refSignals.right    = 0

            

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
