
from geometry_msg import Point
from asclinic_pkg.msg import PoseFloat32, LeftRightFloat32
from math import pi, atan2
import rospy

NODENAME    = "motion_controller"
NAMESPACE   = "asc/control"

STATE_IDE       = "IDEL"
STATE_ROTATE    = "ROTATE"
STATE_STRAIGHT  = "STRAIGHT"

class motion_controller():
    def __init__(self):
       
        
        self.rotationSpeed = 0.5
        self.straightLineSpeed = 3
        self.rotationTolarance = (5/180) * pi 
        self.locationTolarance = 0.05
        
        self.state = STATE_IDE
        self.IDEL_stateCounter = 0
        self.goal_pose = PoseFloat32(0, 0, 0) 


        self.RefPublisher = rospy.Publisher(f"{NAMESPACE}/wheel_speeds_reference", PoseFloat32, queue_size=1)
        rospy.Subscriber(f"{NAMESPACE}/sys_pose", PoseFloat32, self.control_main_loop, queue_size=1)
        rospy.Subscriber("planner/next_target", Point, self.add_to_location_queue, queue_size=1)
        
    def add_to_location_queue(self, event):
        self.IDEL_stateCounter = 0
        self.state = STATE_IDE
        refSignals = LeftRightFloat32(0,0)
        self.RefPublisher.publish(refSignals)
        self.stateQueue.append(STATE_ROTATE)
        
        self.goal_point = event
        self.calc_goal_pose()

    def calc_goal_pose(self):
        self.goal_pose.x = self.goal_point.x
        self.goal_pose.y = self.goal_point.y

        diffVector_x = self.goal_point.x - self.current_pose.x
        diffVector_y = self.goal_point.y - self.current_pose.y
        self.goal_pose.phi = atan2(diffVector_y, diffVector_x)

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
        if (self.state == STATE_IDE) and (self.IDEL_stateCounter >= 5):
            if self.stateQueue:
                self.state = self.stateQueue.pop()

        
        if (self.state == STATE_ROTATE) and self._rotationTransition():
            self.stateQueue.append(STATE_STRAIGHT)
            self.IDEL_stateCounter = 0
            self.state = STATE_IDE
                    
        
        
        # Outputs for the States:
        refSignals = LeftRightFloat32()
        if self.state == STATE_IDE:
            refSignals.left     = 0
            refSignals.right    = 0
            self.IDEL_stateCounter += 1
        
        elif self.state == STATE_ROTATE:
            refSignals.left     = self.rotationSpeed
            refSignals.right    = -self.rotationSpeed
        
        elif self.state == STATE_STRAIGHT:
            refSignals.left     = self.straightLineSpeed
            refSignals.right    = self.straightLineSpeed


        self.RefPublisher.publish(refSignals)







