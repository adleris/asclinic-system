#!/usr/bin/env python

import rospy
from asclinic_pkg.msg import LeftRightFloat32, LeftRightInt32
from math import pi

NAMESPACE = "asc/control"
NODE_NAME = f"{NAMESPACE}/wheel_angular_speed_controller"

ENCODER_TICKS_PER_ROTATION = 2240
SAMPLE_PERIOD = 0.05
LEFT_WHEEL = 0
RIGHT_WHEEL = 1

class wheelAngularSpeedController:
    def __init__(self):
        # coeficients of controller, index is equal to the degree the coefficent is to the power of 
        self.num          = [-2.5649922,  4.5294, 0]
        self.den          = [-1, 1, 0]

        # arrays for storing info need for controller 
        self.error                  = [[0]*2] * len(self.num)
        self.output                 = [[0]*2] * len(self.den)
        self.refSpeed               = [0] * 2
        self.currWheelAngularSpeed  = [0] * 2
        self.leftWheelMultiplier    = 1
        self.rightWheelMultiplier   = 1


        # ROS, set up publishing and subscribing
        self.PWM_publisher          = rospy.Publisher("set_motor_duty_cycle", LeftRightFloat32,  queue_size=1)
        self.wheelSpeed_publisher   = rospy.Publisher(f"{NAMESPACE}/wheel_angular_speeds", LeftRightFloat32,  queue_size=1)
        rospy.Subscriber(f"{NAMESPACE}/wheel_speeds_reference", LeftRightFloat32, self.setRefSpeed, queue_size=1) 
        rospy.Subscriber("encoder_counts", LeftRightInt32, self.newEncoderReading, queue_size=1)

    # Takes a new reference speed set and will process for the information 
    def setRefSpeed(self, event):
        self.refSpeed[LEFT_WHEEL]      = event.left
        self.refSpeed[RIGHT_WHEEL]     = event.right
        
        # This will give the wheel speed its value its direction for each wheel
        if self.refSpeed[LEFT_WHEEL] > 0:
            self.leftWheelMultiplier = 1
        elif self.refSpeed[LEFT_WHEEL] < 0:
            self.leftWheelMultiplier = -1

        if self.refSpeed[RIGHT_WHEEL] > 0:
            self.rightWheelMultiplier = 1
        elif self.refSpeed[RIGHT_WHEEL] < 0:
            self.rightWheelMultiplier = -1

        # rospy.loginfo(f"New Reference Speed for each wheels\n Left:{event.left} rad/s \t\tRight:{event.right} rad/s")

    def newEncoderReading(self, event):
        # Gets the current wheel speed and will publish it
        wheelSpeedToPub = LeftRightFloat32()
        self.currWheelAngularSpeed[LEFT_WHEEL]   = self.leftWheelMultiplier  * self._endcoderTicksToRadPerSec(event.left)
        self.currWheelAngularSpeed[RIGHT_WHEEL]  = self.rightWheelMultiplier * self._endcoderTicksToRadPerSec(event.right)
        wheelSpeedToPub.left = self.currWheelAngularSpeed[LEFT_WHEEL]
        wheelSpeedToPub.right = self.currWheelAngularSpeed[RIGHT_WHEEL]
        self.wheelSpeed_publisher.publish(wheelSpeedToPub)

        # this will make sure the output is nothing if the reference is nothing. Else does controller equation
        for wheel in [LEFT_WHEEL, RIGHT_WHEEL]:
            # calculating the current error
            self.error[0][wheel] = self.refSpeed[wheel] - self.currWheelAngularSpeed[wheel]
            
            if self.refSpeed[wheel] == 0.0:
                self.output[0][wheel] = 0.0
            else:
                self.output[0][wheel] = self.SecndOrderTimeSeriesEquation(self.output[1][wheel],  self.output[2][wheel],  self.error[0][wheel],  
                                                                self.error[1][wheel],   self.error[2][wheel])
        
        # As the direction defined as  forward, is in front of the robot, this was found to require a negative PWM signal to be sent, 
        # thus the output had to be made a negative value 
        currOutput = LeftRightFloat32()
        currOutput.left  = -self.output[0][LEFT_WHEEL]
        currOutput.right = -self.output[0][RIGHT_WHEEL]    

        self.PWM_publisher.publish(currOutput)

        rospy.loginfo(f"\nLeft Wheel:\nCurrent Error: {self.error[0][LEFT_WHEEL]}\t\tCurrent Output:{currOutput.left}\n"
                       f"Right Wheel:\nCurrent Error: {self.error[0][RIGHT_WHEEL]}\t\tCurrent Output:{currOutput.right}\n"
        )

        # translating error down a time step, for the next time
        self.error[1:3] = self.error[0:2]
        self.output[1:3] = self.output[0:2]

    # controller time equation cal and saturation catch
    def SecndOrderTimeSeriesEquation(self, output_1, output_2, error_0, error_1, error_2):
        # This is the input-output time series equation that been dirved in the report. 
        # This could be generalised better, however would be better to numpy arrays inplace of lists for error and output
        output = (1/self.den[0]) * (self.num[2] * error_2 + self.num[1] * error_1 + self.num[0] * error_0 - self.den[2] * output_2 - self.den[1] * output_1)
        
        # Catch for saturation
        if output > 100.0:
            output = 100.0
        elif output < -100.0:
            output = -100.0
        
        # Sign multiplier may not need to go here, instead just put a negative here 
        return  output

    # converts the encoder counts to rad speed 
    def _endcoderTicksToRadPerSec(self, tick: int):
        return 2 * pi * (tick / ENCODER_TICKS_PER_ROTATION) / SAMPLE_PERIOD

      
if __name__ == "__main__":
    global NODE_NAME
    
    rospy.init_node(NODE_NAME)
    controller = wheelAngularSpeedController()
    rospy.spin() 