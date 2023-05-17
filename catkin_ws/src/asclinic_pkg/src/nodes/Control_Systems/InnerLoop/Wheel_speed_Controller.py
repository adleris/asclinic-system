#!/usr/bin/env python

import rospy
from asclinic_pkg.msg import LeftRightFloat32, LeftRightInt32
from math import pi

ENCODER_TICKS_PER_ROTATION = 2240
SAMPLE_PERIOD = 0.05
LEFT_WHEEL = 0
RIGHT_WHEEL = 1

class innerLoopController:
    def __init__(self):
        # coeficients of controller, index is equal to the degree the coefficent is to the power of 
        self.num          = [-2.5649922,  4.5294, 0] # [0, 7.81, -7.321] [5, 0, 0]
        self.den          = [-1, 1, 0]# [1, -1.021, 0.02136] [1, 0, 0]

        # arrays for storing info need for controller 
        self.error      = [[0]*2] * len(self.num)
        self.output     = [[0]*2] * len(self.den)
        self.refSpeed       = [0] *2
        self.currWheelSpeed = [0] *2
        self.leftWheelMultiplier = 1
        self.rightWheelMultiplier = 1


        # ROS, set up publishing and subscribing
        self.PWM_publisher          = rospy.Publisher("set_motor_duty_cycle", LeftRightFloat32,  queue_size=1)
        self.wheelSpeed_publisher   = rospy.Publisher("wheel_Speeds", LeftRightFloat32,  queue_size=1)
        rospy.Subscriber("Wheel_speeds_refernce", LeftRightFloat32, self.setRefSpeed, queue_size=1) #e
        rospy.Subscriber("encoder_counts", LeftRightInt32, self.newEncoderReading, queue_size=1)

    # Takes a new reference speed set and will process for the information 
    def setRefSpeed(self, event):
        self.refSpeed[LEFT_WHEEL]      = event.left
        self.refSpeed[RIGHT_WHEEL]     = event.right
        
        # To go forwards will need a negative PWM signal, this takes that into cosideration
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
        self.currWheelSpeed[LEFT_WHEEL]   = self.leftWheelMultiplier  * self._endcoderTicksToRadPerSec(event.left)
        self.currWheelSpeed[RIGHT_WHEEL]  = self.rightWheelMultiplier * self._endcoderTicksToRadPerSec(event.right)
        wheelSpeedToPub.left = self.currWheelSpeed[LEFT_WHEEL]
        wheelSpeedToPub.right = self.currWheelSpeed[RIGHT_WHEEL]
        self.wheelSpeed_publisher.publish(wheelSpeedToPub)

        # calculating the current error
        self.error[0][LEFT_WHEEL]       = self.refSpeed[LEFT_WHEEL]   - self.currWheelSpeed[LEFT_WHEEL]
        self.error[0][RIGHT_WHEEL]      = self.refSpeed[RIGHT_WHEEL]  - self.currWheelSpeed[RIGHT_WHEEL] 

        currOutput = LeftRightFloat32()
        # this will make sure the output is nothing if the reference is nothing. Else does controller equation
        for wheel in [LEFT_WHEEL, RIGHT_WHEEL]:
            if self.refSpeed[wheel] == 0.0:
                self.output[0][wheel] = 0.0
            else:
                self.output[0][wheel] = self.SecndOrderTimeSeriesEquation(self.output[1][wheel],  self.output[2][wheel],  self.error[0][wheel],  
                                                                self.error[1][wheel],   self.error[2][wheel])
        # TODO: explain why negative
        currOutput.left  = -self.output[0][LEFT_WHEEL]
        currOutput.right = -self.output[0][RIGHT_WHEEL]    

        self.PWM_publisher.publish(currOutput)

        # rospy.loginfo(f"\nLeft Wheel:\nCurrent Error: {self.currError.left}\t\tCurrent Output:{currOutput.left}\n"
        #               f"Right Wheel:\nCurrent Error: {self.currError.right}\t\tCurrent Output:{currOutput.right}\n"
        # )

        # translating error down a time step, for the next time
        self.error[1:3] = self.error[0:2]
        # ! nathan does the flip of signs here again <----
        self.output[1:3] = self.output[0:2]


    # controller time equation cal and saturation catch
    def SecndOrderTimeSeriesEquation(self, output_1, output_2, error_0, error_1, error_2):
        #TODO document the inputs and outputs
        output = (1/self.den[0]) * (self.num[2] * error_2 + self.num[1] * error_1 + self.num[0] * error_0 - self.den[2] * output_2 - self.den[1] * output_1)
        
        # Catch for saturation
        if output > 100.0:
            output = 100.0
        elif output < -100.0:
            output = -100.0
        
        # ! may have to return it as a negative value ask nathan as to why this trickry is done lol <----
        # Sign multiplier may not need to go here, instead just put a negative here 
        return  output

    # converts the encoder counts to rad speed 
    def _endcoderTicksToRadPerSec(self, tick: int):
        return 2 * pi * (tick / ENCODER_TICKS_PER_ROTATION) / SAMPLE_PERIOD

      
if __name__ == "__main__":
    global node_name 
    node_name = "controlInner"
    rospy.init_node(node_name)

    controller = innerLoopController()
    rospy.spin() 