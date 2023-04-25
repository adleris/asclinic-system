#!/usr/bin/env python

import rospy
from asclinic_pkg.msg import LeftRightFloat32, LeftRightInt32
from math import pi

ENCODER_TICKS_PER_ROTATION = 2240
SAMPLE_PERIOD = 0.1

class Controller:
    def __init__(self):
        self.PWM_publisher  = rospy.Publisher("set_motor_duty_cycle", LeftRightFloat32,  queue_size=1)
        self.K              = 4.5294
        self.alpha          = 0.5663
        self.prevError      = LeftRightFloat32()
        self.prevOutput     = LeftRightFloat32()
        self.refSpeed       = LeftRightFloat32()
        self.currError      = LeftRightFloat32()
    
        self.prevError.left     = 0.0
        self.prevError.right    = 0.0
        self.prevOutput.left    = 0.0
        self.prevOutput.right   = 0.0
        self.refSpeed.left      = 0.0
        self.refSpeed.right     = 0.0

        rospy.Subscriber("Wheel_speeds_refernce", LeftRightFloat32, self.setRefSpeed)
        rospy.Subscriber("encoder_counts", LeftRightInt32)

    def setRefSpeed(self, event):
        self.refSpeed.left      = event.left
        self.refSpeed.right     = event.right
        rospy.loginfo(f"New Refernce Speed for each wheels\n Left:{event.left} rad/s \t\tRight:{event.right} rad/s")

    def updatePWM_Signal(self, event):
        self.currError.left     = self.refSpeed.left - self._endcoderTicksToRadPerSec(event.left) 
        self.currError.right    = self.refSpeed.right - self._endcoderTicksToRadPerSec(event.right)

        currOutput = LeftRightFloat32()
        currOutput.left         = self.PID_controller(self.currError.left, self.prevError.left, self.prevOutput.left)
        currOutput.right        = self.PID_controller(self.currError.right, self.prevError.right, self.prevOutput.right)

        self.PWM_publisher.publish(currOutput)

        rospy.loginfo(f"Left Wheel:\nCurrent Error: {self.currError.left}\t\tCurrent Output:{currOutput.left}\n"
                      f"Right Wheel:\nCurrent Error: {self.currError.right}\t\tCurrent Output:{currOutput.right}\n"
        )

        self.prevError = self.currError
        self.prevOutput = currOutput


    def PID_controller(self, currError, prevError, prevOutput):
        output = prevOutput + self.K * (currError - self.alpha * prevError)
        if output > 100.0
            output = 100.0 
        elif output < -100.0
            output = -100.0
        
        return output


    def _endcoderTicksToRadPerSec(self, tick: int)
        return tick * 2 * pi / (ENCODER_TICKS_PER_ROTATION * SAMPLE_PERIOD)

      
if __name__ == "__main__":
    global node_name = "Wheel_Speed_controller"
    rospy.init_node(node_name)

    controller = Controller()
    rospy.spin() 

