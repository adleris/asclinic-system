#!/usr/bin/env python3

from asclinic_pkg.msg import PoseFloat32
from std_msgs.msg import Int32, Bool
from math import pi
import numpy as np
import rospy

from map_data import MapData

class ServoPositionControl:
    def __init__(self):

        self.pan_angle      = 0
        self.tilt_angle     = 0
        self.map_data       = MapData()
        self.enable_tracking = False
        self.inRotation = False

        rospy.Subscriber("/asc/control/curr_pose", PoseFloat32, self.calc_camera_position, queue_size=1)
        rospy.Subscriber("/asc/enable_camera_track", Bool, self.set_tracking_enable, queue_size=1)
        rospy.Subscriber("/asc/control/in_rotation", Bool, self.set_inRotation, queue_size=1)
        
        self.pub_pan = rospy.Publisher("/asc"+"/pan_deg", Int32, queue_size=10)
        self.pub_tilt = rospy.Publisher("/asc"+"/tilt_deg", Int32, queue_size=10)

    def set_tracking_enable(self, event):
        print(f"enable_tracking {event.data}")
        self.enable_tracking = event.data
    
    def set_inRotation(self, event):
        self.inRotation = event.data

    def calc_camera_position(self, event):

        if self.enable_tracking == False:
            return

        curr_pose = event
        # Find closest marker
        closest_distance = 10000000
        closest_index    = None
        closest_pan      = None
        for marker_index in self.map_data.aruco_marker_pose:
            marker_pose = self.map_data.get_target_pose(marker_index)
            # Calculate distance between curr_pose and marker if marker location is valid
            pan_angle = self.delta_angle(curr_pose, marker_pose)
            if pan_angle == None:
                continue
            euclid_dist = np.sqrt((curr_pose.x - marker_pose.x)**2 + (curr_pose.y - marker_pose.y)**2)

            if euclid_dist < closest_distance:
                closest_distance    = euclid_dist
                closest_index       = marker_index
                closest_pan         = pan_angle
        # No valid marker found
        if closest_pan == None:
            self.pub_pan.publish(Int32(0))
            return
        
        # 3 stage moving average to avoid spikes
        closest_pan_d1 = closest_pan
        closest_pan_d2 = closest_pan_d1
        average_closest_pan = (closest_pan + closest_pan_d1 + closest_pan_d2)/3
        rounded_pan = round(average_closest_pan/10)*10

        self.pub_pan.publish(Int32(int(rounded_pan)))


    # Determine whether the camera can rotate to a given target and see an aruco marker
    def delta_angle(self, curr_pose, target_pose):
        curr_phi = np.degrees(curr_pose.phi)
        # Calculate the pan servo angle required to see an aruco marker
        delta_x =  target_pose.x - curr_pose.x
        delta_y =  target_pose.y - curr_pose.y
        theta = np.arctan2(delta_y,delta_x)
        # Phi camera -> marker
        phi_cm = -(curr_phi - np.degrees(theta))

        if phi_cm <= -180:
            phi_cm += 360
        elif phi_cm > 180:
            phi_cm -= 360

        # Angles greater than 90deg cannot be achieved by the pan servo so stop
        # Angle increated to 110deg due to POV angle of camera
        # Reject marker if the angle is too much
        if abs(phi_cm) > 110:
            return None

        # Limit phi_cm to [-90, 90]
        if phi_cm > 90:
            phi_cm = 90
        if phi_cm < -90:
            phi_cm = -90

        # Repeat the same with the deltas switched to determine whether
        # the aruco marker image can be seen at the target pose
        delta_x =  curr_pose.x - target_pose.x
        delta_y =  curr_pose.y - target_pose.y
        theta = np.arctan2(delta_y,delta_x)
        # Phi marker -> camera
        phi_mc = -(target_pose.phi - np.degrees(theta))

        if phi_mc <= -180:
            phi_mc += 360
        elif phi_mc > 180:
            phi_mc -= 360

        if abs(phi_mc) > 45:
            return None
        
        # Condition met when the servo can pan to a given marker and it can be viewed
        return phi_cm

if __name__ == "__main__":

    rospy.init_node("servo_position_control")
    position_control = ServoPositionControl()
    rospy.spin()

