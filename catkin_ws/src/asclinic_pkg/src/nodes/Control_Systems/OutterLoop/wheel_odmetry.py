import rospy
from asclinic_pkg.msg import PoseFloat32
from math import cos, sin 

NAMESPACE = "asc/control"
NODE_NAME = f"{NAMESPACE}/wheel_odometry"

# constants:
WHEEL_BASE = 3
WHEEL_RADIUS = 4

def convert_wheel_speeds_to_pose_differences(event):
    global change_pos_publisher

    # needed for further calculations
    delta_s     = WHEEL_RADIUS * (event.left + event.right) / 2
    delta_phi   = WHEEL_RADIUS * (event.left - event.right) / WHEEL_BASE

    # publising the change in pose to be published
    changeToPose = PoseFloat32()
    # add the phi of the previous phi
    changeToPose.x   = delta_s * cos(0.5 * delta_phi)
    changeToPose.y   = delta_s * sin(0.5 * delta_phi)
    changeToPose.phi = delta_phi

    change_pos_publisher.publish(changeToPose)

if __name__ == "__main__":
    global NODE_NAME, change_pos_publisher

    # setup of node
    rospy.init_node(NODE_NAME)
    change_pos_publisher = rospy.Publisher(f"{NAMESPACE}/change_to_pose", PoseFloat32,  queue_size=1)
    rospy.Subscriber(f"{NAMESPACE}/wheel_angular_speeds", LeftRightFloat32, convert_wheel_speeds_to_pose_differences, queue_size=1)
    rospy.spin()