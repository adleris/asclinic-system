#!/bin/sh

function prompt() {
    echo "$1"
    read -n 1 -s
}

echo "OBSTACLE AVOIDANCE TEST PROCEDURE"
echo " Press ok to go to the next step "
echo "---------------------------------"

prompt "Place the robot in an open space"

prompt "Start the obstacle avoidance launch script, and in a separate terminal run:

    rostopic echo /asc/path/obstacle_avoidance/detection"

# set robot's initial position to (x,y) = (1,1)
rostopic pub --once /asc/control/curr_pose geometry_msgs/Pose '{position:  {x: 1.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'

prompt "Trigger obstacle detection by waving something near the LIDAR sensor. Ensure that one and only one message is sent to the ROS topic"

# simulating the robot moving to (x,y) = (1.2, 0.75)
rostopic pub --once /asc/control/curr_pose geometry_msgs/Pose '{position:  {x: 1.2, y: 0.75, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'
prompt "Try again, ensure nothing is published"

# simulating the robot moving to (x,y) = (1.3, 0.5)
rostopic pub --once /asc/control/curr_pose geometry_msgs/Pose '{position:  {x: 1.3, y: 0.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'
prompt "Try again, this time, ensure another value is published"

echo "If all these tests passed, then verification is successful!"
