#!/bin/bash

echo "In a new terminal, run:

    python3 pathplanner.py
"

read -n 1 -s

echo "Publishing some initial messages: "

# initial pose as node id=3
rostopic pub --once /planner/curr_pose geometry_msgs/Pose '{position:  {x: 5.49, y: 0.54, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'

# target pose at node id=2
rostopic pub --once /main/global_target geometry_msgs/Point '{x: 2.41, y: 0.52, z: 0.0}'

# start moving towards id=2
echo "local target should now be (3.81, 0.73).
Ready to move towards local target?"
# read -n 1 -s

rostopic pub --once /planner/curr_pose geometry_msgs/Pose '{position:  {x: 5.4, y: 0.60, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'
rostopic pub --once /planner/curr_pose geometry_msgs/Pose '{position:  {x: 5.0, y: 0.73, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'
rostopic pub --once /planner/curr_pose geometry_msgs/Pose '{position:  {x: 4.7, y: 0.73, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'
rostopic pub --once /planner/curr_pose geometry_msgs/Pose '{position:  {x: 4.1, y: 0.73, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'

# reach id=2
rostopic pub --once /planner/curr_pose geometry_msgs/Pose '{position:  {x: 3.9, y: 0.73, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'

# with the node reached, we should now be moving towards the next node
echo "Should have reached the target node now. Continue?"
read -n 1 -s

rostopic pub --once /planner/curr_pose geometry_msgs/Pose '{position:  {x: 3.5, y: 0.65, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'
rostopic pub --once /planner/curr_pose geometry_msgs/Pose '{position:  {x: 3.0, y: 0.60, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'
rostopic pub --once /planner/curr_pose geometry_msgs/Pose '{position:  {x: 2.6, y: 0.55, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'

echo "Should have reached global target! Ending test."
