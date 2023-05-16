#!/usr/bin/bash
# remove type hints from python files
# requires there to be a python3 venv in ~/venv with the strip-hints package installed

source ~/venv/bin/activate

cd ~/asclinic-system/catkin_ws/src/asclinic_pkg/src/nodes/pathplanning

for file in $(ls | grep "\.py$"); do
    strip-hints $file > /tmp/stripped \
    && cat /tmp/stripped > $file
done

deactivate
