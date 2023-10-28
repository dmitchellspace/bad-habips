#!/bin/bash

# Change permissions to the uart
sudo chmod a+rw /dev/ttyAMA0

# Set python path
rootPythonPath="/home/pi/habip/python_classes"

for dir in ${rootPythonPath}/*; do
    if [ -d $dir ]; then
        rootPythonPath="${rootPythonPath}:${dir}"
    fi
done
export PYTHONPATH="${rootPythonPath}"

cd /home/pi/ts_tools

sudo nohup python /home/pi/ts_tools/Switching.py &
sleep 1

sudo nohup python /home/pi/ts_tools/BladeRF.py &
sleep 1
