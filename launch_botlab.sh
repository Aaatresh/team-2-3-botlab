#!/bin/bash
source setenv.sh
./bin/timesync &> /dev/null &
./bin/rplidar_driver &> /dev/null &
#./bin/slam  &> /dev/null &
#./bin/motion_controller &
./bin/botgui
