#!/bin/bash
source setenv.sh
./cleanup_botlab.sh &
wait
./bin/timesync &> /dev/null &
./bin/rplidar_driver &> /dev/null &
#./bin/slam --mapping-only &> /dev/null &
#./bin/motion_controller &
./bin/botgui
