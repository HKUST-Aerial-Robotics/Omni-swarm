#!/bin/bash
source /opt/ros/melodic/setup.bash
/opt/ros/melodic/lib/rosbag/play --queue 100 --rate 1.0 --delay 0.2 --start $2 /root/bags/swarm_local_2020_12/$1 --rate-control-max-delay 1.0 & 