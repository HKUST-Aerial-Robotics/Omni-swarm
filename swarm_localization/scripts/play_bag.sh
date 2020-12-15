#!/bin/bash
source /opt/ros/melodic/setup.bash
/opt/ros/melodic/lib/rosbag/play --queue 100 --rate 0.5 --delay 0.2 --start 0.0 /root/bags/swarm_local_2020_12/$1 --rate-control-max-delay 1.0 & 