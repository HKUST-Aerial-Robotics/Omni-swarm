#!/bin/bash
source /opt/ros/melodic/setup.bash
source /root/swarm_ws/devel/setup.bash
BAGPATH=/root/bags/$1
#/opt/ros/melodic/lib/rosbag/play --queue 100 --rate 1.0 --delay 0.2 --duration $2 /root/bags/$1 --rate-control-max-delay 1.0 --clock& 
export LCM_DEFAULT_URL=udpm://239.255.76.67:7667?ttl=1
rosrun sync_bag_player sync_rosplay.py --path $BAGPATH --syst $2 --rate $3
