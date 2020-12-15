#!/bin/bash
source /opt/ros/melodic/setup.bash
source /root/swarm_ws/devel/setup.bash
echo "Run bag replay for drone",$@
roslaunch swarm_localization bag-replay.launch viz:=false drone_id:=$1 vins:=true loop:=true comm:=true \
    bag_path:=/root/output/swarm_local_pc.bag config_file:=/root/SwarmConfig/fisheye_ptgrey_n3/fisheye_cuda.yaml
