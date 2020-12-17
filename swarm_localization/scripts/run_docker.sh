#!/bin/bash
#xhost +local:root
docker run --name swarm$1 --gpus all --rm -it \
    -v /home/xuhao/swarm_ws/src:/root/swarm_ws/src \
    -v/home/xuhao/swarm_ws/build:/root/swarm_ws/build \
    -v/home/xuhao/swarm_ws/devel:/root/swarm_ws/devel \
    -v/home/xuhao/swarm_ws/devel:/home/xuhao/swarm_ws/devel \
    -v/home/xuhao/swarm_ws/src:/home/xuhao/swarm_ws/src \
    -v/home/xuhao/source/:/home/xuhao/source/ \
    -v/home/xuhao/bags:/root/bags/ \
    -v/home/xuhao/Dropbox/data/TRO2020-SwarmLocal/tro2020-bags/:/root/bags2/ \
    -v/home/xuhao/output/output$1:/root/output/ \
    -v/home/xuhao/bags/swarm_local_2020_12/Configs/SwarmConfig$1:/root/SwarmConfig/ \
    --env="DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    xuhao1/swarm2020:pc /root/swarm_ws/src/swarm_localization/swarm_localization/scripts/run_bag_replay.sh $1

# docker run --name swarm2 --gpus all --rm -it \
#     -v /home/xuhao/swarm_ws/src:/root/swarm_ws/src \
#     -v/home/xuhao/swarm_ws/build:/root/swarm_ws/build \
#     -v/home/xuhao/swarm_ws/devel:/root/swarm_ws/devel \
#     -v/home/xuhao/swarm_ws/devel:/home/xuhao/swarm_ws/devel \
#     -v/home/xuhao/swarm_ws/src:/home/xuhao/swarm_ws/src \
#     -v/home/xuhao/source/:/home/xuhao/source/ \
#     -v/home/xuhao/bags:/root/bags/ \
#     -v/home/xuhao/Dropbox/data/TRO2020-SwarmLocal/tro2020-bags/:/root/bags2/ \
#     xuhao1/swarm2020:pc /root/run_bag_replay.sh
