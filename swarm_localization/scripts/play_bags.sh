echo "Will run bags"
docker exec swarm1 /root/swarm_ws/src/swarm_localization/swarm_localization/scripts/play_bag.sh $1 &
docker exec swarm2 /root/swarm_ws/src/swarm_localization/swarm_localization/scripts/play_bag.sh $2