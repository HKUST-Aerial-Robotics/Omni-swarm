echo "Will run bags"
start_t=$[`date +%s`+10]
docker exec swarm1 bash /root/swarm_ws/src/swarm_localization/swarm_localization/scripts/play_bag.sh $1 $start_t $4 &
docker exec swarm2 bash /root/swarm_ws/src/swarm_localization/swarm_localization/scripts/play_bag.sh $2 $start_t $4 &
docker exec swarm5 bash /root/swarm_ws/src/swarm_localization/swarm_localization/scripts/play_bag.sh $3 $start_t $4 &
#rosrun sync_bag_player sync_bag_cmd.py