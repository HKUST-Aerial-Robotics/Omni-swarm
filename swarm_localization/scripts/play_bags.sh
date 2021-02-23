echo "Will run bags"
start_t=$[`date +%s`+10]
docker exec swarm1 bash /root/swarm_ws/src/swarm_localization/swarm_localization/scripts/play_bag.sh $1 $start_t &
docker exec swarm2 bash /root/swarm_ws/src/swarm_localization/swarm_localization/scripts/play_bag.sh $2 $start_t &
#docker exec swarm5 /root/swarm_ws/src/swarm_localization/swarm_localization/scripts/play_bag.sh $3 $4 &