while :
do
   cd ~/ros2_ws/map
   ros2 run nav2_map_server map_saver_cli -f assignment_3_map
   sleep 0.5
done
