while :
do
   cd ~/ros2_ws/home/Gruppe4/ros2_ws/install/map_server/share/map_server/config/
   ros2 run nav2_map_server map_saver_cli -f turtlebot_area
   sleep 0.5
done
