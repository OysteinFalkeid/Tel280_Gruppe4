while :
do
   cd /home/Gruppe4/ros2_ws/install/path_planner_server/share/path_planner_server/config
   ros2 run nav2_map_server map_saver_cli -f turtlebot_area
   sleep 2
done
