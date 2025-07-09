# 保存地图
至pbstream  
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/thunder/ros2_ws/src/startlocal/map/my_map.pbstream'}"  
至pgm&&yaml  
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -map_filestem=/home/thunder/ros2_ws/src/startlocal/map/my_map \
  -pbstream_filename=/home/thunder/ros2_ws/src/startlocal/map/my_map.pbstream \
  -resolution=0.05  
或  
ros2 run nav2_map_server map_saver_cli -t map -f /home/thunder/ros2_ws/src/startlocal/map/my_map

# 改进
现在直接运行scripts/savemap.sh就可以保存，记得在里面修改路径

# 启动导航
运行bringup_nav.launch.py