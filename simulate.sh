#!/bin/bash
# cd workspace
cd ~/devel_ws || exit
# build the whole workspace
colcon build
# source terminal
# . install/setup.bash

MAP=${1}

cd ~/devel_ws || exit

. ~/devel_ws/install/setup.bash

killall rviz
killall gazebo

ros2 launch explorer_bringup ${MAP}.launch.py > ./ros_launch.log


# open new terminal-1 ros2-launch for simulations
# gnome-terminal -t " ros2-launch " -x bash -c " cd ~/devel_ws; . install/setup.bash; killall rviz; killall gazebo; ros2 launch explorer_bringup map1.launch.py > ./ros_launch.log; exec bash;"

# open another new terminal-2 ros2-run for control simulations
# gnome-terminal -t " ros2-run " -x bash -c " cd ~/devel_ws; . install/setup.bash; ros2 run explorer_bringup manager > ./ros_run.log; exec bash;"

# sleep 5 seconds waiting ros launch & run finish
# sleep 8
# gnome-terminal -t " test " -x bash -c " cd ~/devel_ws; . install/setup.bash; ros2 topic echo /amcl_pose; exec bash;"

# ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: x: 0.0 y: 0.0 z: 0.0 angular: x: 0.0 y: 0.0 z: 0.3" -r 3

# ros2 topic pub --rate 3 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.8}}"
