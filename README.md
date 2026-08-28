source install/setup.bash

ros2 launch gazebo_simulation simulation_with_kinematics.launch.py world-name:='marsyard2020.world' pose:='1.0 2.0 1.0'
ros2 launch gazebo_simulation simulation_with_kinematics.launch.py world-name:='mars_yard.sdf'
ros2 launch gazebo_simulation simulation.launch.py

ros2 launch rover_kinematics_bridge rover_kinematics_bridge.launch.xml

ros2 launch rover_kinematics qrk.yaml

ros2 run rover_teleop teleop_keyboard 

ros2 launch rover_autonomy_outdated playground_mapping.launch.py 

ros2 launch rover_autonomy_outdated playground_naviagtion.launch.py

rviz2 --ros-args --remap use_sim_time:=true

ros2 launch rover_autonomy main_compute.launch.py 

ros2 run rover_autonomy_outdated path_recorder --ros-args -p odom_topic:=/kinematics/odom

ros2 service call /rover_recovery/escape std_srvs/srv/Trigger

ros2 launch rosbridge_server rosbridge_websocket_launch.xml

python3 web_action_proxy.py