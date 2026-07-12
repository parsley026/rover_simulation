ros2 launch gazebo_simulation simulation.launch.py

ros2 launch rover_kinematics_bridge rover_kinematics_bridge.launch.xml

ros2 launch quad_rover_kinematics qrk.yaml

ros2 run rover_teleop teleop_keyboard 

ros2 launch rover_autonomy playground_mapping.launch

ros2 launch rover_autonomy playground_naviagtion.launch