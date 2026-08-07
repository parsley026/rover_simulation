Simple db save and load test workflow

Terminal 1:
ros2 launch gazebo_simulation simulation_with_kinematics.launch.py \
        world-name:=marsyard2022.world \
        pose:="0.0 0.0 3.5"

Terminal 2:
ros2 launch rover_autonomy playground_mapping.launch.py \
        database_path:=/home/bartolini/moj_marsyard.db \
        delete_db_on_start:=true

Terminal 3: 
ros2 run rover_teleop teleop_keyboard

Terminal 4:
rviz2

Add /mapping/map or /mapping/(one of available pointclouds) in rviz
Then drive somewhere using Terminal 3
Stop all sessions
Run again but with delete_db_on_start:=false
You will see that the map in rviz will be the same