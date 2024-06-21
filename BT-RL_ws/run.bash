# colcon build and run gazebo
# gnome-terminal -x bash -c "source ~/Fang_Folder/env/bin/activate && source install/setup.bash && ros2 launch ur5_ros2_gazebo ur5_simulation.launch.py;";
# && cd ~/Fang_Folder/BT-RL_ws && colcon build 

# run aruco node
gnome-terminal -x bash -c "source ~/Fang_Folder/env/bin/activate && cd ~/Fang_Folder/BT-RL-Senior-Thesis/BT-RL_ws && source install/setup.bash && ros2 run ros2_aruco aruco_node;";

# run navigation
gnome-terminal -x bash -c "source ~/Fang_Folder/env/bin/activate && cd ~/Fang_Folder/BT-RL-Senior-Thesis/BT-RL_ws && source install/setup.bash && ros2 launch carver_navigation vmegarover_navigation.launch.py;";

# # run state
gnome-terminal -x bash -c "source ~/Fang_Folder/env/bin/activate && cd ~/Fang_Folder/BT-RL-Senior-Thesis/BT-RL_ws && source install/setup.bash && python3 ~/Fang_Folder/BT-RL-Senior-Thesis/BT-RL_ws/src/environment/mm_bt/scripts/mobile_manipulator_state.py;";

# # run action
gnome-terminal -x bash -c "source ~/Fang_Folder/env/bin/activate && cd ~/Fang_Folder/BT-RL-Senior-Thesis/BT-RL_ws && source install/setup.bash && python3 ~/Fang_Folder/BT-RL-Senior-Thesis/BT-RL_ws/src/environment/mm_bt/scripts/mobile_manipulator_action.py";

# run BT
# gnome-terminal -x bash -c "source ~/Fang_Folder/env/bin/activate && cd ~/Fang_Folder/BT-RL_ws && source install/setup.bash && ros2 run mm_bt autonomous_modeV2.py"

# ros2 run mm_bt fang_test.py
#ros2 run rl_brain test.py

# gnome-terminal -x bash -c "source ~/Fang_Folder/env/bin/activate && cd ~/Fang_Folder/BT-RL-Senior-Thesis/BT-RL_ws && source install/setup.bash && ros2 run mm_bt respawn.py;";