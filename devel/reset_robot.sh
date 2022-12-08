source devel/setup.bash
rosrun intera_interface enable_robot.py -e
roslaunch intera_examples sawyer_tuck.launch
rosrun intera_interface joint_trajectory_action_server.py
bash ~/intera.sh
