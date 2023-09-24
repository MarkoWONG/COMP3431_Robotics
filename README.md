# COMP3431_Robotics

# Running wall follower on sim
*make sure your cwd is yuor workspace
1.  rosdep install -i --from-path src --rosdistro foxy -y
2.  colcon build --symlink-install
3.  source /opt/ros/foxy/setup.bash
4.  source install/local_setup.bash
5.  ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
6.  Open new terminal and in this new terminal do the following
7.  source install/local_setup.bash
8.  ros2 run wall_follower wall_follower
