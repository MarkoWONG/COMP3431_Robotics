# COMP3431_Robotics

# Running wall follower on sim 
*make sure your cwd is your workspace
1.  rosdep install -i --from-path src --rosdistro foxy -y
2.  colcon build --symlink-install
3.  source /opt/ros/foxy/setup.bash
4.  source install/local_setup.bash
5.  ros2 launch turtlebot3_gazebo empty_world.launch.py
6.  Open new terminal and in this new terminal do the following
7.  source install/local_setup.bash
8.  ros2 run wall_follower wall_follower

## Adding custom maze to sim
*this takes place inbetween steps 5 and 6 of "Running wall follower on sim"
1. In gazebo, to the insert tab locationed on the left panel
2. click on add path
3. Select the folder the mazes folder (should be at the same level as src)
4. click on dropdown arrow of the newly added folder
5. You should now see all my levels (mazes)
6. you can click on the level of choice
7. Place the maze by click on the grid of where you want to place it (make sure to position it so the robot is in the maze)

# Running wall follower on actual robot
colcon build
1.  source /opt/ros/foxy/setup.bash
2.  source install/local_setup.bash
3.  export TURTLEBOT3_MODEL=waffle_pi
4.  export ROS_DOMAIN_ID=4\
**Terminal 1**\
ssh ubuntu@192.168.1.104\
ros2 launch turtlebot3_bringup robot.launch.py\
**Terminal 2**\
ros2 launch turtlebot3_cartographer cartographer.launch.py\
**Terminal 3**\
ros2 run wall_follower wall_follower\\

**After run**\
ros2 run nav2_map_server map_saver_cli -f ~/{mapName}\
sudo shutdown now (in ssh terminal)

# Notes
1. If your build fails check if there is a empty folder called "launch" in the src/wall_follower/wall_follower. If the folder doesn't exist then create the folder.
2. If cartographer has an error, run bringup again
