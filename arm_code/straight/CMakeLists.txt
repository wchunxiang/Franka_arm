cmake_minimum_required(VERSION 3.0.2)
project(arm_control_magnet_straight)

# Python scripts only: nothing to build, catkin just registers the package
# so that `rosrun` / `roslaunch` can find it.
find_package(catkin REQUIRED)
catkin_package()
