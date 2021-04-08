# Cartesian Trajectory Generator

A simple trajectory generator that creates and publishes a linear trajectory in Cartesian space.

How to use:
Open three terminals:

1. $ roscore
2. For simulation in gazebo: $ mon launch bh_robot bringup.launch run_moveit:=true 
3. $ roslaunch cartesian_trajectory_generator trajectory_generator.launch
4. $ rosrun rqt_reconfigure rqt_reconfigure

In the rqt-gui, choose a new goal pose and click the "ready_to_send" button.
