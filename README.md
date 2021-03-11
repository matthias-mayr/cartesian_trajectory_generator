# Cartesian Trajectory Generator

A simple trajectory generator that creates and publishes a linear trajectory in Cartesian space.

How to use in gazebo:

Open 3 terminals:

1. $ roscore
2. $ mon launch bh_robot bringup.launch run_moveit:=true 
3. $ rosrun cartesian_trajectory_generator cartesian_trajectory_generator -0.5 0 1.5 0 0 0 1 /bh/CartesianImpedance_trajectory_controller/target_pose bh_link_ee 100 4.0 0.2

