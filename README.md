# Cartesian Trajectory Generator

A simple trajectory generator that creates and publishes a linear trajectory in Cartesian space.

How to use:

Open 4 terminals:

1. $ roscore

2. a) for simulation in gazebo: $ mon launch bh_robot bringup.launch run_moveit:=true 
2. b) for debugging_ $ rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 world bh_link_ee 100 

3. $ rosrun cartesian_trajectory_generator cartesian_trajectory_generator /bh/CartesianImpedance_trajectory_controller/target_pose bh_link_ee 50 0.5 0.5

4. $ rosrun rqt_reconfigure rqt_reconfigure

In the rqt-gui, choose a new goal pose and click the "ready_to_send" button.
