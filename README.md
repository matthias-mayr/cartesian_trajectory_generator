# Cartesian Trajectory Generator

A simple trajectory generator that creates and publishes a linear trajectory in Cartesian space.

## Features:
- Publish a linear trajectory based on the current pose of a frame and a goal
- Optional synchronization of rotation and translation
- Implements a velocity profile with
  - User-defined separate values for constant acceleration and deceleration
  - Maximum rotational and translational velocity
- Allows implementation of other velocity profiles
- dynamic reconfigure configuration

## Limitations:
- No collision checks
- No feasibility checks
- No handling of existing velocity of the end effector

## How to use:
Open three terminals:

1. $ `roscore`
2. $ `mon launch cartesian_trajectory_generator publisher_demo.launch`
3. $ `mon launch cartesian_trajectory_generator trajectory_generator.launch`
4. $ `rosrun rqt_reconfigure rqt_reconfigure`

In the rqt-gui, choose a new goal pose and click the "ready_to_send" button.

The demo will use three topics:
1. `/cartesian_trajectory_generator/current_goal`: Publishes the current goal
2. `/cartesian_trajectory_generator/new_goal`: Accepts new goals
3. `/cartesian_trajectory_generator/ref_pose`: Publishes the current reference pose


You can send new goal to `/cartesian_trajectory_generator/new_goal` like this:
```
$ rostopic pub /cartesian_trajectory_generator/new_goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'world'
pose:
  position:
    x: 0.0
    y: 0.0
    z: 2.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```