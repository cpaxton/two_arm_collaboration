# Two Arm Collaboration

This is a package for creating and analyzing simulations of tasks performed by two of the Barrett WAM arms.

## About

## Requirements

As of 2014-07-14, there are a couple of non-standard versions of standard ROS packages you should have.

- You need an updated version of the MoveIt RVIZ plugin, with the option for namespacing the `move_group` node
- You need an updated version of the Gazebo ROS Plugins

Both of these are available on the LCSR forks github page.

## Launching a Demo

The `collab_env` package contains demos and the launch files to bring them up.
These include some demos with just markers, and some demos with physical objects the robots can interact with.

For example:

```
roslaunch collab_env test7_pegs.launch # launch with pegs and a ring
roslaunch collab_env test7_markers.launch # launch with 2 interactive markers
roslaunch collab_env test7_markers4.launch # launch with 4 interactive markers
roslaunch collab_env test7_square.launch # launch with a square made of magnetic blocks
```

You can also launch a "plain" simulation:

```
roslaunch collab_env test7.launch
```

*Fun fact: the "test7" prefix to all of these environments refers to the 7 DOF WAM arms used, and the fact that, at first, the author was not sure what format these tests should be in or how the packages would be organized. These worked, so they stayed.*

### Launch with Spacenav

The launch files in `collab_env` are included in the `collab_spacenav` package. Use the same names (i.e., `test7_pegs.launch`) in this package to launch with the Space Navigator 3D mouse.

For example:

```
roslaunch collab_spacenav test7_pegs.launch
```

### Launch with Razer Hydras

Right now, only one Razer Hydra package is in place. You can launch a version of the peg demo, with both arms being controlled by the movements of the Razer Hydras, with the command:

```
roslaunch collab_hydra pegs1.launch
```

### Peg Assistance Demo

The Peg Assistance Demo (`peg_assist_demo`) is a package in development *(as of 07/02)* that should integrate a number of different tools. There is a task execution plan (currently a SMACH placeholder, but will be replaced with CoSTAR Instructor), predicates, and MoveIt to control an automated arm while a human operator controls a primary arm.

As of *(07/22)* I am also adding a second part to the peg assistance demo: an all-automatic launch file. You can run this with:

```
roslaunch peg_assist_demo pegs1_auto.launch
```

#### Options

*Console*: You can bring up the RQT console to debug by setting `console:=true` from roslaunch.
*Smach Viewer*: You can bring up the smach viewer by setting `smach_viewer:=true` from roslaunch.

#### Known Issues

- The controllers don't always work right. Sometimes, you run into issues because position error is too high. If so, just replan a path.
- RViz crashes a lot on my NVidia graphics card.
- There may be a memory leak that shows up with RViz and the MoveIt motion planning plugin.

### Markers Demo

The Markers demo is a very simple demo. Up to four markers exist in the world; these are considered to be both objects and locations for interaction.

Launch a demo with two markers:

```
roslaunch markers_demo markers2.launch
```

Launch a demo with four markers:

```
roslaunch markers_demo markers4.launch
```

## Other Tools

### Replay

Replay is an older set of tools. It collects features (TF frame relationships) to export so that we can use these relationships in robot learning from demonstration. This functionality has been replaced by the Predicator set of tools, but you may still want to look at Replay for its support of DMPs and playing commands on the simulated arms.
Replay has tools to record Barrett arm trajectories into bags with features, and to replay these in simulation.

You should usually launch simulations with the SpaceNav controller at first, to make sure there are always TF frames published. The Barrett inverse kinematics controllers will die if no such frames (`/wam/cmd` and `/wam2/cmd`) are available.

Before you replay a bag on the arms, make sure to run:

```
rosnode kill spacenav_client_node
```

TODO: Replay can be modified from its current state to record Predicator messages instead.

### Instructor Plugins

The `collab_plugins` package contains/will contain information to create nodes in Instructor that can control the robot or test for robot-related conditions.

### Construction Plugin

## Troubleshooting

### Contact

The `two_arm_collaboration` project is maintained by Chris Paxton (cpaxton3@jhu.edu)

