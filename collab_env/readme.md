== LCSR Collaboration Experiments ==

Includes materials for peg transfer demo, and user control of two separate WAM arms at once.

=== Launch Files ===

=== Plans ===

- This is the core of a group of different packages, providing only the actual experiments themselves and some tools.
- I may also put ROSINSTALL files here eventually, to load all the packages we need.
- Packages:
  - This package contains demos, like the peg task.
  - LCSR\_Spacenav contains wrappers for these launch files that also start up a Space Navigator mouse.
  - LCSR\_Replay has tools for saving and replaying trajectories.
  - LCSR\_Construction\_Plugin contains the Gazebo plugin I wrote/I am working on.
  - RPD\_Demo contains/will contain more demos for programming from demonstration.
- Learning task segments:
  - segments begin/end with gripper and arm switching
  - we can learn how to recognize segements using something
