#!/usr/env/python

import rospy
import control_msgs.msg
import actionlib
import trajectory_msgs.msg

rospy.init_node('dotdotdot')
client = actionlib.SimpleActionClient('/gazebo/w2traj_rml/action', control_msgs.msg.FollowJointTrajectoryAction)
goal = control_msgs.msg.FollowJointTrajectoryGoal()
goal.trajectory.joint_names = ['wam2/base_yaw_joint', 'wam2/shoulder_pitch_joint', 'wam2/shoulder_yaw_joint', 'wam2/elbow_pitch_joint', 'wam2/wrist_yaw_joint', 'wam2/wrist_pitch_joint', 'wam2/palm_yaw_joint']

position = [0.008328843321058876, -1.483062647536216, -1.217074073556475, -0.005712382625344681, 1.2163121969484987, -1.0463223725342576, -0.10470283738232222]
velocity = [0.009395809632488558, 0.0027401312884123745, 0.005052757306473386, -0.014209957512283738, -0.04330845164226659, 0.007475119807713124, -0.023230773950301726]
effort = [-1.0785431805112875, 2.512435141404044, -2.93663211302304, -0.9600791293870343, 0.21004171182834, -1.1568855489600967, -0.08858930576828059]

point = trajectory_msgs.msgs.JointTrajectoryPoint

point.positions = position
#point.velocity = velocity
#point.effort = effort

goal.trajectory.points.append(point)

client.send_goal(goal)
client.wait_for_server()
