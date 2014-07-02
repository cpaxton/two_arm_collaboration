###################
# Features Message
# Contains information on the world state for a prior demonstration
# Collected via:
#   rosrun lcsr_replay record [filename.bag]
#
###################
Header h
##################
# reference frame: command point/hand location
#string[] base
# compared to: peg stand/ring/hand position
#string[] child
# transform: position/rotation difference
string[] names
geometry_msgs/Transform[] transform
