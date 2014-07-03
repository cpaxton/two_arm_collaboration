#!/usr/bin/env python
import roslib; roslib.load_manifest('instructor_plugins')
import rospy 
from std_msgs.msg import *
# Qt
from PyQt4 import QtGui, QtCore, uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *
# Beetree and Instructor
import beetree; from beetree import Node
from instructor_core import NodeGUI
from instructor_core.instructor_qt import NamedField
from instructor_core.instructor_qt import NamedComboBox
# For testing the service node
from instructor_plugins.srv import *

from predicator_msgs.msg import *
from predicator_core.srv import *

# Sample Node Wrappers -----------------------------------------------------------
'''
class NodeActionSampleGUI(NodeGUI):
    def __init__(self):
        super(NodeActionSampleGUI,self).__init__()

    def generate(self,parent=None):
        if all([self.name.full(),self.label.full()]):
            return beetree.NodeAction(parent,self.get_name(),self.get_label())
        else:
            return 'ERROR: node not properly defined'
'''

get_params = rospy.ServiceProxy('/predicator/get_possible_assignment', GetTypedList)

'''
NodeSetDestinationGUI()
choose a robot
choose a location
Set destination to some frame

written 2014/07/03
'''
class NodeSetDestinationGUI(NodeGUI):
    def __init__(self):
        super(NodeSetDestinationGUI,self).__init__()
        self.location = NamedComboBox('Destination')
        self.robot = NamedComboBox('Robot')
        self.layout_.addWidget(self.robot)
        self.layout_.addWidget(self.location)

        self.robots_list = get_params(id="robot").data
        self.locations_list = get_params(id="location").data

        self.robot.add_items(self.robots_list)
        self.location.add_items(self.locations_list)

    def generate(self,parent=None):

        if len(self.robots_list) < 1:
            return 'ERROR: no robots defined!'
        elif len(self.locations_list) < 1:
            return 'ERROR: no locations defined!'

        robot = self.robots_list[int(self.robot.get())]
        location = self.locations_list[int(self.robot.get())]

        if all([self.name.full(),self.label.full()]):
            return NodeSetDestination(parent,self.get_name(),self.get_label(),
                    robot, location)
        else:
            return 'ERROR: node not properly defined'

'''
NodeMoveToDestinationGUI()
choose a robot
move to destination

written 2014/07/03
'''
class NodeMoveToDestinationGUI(NodeGUI):
    def __init__(self):
        super(NodeMoveToDestinationGUI,self).__init__()
        self.robot = NamedComboBox('Robot')
        self.layout_.addWidget(self.robot)

        self.robots_list = get_params(id="robot").data

        self.robot.add_items(self.robots_list)

        self.time = NamedField('Time Limit','0.0')
        self.tolerance = NamedField('Tolerance','0.1')
        self.layout_.addWidget(self.time)
        self.layout_.addWidget(self.tolerance)

    def generate(self,parent=None):

        if len(self.robots_list) < 1:
            return 'ERROR: no robots defined!'
        elif len(self.locations_list) < 1:
            return 'ERROR: no locations defined!'

        robot = self.robots_list[int(self.robot.get())]

        if all([self.name.full(),self.label.full(), self.time.full(), self.tolerance.full()]):
            return NodeMoveToDestination(parent,self.get_name(),self.get_label(),
                    robot,self.time.get(),self.tolerance.get())
        else:
            return 'ERROR: node not properly defined'

'''
NodeOpenGripperGUI
Choose a robot
Open gripper

written 2014/07/03
'''
class NodeOpenGripperGUI(NodeGUI):
    def __init__(self):
        super(NodeOpenGripperGUI,self).__init__()
        self.robot = NamedComboBox('Robot')
        self.layout_.addWidget(self.robot)

        self.robots_list = get_params(id="robot").data

        self.robot.add_items(self.robots_list)

        self.time = NamedField('Time Limit','0.0')
        self.tolerance = NamedField('Tolerance','0.1')
        self.layout_.addWidget(self.time)
        self.layout_.addWidget(self.tolerance)

    def generate(self,parent=None):

        if len(self.robots_list) < 1:
            return 'ERROR: no robots defined!'
        elif len(self.locations_list) < 1:
            return 'ERROR: no locations defined!'

        robot = self.robots_list[int(self.robot.get())]

        if all([self.name.full(),self.label.full(), self.time.full(), self.tolerance.full()]):
            return NodeOpenGripper(parent,self.get_name(),self.get_label(),
                    robot,self.time.get(),self.tolerance.get())
        else:
            return 'ERROR: node not properly defined'

'''
NodeCloseGripperGUI()
Choose a robot
Close gripper

written 2014/07/03
'''
class NodeCloseGripperGUI(NodeGUI):
    def __init__(self):
        super(NodeCloseGripperGUI,self).__init__()
        self.robot = NamedComboBox('Robot')
        self.layout_.addWidget(self.robot)

        self.robots_list = get_params(id="robot").data

        self.robot.add_items(self.robots_list)

        self.time = NamedField('Time Limit','0.0')
        self.tolerance = NamedField('Tolerance','0.1')
        self.layout_.addWidget(self.time)
        self.layout_.addWidget(self.tolerance)

    def generate(self,parent=None):

        if len(self.robots_list) < 1:
            return 'ERROR: no robots defined!'
        elif len(self.locations_list) < 1:
            return 'ERROR: no locations defined!'

        robot = self.robots_list[int(self.robot.get())]

        if all([self.name.full(),self.label.full(), self.time.full(), self.tolerance.full()]):
            return NodeCloseGripper(parent,self.get_name(),self.get_label(),
                    robot,self.time.get(),self.tolerance.get())
        else:
            return 'ERROR: node not properly defined'

class NodeQueryClosestObjectGUI(NodeGUI):
    def __init__(self):
        super(NodeQueryClosestObjectGUI,self).__init__()

    def generate(self,parent=None):
        if all([self.name.full(),self.label.full()]):
            return NodeQueryClosestObject(parent,self.get_name(),self.get_label())
        else:
            return 'ERROR: node not properly defined'

# Sample Nodes -------------------------------------------------------------------
class NodeQueryClosestObject(Node):
    def __init__(self,parent,name,label):
        color='#5B8EEB'
        super(NodeQueryClosestObject,self).__init__(False,parent,name,label,color)
    def get_node_type(self):
        return 'SERVICE'
    def get_node_name(self):
        return 'Service'
    def execute(self):
        # TODO
        print 'Executing Service: ' + self.name_
        self.node_status_ = 'SUCCESS'
        print '  -  Node: ' + self.name_ + ' returned status: ' + self.node_status_
        return self.node_status_


class NodeMoveToDestination(Node):
    def __init__(self,parent,name,label,robot,location):
        super(NodeServiceSample,self).__init__(False,parent,name,label,'#92D665')
        self.robot_ = robot

    def get_node_type(self):
        return 'SERVICE'
    def get_node_name(self):
        return 'Service'

    def execute(self):
        return set_status('SUCCESS')

class NodeOpenGripper(Node):
    def __init__(self,parent,name,label,robot,location):
        super(NodeServiceSample,self).__init__(False,parent,name,label,'#92D665')
        self.robot_ = robot

    def get_node_type(self):
        return 'SERVICE'
    def get_node_name(self):
        return 'Service'

    def execute(self):
        return set_status('SUCCESS')

class NodeCloseGripper(Node):
    def __init__(self,parent,name,label,robot,location):
        super(NodeServiceSample,self).__init__(False,parent,name,label,'#92D665')
        self.robot_ = robot

    def get_node_type(self):
        return 'SERVICE'
    def get_node_name(self):
        return 'Service'

    def execute(self):
        return set_status('SUCCESS')

'''
NodeSetDestination()
store location to predicate server
that's it

written 2014/07/03
'''
class NodeSetDestination(Node):
    def __init__(self,parent,name,label,robot,location):
        super(NodeServiceSample,self).__init__(False,parent,name,label,'#92D665')
        self.pub_ = rospy.Publisher('predicator/update_param', predicator_msgs.msg.UpdateParam)
        self.robot_ = robot
        self.location_ = location

    def get_node_type(self):
        return 'SERVICE'
    def get_node_name(self):
        return 'Service'

    def execute(self):
        msg = predicator_msgs.msg.UpdateParam()
        msg.operation = UpdateParam.PUBLISH_PREDICATE
        msg.statement.predicate = "destination_frame"
        msg.statement.num_params = 2
        msg.statement.params = [self.robot_, self.location_, '']

        self.pub_.publish(msg)

        return set_status('SUCCESS')

