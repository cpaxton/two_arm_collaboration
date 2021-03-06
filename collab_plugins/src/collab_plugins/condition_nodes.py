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
# Thread for service calls
from threading import Thread

from predicator_msgs.srv import *

entity_types = ['robot','location','object']

# Node Wrappers -----------------------------------------------------------
class NodeConditionExistsGUI(NodeGUI):
    def __init__(self):
        super(NodeConditionExistsGUI,self).__init__()
        self.predicate = NamedComboBox('Entity Type')
        self.param1 = NamedComboBox('Identifier')
        self.layout_.addWidget(self.predicate)
        self.layout_.addWidget(self.param1)

        self.predicate.add_items(entity_types)

    def generate(self,parent=None):
        idx = int(self.predicate.get())
        if all([self.name.full(),self.label.full()]):
            return NodeConditionExists(parent,self.get_name(),self.get_label(),self.predicate.get(),
                    self.param1.get())
        else:
            return 'ERROR: node not properly defined'

# Nodes -------------------------------------------------------------------
class NodeConditionExists(Node):
    def __init__(self,parent,name,label,predicate_name=None,param1=None):
        L = '( condition )\\n' + label.upper()
        color = '#FAE364'
        super(NodeConditionExists,self).__init__(False,parent,name,L,color,'ellipse')
        self.service_thread = Thread(target=self.make_service_call)
        self.predicate_ = predicate_name
        self.param1_ = param1
        self.running = False
        self.finished_with_success = None
    def get_node_type(self):
        return 'CONDITION'
    def get_node_name(self):
        return 'Condition'

    def execute(self):
        #print '  -  Node: ' + self.name_ + ' returned status: ' + self.node_status_
        print 'Executing Condition: (' + self.name_ + ')'
        if not self.running: # Thread is not running
            if self.finished_with_success == None: # Service was never called
                try:
                    self.service_thread.start()
                    return set_status('RUNNING')
                    self.running = True
                except Exception, errtxt:
                    return set_status('FAILURE')
                    self.running = False
        else:
            # If thread is running
            if self.service_thread.is_alive():
                return set_status('RUNNING')
            else:
                if self.finished_with_success == True:
                    return set_status('SUCCESS')
                    self.running = False
                else:
                    return set_status('FAILURE')
                    self.running = False
        return self.node_status_

    def make_service_call(self,*args):
        req = TestPredicateRequest
        req.statement.predicate = self.predicate
        req.params[0] = self.param1
        rospy.wait_for_service('predicator/test_predicate')
        try:
            test_service = rospy.ServiceProxy('predicator/test_predicate', TestPredicate)
            self.result = test_service(request)
            self.finished_with_success = self.result.found
        except rospy.ServiceException, e:
            print e
            self.finished_with_success = False


