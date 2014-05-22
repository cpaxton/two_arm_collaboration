#!/usr/bin/env python
import roslib; 
roslib.load_manifest('lcsr_replay')

import rospy 
import numpy as np

from replay import *

from dmp.srv import *
from dmp.msg import *

#Learn a DMP from demonstration data
def makeLFDRequest(dims, traj, t, K_gain, 
                   D_gain, num_bases):
    demotraj = DMPTraj()
        
    for i in range(len(traj)):
        pt = DMPPoint();
        pt.positions = traj[i]
        demotraj.points.append(pt)
        demotraj.times.append(t[i]-t[0])
            
    k_gains = [K_gain]*dims
    d_gains = [D_gain]*dims
        
    print "Starting LfD..."
    rospy.wait_for_service('learn_dmp_from_demo')
    try:
        lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
        resp = lfd(demotraj, k_gains, d_gains, num_bases)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "LfD done"    
            
    return resp;


#Set a DMP as active for planning
def makeSetActiveRequest(dmp_list):
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


#Generate a plan from a DMP
def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                    seg_length, tau, dt, integrate_iter):
    print "Starting DMP planning..."
    rospy.wait_for_service('get_dmp_plan')
    try:
        gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
        resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, 
                   seg_length, tau, dt, integrate_iter)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "DMP planning done"   
            
    return resp;


if __name__ == '__main__':
    rospy.init_node('replay_dmp_node')

    io = default_io_startup()
    io.parse()

    segment = rospy.get_param('~segment', 0)
    topic = rospy.get_param('~topic', '/wam/cmd');

    (t, traj) = io.getTrajectory(topic, segment)

    #Create a DMP from a 2-D trajectory
    dims = len(traj[0])
    K = 100                 
    D = 2.0 * np.sqrt(K)      
    num_bases = 10

    print "Requesting DMP from trajectory of length %d with %d dimensions"%(len(traj), dims)
    resp = makeLFDRequest(dims, traj, t, K, D, num_bases)

    print resp

    #Set it as the active DMP
    makeSetActiveRequest(resp.dmp_list)

    #Now, generate a plan
    x_0 = [x for x in traj[-1]]          #Plan starting at a different point than demo 
    x_dot_0 = [0.0]*dims
    t_0 = 0                
    goal = [x for x in traj[0]]         #Plan to a different goal than demo
    goal_thresh = [0.01]*dims
    seg_length = -1          #Plan until convergence to goal
    tau = 2 * resp.tau       #Desired plan should take twice as long as demo
    dt = (t[-1] - t[0]) / len(t) # speed to replay the trajectory at 
    integrate_iter = 5       #dt is rather large, so this is > 1  

    print dt
    print x_0
    print goal

    plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                           seg_length, tau, dt, integrate_iter)

    new_traj = []

    for i in range(len(plan.plan.points)) :
        new_traj += [plan.plan.points[i].positions]

    io.play_trajectory(new_traj, plan.plan.times, topic)

    if verbosity > 2 :
        print plan

