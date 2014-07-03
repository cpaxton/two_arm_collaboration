
from oro_barrett_msgs.msg import *
from collab_msgs.srv import *
from collab_msgs.msg import *
import tf

'''
Publish the current gripper/IK commands to the arms
'''
class CollabManager(object):

    __init__(self, arms=2):

        if(arms > 2):
            print "More than two arms not supported at this time."

        if (arms == 2):
            # start up publishers and subscribers for arm 2
            pass

        # start up publishers and subscribers for arm 1

    '''
    tick()
    publish command messages for the 
    '''
    def tick():
        pass

if __name__ == "__main__":

    rospy.init_node('collaboration_arm_manager')

    spin_rate = rospy.get_param('rate',10)
    arms = int(rospy.get_param('arms', 2))
    rate = rospy.Rate(spin_rate)

    print "starting multi-arm collaboration manager"

    try:

        cm = CollabManager(arms)

        while not rospy.is_shutdown():

            rate.sleep()

    except rospy.ROSInterruptException: pass
