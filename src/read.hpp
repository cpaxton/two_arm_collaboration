#ifndef _LCSR_REPLAY
#define _LCSR_REPLAY

// STL includes
#include <vector>
#include <string>
#include <map>

// ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// LCSR includes
#include <oro_barrett_msgs/BHandCmd.h>

namespace lcsr_replay {

  /* typedef to avoid worrying about templates just yet */
  typedef oro_barrett_msgs::BHandCmd discrete_msg_t;

  /**
   * DemoReader
   * Read a demonstration from ROSBAG file.
   * Extract features.
   * */
  class DemoReader : rosbag::Bag {
  private:
    int verbosity;

    const int DEFAULT_RATE = 60;

  protected:
    
    std::vector<std::string> topics; // list of all topics

    ros::NodeHandle nh; // default namespace for publishers
    ros::NodeHandle nh_tilde; // private namespace

    // record certain things which we just send as-is at times, we do not interpret or warp them
    std::vector<ros::Publisher> discrete_topics;

  public:

    DemoReader(const std::string &bagfile) : rosbag::Bag(bagfile), nh(), nh_tilde("~") {

      nh_tilde.param("verbosity", verbosity, int(1));

      if(verbosity > 0) {
        ROS_INFO("Initializing Demonstration Reader...");
      }
    }

    /* addDiscreteTopic()
     * Add a discrete topic.
     * For these, we look at what messages get sent and when.
     * We repeat them when the right conditions are met.
     * 
     * FUTURE PLANS: template function that adds a ROS publisher?
     */
    //template<class T = discrete_msg_t>
    int addDiscreteTopic(const std::string &topic, int rate = DEFAULT_RATE) {
      topics.push_back(topic);

      ros::Publisher pub = nh.advertise<discrete_msg_t>(topic, rate);

      if(verbosity > 0) {
        ROS_INFO("Adding discrete topic %s, publishing at rate %d", topic.c_str(), rate);
      }

      return 1;
    }



    /* addTopic()
     * Add a continuous/inverse kinematics command topic.
     * These are interpreted according to feature counts.
     */
    int addTopic(const std::string &topic, int rate = DEFAULT_RATE) {
      topics.push_back(topic);

      if(verbosity > 0) {
        ROS_INFO("Adding topic %s, publishing at rate %d", topic.c_str(), rate);
      }

      return 1;
    }

    int examine() {
      rosbag::View view((rosbag::Bag *this, rosbag::TopicQuery(topics)));

      foreach(rosbag::MessageInstance const m, view) {
        ROS_INFO("topic=\"%s\", time=%f", m.getTopic().c_str(), m.getTime().toSec());
      }
    }
    
  }




}


#endif
