#ifndef _LCSR_REPLAY_READ
#define _LCSR_REPLAY_READ

// STL includes
#include <vector>
#include <string>
#include <map>

// ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>

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
  class DemoReader {
  private:
    int verbosity;

    static const int DEFAULT_RATE = 60;

  protected:
    
    std::vector<std::string> topics; // list of all topics

    ros::NodeHandle nh; // default namespace for publishers
    ros::NodeHandle nh_tilde; // private namespace

    // record certain things which we just send as-is at times, we do not interpret or warp them
    std::vector<ros::Publisher> discrete_topics;

    rosbag::Bag bag;

  public:

    DemoReader(const std::string &bagfile) : bag(bagfile, rosbag::bagmode::Read), nh(), nh_tilde("~") {

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

    /* examine()
     * Read the contents of a bag so that we know what happened at each point in time in the demonstration.
     */
    int examine() {

      rosbag::View view(bag, rosbag::TopicQuery(topics));

      for(const rosbag::MessageInstance &m: view) {
      //for(rosbag::View::iterator it = view.begin(); it != view.end(); ++it) {
        //rosbag::MessageInstance m = *it;
        ROS_INFO("topic=\"%s\", time=%f", m.getTopic().c_str(), m.getTime().toSec());
      }

      return 1;
    }
    
  };




}


#endif
