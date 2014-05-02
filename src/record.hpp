#ifndef LCSR_REPLAY_RECORD
#define LCSR_REPLAY_RECORD

// ros includes
#include <ros/ros.h>
#include <rosbag/bag.h>

// boost includes
#include <boost/bind/bind.hpp>

//message includes
#include <geometry_msgs/TransformStamped.h>
#include <oro_barrett_msgs/BHandCmd.h>

namespace lcsr_replay {

  /* typedef to avoid worrying about templates just yet */
  typedef oro_barrett_msgs::BHandCmd discrete_msg_t;
  typedef oro_barrett_msgs::BHandCmdConstPtr discrete_msg_ptr;
  typedef geometry_msgs::TransformStamped msg_t;
  typedef geometry_msgs::TransformStampedConstPtr msg_ptr;


  class DemoWriter {
  private:
    int verbosity;

  protected:
    std::vector<std::string> topics;

    ros::NodeHandle nh; // default namespace
    ros::NodeHandle nh_tilde; //private namespace for parameters


    rosbag::Bag bag;

    // set of all subscribers
    std::map<std::string, ros::Subscriber> subscribers;

    // set of all command messages
    std::map<std::string, discrete_msg_t> hand_msgs;

    // set of all transform messages 
    std::map<std::string, msg_t> arm_msgs;

  public:

    void discrete_callback(const discrete_msg_ptr &msg, const std::string &topic) {
      
    }

    DemoWriter(const std::string &bagfile) : bag(bagfile, rosbag::bagmode::Write), nh(), nh_tilde("~") {

      nh_tilde.param("verbosity", verbosity, int(1));

      if(verbosity > 0) {
        ROS_INFO("Initialized Demonstration Writer with file=\"%s\"!", bagfile.c_str());
      }
    }


    void addDiscreteTopic(const std::string &topic) {
      subscribers[topic] = nh.subscribe<discrete_msg_t>(topic, 1,
        boost::bind(&DemoWriter::discrete_callback, this, _1, topic));
    }

    void def_callback(const msg_ptr &msg, const std::string &topic) {

    }

    void addTopic(const std::string &topic) {
      subscribers[topic] = nh.subscribe<msg_t>(topic, 1,
        boost::bind(&DemoWriter::def_callback, this, _1, topic));
    }

  };

}

#endif
