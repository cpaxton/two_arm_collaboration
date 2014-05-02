#ifndef LCSR_REPLAY_RECORD
#define LCSR_REPLAY_RECORD

#include "transform.hpp"

// stl includes
#include <set>
#include <map>
#include <iostream>

// ros includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <XmlRpcValue.h>

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
    ros::Rate rate_;

    static const int DEFAULT_RATE = 60;

    DemoTransformFinder finder;

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

    std::set<std::string> discrete_topics;
    std::set<std::string> def_topics;

    std::vector<std::string> bases;
    std::vector<std::string> children;

  public:

    void discrete_callback(const discrete_msg_ptr &msg, const std::string &topic) {
      hand_msgs[topic] = *msg;
    }

    DemoWriter(const std::string &bagfile, ros::Rate rate = ros::Rate(DEFAULT_RATE))
      : bag(bagfile, rosbag::bagmode::Write),
      finder(),
      rate_(rate),
      nh(),
      nh_tilde("~")
    {

      nh_tilde.param("verbosity", verbosity, int(1));

      XmlRpc::XmlRpcValue base_list;
      XmlRpc::XmlRpcValue child_list;

      if(nh_tilde.hasParam("base_frames") &&  nh_tilde.hasParam("child_frames")) {
        nh_tilde.param("base_frames", base_list, base_list);
        nh_tilde.param("child_frames", child_list, child_list);
        for(int i = 0; i < base_list.size(); ++i) {
          if(base_list[i].getType() == XmlRpc::XmlRpcValue::TypeString &&
            child_list[i].getType() == XmlRpc::XmlRpcValue::TypeString)
          {
            std::string b = static_cast<std::string>(base_list[i]);
            std::string c = static_cast<std::string>(child_list[i]);
            ROS_INFO("Feature: transform from %s to %s", b.c_str(), c.c_str());
            bases.push_back(b);
            children.push_back(c);
          }
        }
      }

      if(verbosity > 0) {
        ROS_INFO("Initialized Demonstration Writer with file=\"%s\"!", bagfile.c_str());
      }
    }


    void addDiscreteTopic(const std::string &topic, int rate = DEFAULT_RATE) {
      subscribers[topic] = nh.subscribe<discrete_msg_t>(topic, rate,
        boost::bind(&DemoWriter::discrete_callback, this, _1, topic));
      discrete_topics.insert(topic);
    }

    void def_callback(const msg_ptr &msg, const std::string &topic) {
        arm_msgs[topic] = *msg;
    }

    void addTopic(const std::string &topic, int rate = DEFAULT_RATE) {
      subscribers[topic] = nh.subscribe<msg_t>(topic, rate,
        boost::bind(&DemoWriter::def_callback, this, _1, topic));
      def_topics.insert(topic);
    }

    

    /* update()
     * Write the synchronized last set of 
     *
     */
    void update() {

      ros::Time t = ros::Time::now();

      if(verbosity > 0) {
        std::cout << t << ": writing from " << (hand_msgs.size() + arm_msgs.size()) << " topics." << std::endl;
      }

      for(const std::string &topic: discrete_topics) {
        if(hand_msgs.find(topic) != hand_msgs.end()) {
         bag.write(topic, t, hand_msgs[topic]);
        }
      }

      for(const std::string &topic: def_topics) {
        if(arm_msgs.find(topic) != arm_msgs.end()) {
          bag.write(topic, t, arm_msgs[topic]);
        }
      }



    }

    void spin() {
      while(ros::ok()) {
        ros::spinOnce();
        update();
        rate_.sleep();
      }
    }

    ~DemoWriter() {
      bag.close();
    }

  };


}

#endif
