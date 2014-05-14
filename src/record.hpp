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
#include <lcsr_replay/Features.h>

namespace lcsr_replay {

  /* typedef to avoid worrying about templates just yet */
  typedef oro_barrett_msgs::BHandCmd discrete_msg_t;
  typedef oro_barrett_msgs::BHandCmdConstPtr discrete_msg_ptr;
  typedef geometry_msgs::TransformStamped msg_t;
  typedef geometry_msgs::TransformStampedConstPtr msg_ptr;


  class DemoWriter {
  private:
    int verbosity;
    int wait_for_transforms;
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

    // what was the last arm topic that changed?
    // used for segment labeling
    std::string last_arm_topic;

    std::set<std::string> discrete_topics;
    std::set<std::string> def_topics;

    std::vector<std::string> frames;

    int current_segment; // what is the label of the current segment?
    bool label_segments; // should we automatically label segments based on when things are happening?

  public:

    void discrete_callback(const discrete_msg_ptr &msg, const std::string &topic) {
      hand_msgs[topic] = *msg;
    }

    DemoWriter(const std::string &bagfile, ros::Rate rate = ros::Rate(DEFAULT_RATE))
      : bag(bagfile, rosbag::bagmode::Write),
      finder(),
      rate_(rate),
      nh(),
      nh_tilde("~"),
      current_segment(1)
    {

      nh_tilde.param("verbosity", verbosity, int(1));
      nh_tilde.param("wait_for_transforms", wait_for_transforms, int(1));

      XmlRpc::XmlRpcValue base_list;
      XmlRpc::XmlRpcValue frame_list;

      if(nh_tilde.hasParam("frames")) {
        nh_tilde.param("frames", frame_list, frame_list);
        for (int i = 0; i < frame_list.size(); ++i) {
          if(frame_list[i].getType() == XmlRpc::XmlRpcValue::TypeString)
          {
            std::string c = static_cast<std::string>(frame_list[i]);
            ROS_INFO("Feature: transform to %s", c.c_str());
            frames.push_back(c);
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

      Features f;

      // loop over frame pairs (features)
      for(unsigned int i = 0; i < frames.size(); ++i) {
        geometry_msgs::Transform t = finder.find(frames[i], "/world");
        f.names.push_back(frames[i]);
        f.transform.push_back(t);

        ROS_INFO("%s-->%s transform found", frames[i].c_str(), "/world");
      }

      bag.write("/FEATURES", t, f);
    }


    void spin() {

      if(wait_for_transforms) {
        // make sure we have transforms for all of these things
        for(unsigned int i = 0; i < frames.size(); ++i) {
          finder.wait(frames[i], "/world", ros::Duration(1.0));
          // NOTE: default to using world as a reference frame here
        }
      }

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
