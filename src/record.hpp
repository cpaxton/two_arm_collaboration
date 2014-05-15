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
#include <lcsr_replay/Segment.h>

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
    std::map<std::string, discrete_msg_t> old_hand_msgs;

    // set of all transform messages 
    std::map<std::string, msg_t> arm_msgs;
    std::map<std::string, msg_t> old_arm_msgs;

    // what was the last arm topic that changed?
    // used for segment labeling
    std::string last_arm_topic;

    std::set<std::string> discrete_topics;
    std::set<std::string> def_topics;

    std::vector<std::string> frames;

    int current_segment; // what is the label of the current segment?
    bool label_segments; // should we automatically label segments based on when things are happening?

    std::string last_arm; // last arm topic to change (only if doing automatic labelling)

    // are we receiving the same hand positions?
    static inline bool equal_discrete_msgs(const discrete_msg_t &msg1, const discrete_msg_t &msg2) {
      return (msg1.cmd[0] == msg2.cmd[0]
          && msg1.cmd[1] == msg2.cmd[1]
          && msg1.cmd[2] == msg2.cmd[2]
          && msg1.cmd[3] == msg2.cmd[3]
          && msg1.mode[0] == msg2.mode[0]
          && msg1.mode[1] == msg2.mode[1]
          && msg1.mode[2] == msg2.mode[2]
          && msg1.mode[3] == msg2.mode[3]);
    }

    /**
     * Is a transform the same or did it change?
     */
    static inline bool equal_msg(const msg_t &msg1, const msg_t &msg2) {
      return (msg1.transform.translation.x == msg2.transform.translation.x
              && msg1.transform.translation.y == msg2.transform.translation.y
              && msg1.transform.translation.z == msg2.transform.translation.z
              && msg1.transform.rotation.x == msg2.transform.rotation.x
              && msg1.transform.rotation.y == msg2.transform.rotation.y
              && msg1.transform.rotation.z == msg2.transform.rotation.z);
    }



  public:

    void discrete_callback(const discrete_msg_ptr &msg, const std::string &topic) {
      if(hand_msgs.find(topic) != hand_msgs.end()) {
        old_hand_msgs[topic] = hand_msgs[topic];
      }
      hand_msgs[topic] = *msg;
    }

    DemoWriter(const std::string &bagfile, ros::Rate rate = ros::Rate(DEFAULT_RATE))
      : bag(bagfile, rosbag::bagmode::Write),
      finder(),
      rate_(rate),
      nh(),
      nh_tilde("~"),
      current_segment(0), // first segment is segment 1, starts when first arm moves
      label_segments(false)
    {

      nh_tilde.param("verbosity", verbosity, int(1));
      int label;
      nh_tilde.param("label_segments", label, int(1));
      label_segments = label == 1;
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

    /**
     * callback for the trajectory controlling messages
     */
    void def_callback(const msg_ptr &msg, const std::string &topic) {
      if(arm_msgs.find(topic) != arm_msgs.end()) {
        old_arm_msgs[topic] = arm_msgs[topic];
      }
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

      bool switched_topic = false;

      for(const std::string &topic: discrete_topics) {
        if(hand_msgs.find(topic) != hand_msgs.end()) {
         bag.write(topic, t, hand_msgs[topic]);
        }

        // if we are labelling the segments, see if the hands changed
        if(label_segments
           && !switched_topic
           && old_hand_msgs.find(topic) != old_hand_msgs.end()
           && !equal_discrete_msgs(hand_msgs[topic], old_hand_msgs[topic]))
        {
          ++current_segment;
          switched_topic = true;
          last_arm = ""; // make it so we start a new segment as soon as any arm moves again
          if(verbosity > 2) {
            ROS_INFO("current segment = %d", current_segment);
          }
        }
      }

      for(const std::string &topic: def_topics) {
        if(arm_msgs.find(topic) != arm_msgs.end()) {
          bag.write(topic, t, arm_msgs[topic]);
        }

        // if not that, check to see if there were any changes in the position described
        if(label_segments
           && !switched_topic
           && old_arm_msgs.find(topic) != old_arm_msgs.end()
           && !equal_msg(arm_msgs[topic], old_arm_msgs[topic])
           && last_arm != topic)
        {
          last_arm = topic;
          switched_topic = true;
          ++current_segment;
          if(verbosity > 2) {
            ROS_INFO("arm update: current segment = %d, topic=%s", current_segment, last_arm.c_str());
          }
        }
      }

      Features f;

      // loop over frame pairs (features)
      for(unsigned int i = 0; i < frames.size(); ++i) {
        geometry_msgs::Transform t = finder.find(frames[i], "/world");
        f.names.push_back(frames[i]);
        f.transform.push_back(t);

        if(verbosity > 1) {
          ROS_INFO("%s-->%s transform found", frames[i].c_str(), "/world");
        }
      }

      bag.write("/FEATURES", t, f);

      // write the segement labels from what we were doing here
      if(label_segments) {
        Segment s;
        s.name = "";
        s.num = current_segment;
        bag.write("/SEGMENT", t, s);
      }
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
