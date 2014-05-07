#ifndef _LCSR_REPLAY_READ
#define _LCSR_REPLAY_READ

// STL includes
#include <vector>
#include <string>
#include <map>
#include <iostream>

// ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// message includes
#include <oro_barrett_msgs/BHandCmd.h>
#include <geometry_msgs/TransformStamped.h>
#include <lcsr_replay/Features.h>

// replay includes
#include "features.hpp"
#include "transform.hpp"

//opencv includes (for point registration)
#include <opencv2/opencv.hpp>

namespace lcsr_replay {

  /* typedef to avoid worrying about templates just yet */
  typedef oro_barrett_msgs::BHandCmd discrete_msg_t;
  typedef geometry_msgs::TransformStamped msg_t;
  typedef geometry_msgs::TransformStampedConstPtr msg_ptr;

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

    FeaturesLookup features_;
    DemoTransformFinder finder_;
    
    std::vector<std::string> topics; // list of all topics

    ros::NodeHandle nh; // default namespace for publishers
    ros::NodeHandle nh_tilde; // private namespace

    // publishers to replay content
    std::map<std::string, ros::Publisher> publishers;
    std::set<std::string> discrete_topics; // discrete topics
    std::set<std::string> def_topics; // deformable topics

    rosbag::Bag bag;

  public:

    DemoReader(const std::string &bagfile) : bag(bagfile, rosbag::bagmode::Read), nh(), nh_tilde("~") {

      nh_tilde.param("verbosity", verbosity, int(1));

      if(verbosity > 0) {
        ROS_INFO("Initialized Demonstration Reader with file=\"%s\"!", bagfile.c_str());
      }

      topics.push_back("/FEATURES");
    }

    /* addDiscreteTopic()
     * Add a discrete topic.
     * For these, we look at what messages get sent and when.
     * We repeat them when the right conditions are met.
     * 
     * FUTURE PLANS: template function that adds a ROS publisher?
     */
    //template<class T = discrete_msg_t>
    void addDiscreteTopic(const std::string &topic, int rate = DEFAULT_RATE) {
      topics.push_back(topic);
      discrete_topics.insert(topic);

      ros::Publisher pub = nh.advertise<discrete_msg_t>(topic, rate);
      publishers[topic] = pub;

      if(verbosity > 0) {
        ROS_INFO("Adding discrete topic %s, publishing at rate %d", topic.c_str(), rate);
      }
    }



    /* addTopic()
     * Add a continuous/inverse kinematics command topic.
     * These are interpreted according to feature counts.
     */
    void addTopic(const std::string &topic, int rate = DEFAULT_RATE) {
      topics.push_back(topic);
      def_topics.insert(topic);
      
      ros::Publisher pub = nh.advertise<msg_t>(topic, rate);
      publishers[topic] = pub; 

      if(verbosity > 0) {
        ROS_INFO("Adding topic %s, publishing at rate %d", topic.c_str(), rate);
      }
    }

    /* show()
     * Read the contents of a bag so that we know what happened at each point in time in the demonstration.
     * Also read contents of the bag and show them as features
     */
    void show() {

      rosbag::View view(bag, rosbag::TopicQuery(topics));

      for(const rosbag::MessageInstance &m: view) {
        std::cout << m.getDataType() << " " << m.getTopic() << " at t=" << m.getTime().toSec() << std::endl;
        if(m.getTopic() == "/FEATURES") {
          Features::ConstPtr f = m.instantiate<Features>();
          FeaturesLookup fl;


          for(unsigned int i =  0; i < f->base.size(); ++i) {
            //std::cout << "adding data " << i << std::endl;
            fl.add_data(f->base[i], f->child[i], f->transform[i]);
          }

          fl.print();
        }
      }
    }

    /** registered replay
     * Replay, but alter the transforms before publishing using PCL
     */
    void registered_replay(double rate = 1.0) {
      
      rosbag::View view(bag, rosbag::TopicQuery(topics));

      ros::Time start = view.getBeginTime();
      ros::Time cur = view.getBeginTime();
      double total_time = ros::Duration(view.getEndTime() - view.getBeginTime()).toSec();
      double time_spent = 0;

      unsigned int topics_loaded = 0;
      bool start_message = false; // have we started yet?
      bool features_loaded = false; // have we gotten the features yet?
      FeaturesConstPtr f; // starting features

      std::vector<cv::Point3f> obs_pts;
      std::vector<cv::Point3f> rec_pts;

      std::map<std::string, msg_ptr> base_tfs;

      for(const rosbag::MessageInstance &m: view) {

        // compute registration
        if(m.getTime() == start) {

          if(verbosity > 0 && start_message == false) {
            ROS_INFO("Starting to compute registration between known correspondences...");
            start_message = true;
          }

          // first let's look at what all the features are
          if(m.getTopic() == "/FEATURES") {
            features_loaded = true;
            f = m.instantiate<Features>();

          }
          else if (def_topics.find(m.getTopic()) != def_topics.end()) {
            base_tfs[m.getTopic()] = m.instantiate<msg_t>();
          }

          if(features_loaded && topics_loaded == def_topics.size()) {
            ROS_INFO("Computing values");

            for(unsigned int i = 0; i < f->base.size(); ++i) {
              cv::Point3f pt;
              pt.x = base_tfs[f->base[i]]->transform.translation.x + f->transform[i].translation.x;
              pt.y = base_tfs[f->base[i]]->transform.translation.y + f->transform[i].translation.y;
              pt.z = base_tfs[f->base[i]]->transform.translation.z + f->transform[i].translation.z;

              rec_pts.push_back(pt);

              ROS_INFO("Adding reference point at (%f, %f, %f)", pt.x, pt.y, pt.z);
            }
          }


        } else {

          if(features_loaded == false) {
            ROS_ERROR("Could not find features at time=%f!", start.toSec());
            exit(-1);
          }

          if(discrete_topics.find(m.getTopic()) != discrete_topics.end()) {
            publishers[m.getTopic()].publish(m.instantiate<discrete_msg_t>());
          } else if (def_topics.find(m.getTopic()) != def_topics.end()) {
            publishers[m.getTopic()].publish(m.instantiate<msg_t>());
          }

          ros::spinOnce();
          ros::Duration wait((m.getTime() - cur).toSec() / rate);
          cur = m.getTime();
          time_spent += wait.toSec();
          if(verbosity > 0) {
            double percent = time_spent / total_time * 100.0;
            std::cout << "Replay: " << percent << "% done, waiting " << wait.toSec() << " seconds" << std::endl;
          }
          wait.sleep();
        }
      }
    }

    /* replay()
     * Publish all commands on their old ROS topics with the same timing
     */
    void replay(double rate = 1.0) {

      rosbag::View view(bag, rosbag::TopicQuery(topics));

      ros::Time cur = view.getBeginTime();
      double total_time = ros::Duration(view.getEndTime() - view.getBeginTime()).toSec();
      double time_spent = 0;
      for(const rosbag::MessageInstance &m: view) {

        if(discrete_topics.find(m.getTopic()) != discrete_topics.end()) {
          publishers[m.getTopic()].publish(m.instantiate<discrete_msg_t>());
        } else if (def_topics.find(m.getTopic()) != def_topics.end()) {
          publishers[m.getTopic()].publish(m.instantiate<msg_t>());
        }

        ros::spinOnce();
        ros::Duration wait((m.getTime() - cur).toSec() / rate);
        cur = m.getTime();
        time_spent += wait.toSec();
        if(verbosity > 0) {
          double percent = time_spent / total_time * 100.0;
          std::cout << "Replay: " << percent << "% done, waiting " << wait.toSec() << " seconds" << std::endl;
        }
        wait.sleep();
      }
    }
  };

}

#endif
