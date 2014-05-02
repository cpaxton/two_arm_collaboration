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

namespace lcsr_replay {

  /* typedef to avoid worrying about templates just yet */
  typedef oro_barrett_msgs::BHandCmd discrete_msg_t;
  typedef geometry_msgs::TransformStamped msg_t;

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
     */
    void show() {

      rosbag::View view(bag, rosbag::TopicQuery(topics));

      for(const rosbag::MessageInstance &m: view) {
        std::cout << m.getDataType() << " " << m.getTopic() << " at t=" << m.getTime().toSec() << std::endl;
      }
    }

    /* replay()
     * Publish all commands on their old ROS topics with the same timing
     */
    void replay() {

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
        ros::Duration wait = m.getTime() - cur;
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
