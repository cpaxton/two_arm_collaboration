#ifndef LCSR_REPLAY_RECORD
#define LCSR_REPLAY_RECORD

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <oro_barrett_msgs/BHandCmd.h>

namespace lcsr_replay {

  class DemoWriter {
  private:
    int verbosity;

  protected:
    std::vector<std::string> topics;

    ros::NodeHandle nh; // default namespace
    ros::NodeHandle nh_tilde; //private namespace for parameters


    rosbag::Bag bag;

  public:

    DemoWriter(const std::string &bagfile) : bag(bagfile, rosbag::bagmode::Write), nh(), nh_tilde("~") {

      nh_tilde.param("verbosity", verbosity, int(1));

      if(verbosity > 0) {
        ROS_INFO("Initialized Demonstration Writer with file=\"%s\"!", bagfile.c_str());
      }
    }

  };

}

#endif
