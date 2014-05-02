#ifndef _LCSR_REPLAY
#define _LCSR_REPLAY

// STL includes
#include <vector>
#include <string>
#include <map>

// ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>

// LCSR includes
#include <oro_barrett_msgs/BHandCmd.h>

namespace lcsr_replay {

  /**
   * DemoReader
   * Read a demonstration from ROSBAG file.
   * Extract features.
   * */
  template <discrete_msg_t>
  class DemoReader : rosbag::Bag {
  protected:
    
    // record certain things which we just send as-is at times, we do not interpret or warp them
    std::map<std::string, discrete_msg_t> discrete_topics;

  public:

    DemoReader(const std::string &bagfile) : rosbag::Bag(bagfile) {

    }

    /* addDiscreteTopic()
     * Add a discrete topic.
     * For these, we look at what messages get sent and when.
     * We repeat them when the right conditions are met.
     */
    addDiscreteTopic(const std::string &topic);

    /* addTopic()
     * Add a continuous/inverse kinematics command topic.
     * These are interpreted according to feature counts.
     */
    addTopic(const std::string &topic);
    
  }




}


#endif
