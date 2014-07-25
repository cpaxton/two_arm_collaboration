#ifndef _LCSR_REPLAY_FEATURES
#define _LCSR_REPLAY_FEATURES

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <utility>
#include <map>
#include <iostream>
#include <sstream>
#include <math.h>

using geometry_msgs::Transform;

namespace collab_replay {

  class FeaturesLookup {
    private:
      std::map <std::string, double> data_;
      
    protected:
      std::string item_name(std::string base_frame, std::string child_frame, std::string name = "") {
        std::stringstream ss;
        ss << base_frame << "_" << child_frame << "_" << name;
        return ss.str();
      }

    public:
      void add_data(std::string base_frame,
                    std::string child_frame,
                    Transform tf)
      {

        double x, y, z;
        double dist;

        x = tf.translation.x;
        y = tf.translation.y;
        z = tf.translation.z;

        dist = sqrt((x*x) + (y*y) + (z*z));

        data_[item_name(base_frame, child_frame, "dist")] = dist;
        data_[item_name(base_frame, child_frame, "dx")] = x;
        data_[item_name(base_frame, child_frame, "dy")] = y;
        data_[item_name(base_frame, child_frame, "dz")] = z;
      }

      void print() {
        for (typename std::map<std::string, double>::const_iterator it = data_.begin();
             it != data_.end();
             ++it)
        {
          std::cout << it->first << "=" << it->second << std::endl;
        }
      }

  };
}

#endif
