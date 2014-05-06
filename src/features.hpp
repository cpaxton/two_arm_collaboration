#ifndef _LCSR_REPLAY_FEATURES
#define _LCSR_REPLAY_FEATURES

#include <ros/ros.h>
#include "geometry_msgs/Transform"
#include <pair>
#include <map>
#include <iostream>
#include <sstream>
#include <math.h>

using geometry_msgs::Transform;

namespace lcsr_replay {

  class FeaturesLookup {
    private:
      std::map <std::string, Transform> tf_data_;
      
    protected:
      std::string item_name(std::string item1, std::string item2, std::string name = "") {
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

        tf_data_[item_name(base_frame, child_frame, "dist")] = dist;
        tf_data_[item_name(base_frame, child_frame, "dx")] = x;
        tf_data_[item_name(base_frame, child_frame, "dy")] = y;
        tf_data_[item_name(base_frame, child_frame, "dz")] = z;
      }

      void print() {
        for (std::pair<std::string, Transform> d: tf_data_) {
          std::cout << d->first << ": " << std::endl;
        }
      }

  };
}

#endif
