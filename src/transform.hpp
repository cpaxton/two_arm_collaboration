#ifndef LCSR_REPLAY_TF
#define LCSR_REPLAY_TF

// tf includes
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// message includes
#include <geometry_msgs/Transform.h>

namespace lcsr_replay {

  class DemoTransformFinder {
  private:
    tf::TransformListener listener;

  public: 

    DemoTransformFinder() : listener() {
      // nothing here yet
    }

    void wait(const std::string &frame1, const std::string &frame2, ros::Duration time) {
      listener.waitForTransform(frame2, frame1, ros::Time::now(), time);
    }

    geometry_msgs::Transform find(const std::string &frame1, const std::string &frame2) {
      try{
        // read through pairs of frames, record transforms
        tf::StampedTransform t;
        geometry_msgs::TransformStamped gt;
        listener.lookupTransform(frame2, frame1, ros::Time(0), t);

        tf::transformStampedTFToMsg(t, gt);

        return gt.transform;

      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
      }
    }
  };

}

#endif
