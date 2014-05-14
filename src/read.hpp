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

    int segment_id; // id of a segment to find
    std::string segment_name; // name of a segment to find

  public:

    DemoReader(const std::string &bagfile) :
      bag(bagfile, rosbag::bagmode::Read),
      nh(),
      nh_tilde("~"),
      segment_id(0),
      segment_name("")
    {

      nh_tilde.param("verbosity", verbosity, int(1));
      nh_tilde.param("segment_id", segment_id, int(0));
      nh_tilde.param("segment_name", segment_name, std::string(""));

      if(verbosity > 0) {
        ROS_INFO("Initialized Demonstration Reader with file=\"%s\"!", bagfile.c_str());
      }

      topics.push_back("/FEATURES"); // records feature positions
      topics.push_back("/SEGMENTS"); // records segment labels
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


          for(unsigned int i =  0; i < f->names.size(); ++i) {
            //std::cout << "adding data " << i << std::endl;
            fl.add_data("/world", f->names[i], f->transform[i]);
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

      if(verbosity > 0) {
        ROS_INFO("Starting feature registered replay!");
      }

      cv::Mat affine = cv::Mat(3, 4, CV_64FC1);
      cv::Point3f rec_avg;
      cv::Point3f obs_avg;
      for(const rosbag::MessageInstance &m: view) {

        if((features_loaded == false || topics_loaded < def_topics.size()) && m.getTime() != start) {
          ROS_INFO("Setting start time at %f seconds!", m.getTime().toSec());
          start = m.getTime();
          rec_avg.x = rec_avg.y = rec_avg.z = 0;
          obs_avg.x = obs_avg.y = obs_avg.z = 0;
        }

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
            topics_loaded = base_tfs.size();
            ROS_INFO("added topic %s (%u/%lu)", m.getTopic().c_str(), topics_loaded, def_topics.size());
          }

          if(features_loaded && topics_loaded == def_topics.size()) {
            ROS_INFO("Computing values");


            for(unsigned int i = 0; i < f->names.size(); ++i) {

              ROS_INFO("frame=%s", f->names[i].c_str());

              cv::Point3f pt;
              // NOTE: for some reason it looks like x and z were switched here
              pt.x = f->transform[i].translation.x;
              pt.y = f->transform[i].translation.y;
              pt.z = f->transform[i].translation.z;

              if(verbosity > 2) {
                ROS_INFO("RECOVERED FEATURES: %f %f %f", f->transform[i].translation.x,  f->transform[i].translation.y,  f->transform[i].translation.z);
              }

              rec_avg.x += pt.x;
              rec_avg.y += pt.y;
              rec_avg.z += pt.z;

              rec_pts.push_back(pt);
 
              finder_.wait(f->names[i], "/world", ros::Duration(1.0));
              geometry_msgs::Transform child_tf = finder_.find(f->names[i], "/world");

              cv::Point3f npt;
              npt.x = child_tf.translation.x;
              npt.y = child_tf.translation.y;
              npt.z = child_tf.translation.z;

              obs_avg.x += npt.x;
              obs_avg.y += npt.y;
              obs_avg.z += npt.z;

              obs_pts.push_back(npt);
              
              if(verbosity > 0) {
                ROS_INFO("demo: (%f, %f, %f) world: (%f %f %f)", pt.x, pt.y, pt.z, npt.x, npt.y, npt.z);
              }
            }

            rec_avg.x /= f->names.size();
            rec_avg.y /= f->names.size();
            rec_avg.z /= f->names.size();
            obs_avg.x /= f->names.size();
            obs_avg.y /= f->names.size();
            obs_avg.z /= f->names.size();

            ROS_INFO("demo avg: (%f, %f, %f) world avg: (%f, %f, %f)", rec_avg.x, rec_avg.y, rec_avg.z, obs_avg.x, obs_avg.y, obs_avg.z);

            
            for(unsigned int i = 0; i < rec_pts.size(); ++i) {
              rec_pts[i].x -= rec_avg.x;
              rec_pts[i].y -= rec_avg.y;
              rec_pts[i].z -= rec_avg.z;
              obs_pts[i].x -= obs_avg.x;
              obs_pts[i].y -= obs_avg.y;
              obs_pts[i].z -= obs_avg.z;
            }

            cv::Mat inliers;
            int status = cv::estimateAffine3D(rec_pts, obs_pts, affine, inliers, 10);
            cv::Mat extra_affine_row = cv::Mat::zeros(1, 4, CV_64FC1);
            extra_affine_row.at<double>(3) = 1;

            affine.push_back(extra_affine_row);

            if(verbosity > 0) {
              ROS_INFO("computed %dx%d affine transform with status=%d, type=%d",
                       affine.rows, affine.cols, status, affine.type());
              std::cout << "Transform = " << std::endl;
              for(int i = 0; i < 4; ++i) {
                for( int j = 0; j < 4; ++j) {
                  std::cout << "\t" << affine.at<double>(i, j);
                }
                std::cout << std::endl;
              }
            }
          }

        } else if(features_loaded == false && !(start == m.getTime())) {
          ROS_ERROR("Could not find features at time=%f!", start.toSec());
          exit(-1);
        } else if (topics_loaded != def_topics.size() && !(start == m.getTime())) {
          ROS_ERROR("Could not find all trajectory topics! Only found %u.", topics_loaded);
          exit(-1);
        } else {

          if(discrete_topics.find(m.getTopic()) != discrete_topics.end()) {
            publishers[m.getTopic()].publish(m.instantiate<discrete_msg_t>());
          } else if (def_topics.find(m.getTopic()) != def_topics.end()) {

            msg_ptr mptr = m.instantiate<msg_t>();

            cv::Mat cv_pt = cv::Mat::ones(1, 4, CV_64FC1);
            cv_pt.at<double>(0) = mptr->transform.translation.x - rec_avg.x;
            cv_pt.at<double>(1) = mptr->transform.translation.y - rec_avg.y;
            cv_pt.at<double>(2) = mptr->transform.translation.z - rec_avg.z;

            cv::Mat res = cv_pt * affine;
            ROS_INFO("(%f %f %f) mapped to (%f %f %f)", cv_pt.at<double>(0),
                cv_pt.at<double>(1), cv_pt.at<double>(2), 
                res.at<double>(0), res.at<double>(1), res.at<double>(2));

            msg_t pub_pt = *mptr;
          
            pub_pt.transform.translation.x = res.at<double>(0) + obs_avg.x;
            pub_pt.transform.translation.y = res.at<double>(1) + obs_avg.y;
            pub_pt.transform.translation.z = res.at<double>(2) + obs_avg.z;

            publishers[m.getTopic()].publish(pub_pt);
          }

          ros::spinOnce();
          ros::Duration wait((m.getTime() - cur).toSec() / rate);
          cur = m.getTime();
          time_spent += wait.toSec();
          if(verbosity > 0) {
            double percent = time_spent / (total_time / rate) * 100.0;
            if(verbosity > 1) {
              std::cout << "Registered Replay: " << percent << "% done, waiting " << wait.toSec() << " seconds" << std::endl;
            }
          }
          wait.sleep();
        }
      }
    }

    void write_feature_topic(std::string &topic) {

      rosbag::View view(bag, rosbag::TopicQuery(topics));

      ros::Time cur = view.getBeginTime();
      double total_time = ros::Duration(view.getEndTime() - view.getBeginTime()).toSec();
      double time_spent = 0;
      for(const rosbag::MessageInstance &m: view) {

        ros::spinOnce();
        ros::Duration wait((m.getTime() - cur).toSec());
        cur = m.getTime();
        time_spent += wait.toSec();
        if(verbosity > 0) {
          double percent = time_spent / total_time * 100.0;
          std::cout << "Replay: " << percent << "% done, waiting " << wait.toSec() << " seconds" << std::endl;
        }
        wait.sleep();
      }
    }

    void write_discrete_topic(std::string &topic) {

      rosbag::View view(bag, rosbag::TopicQuery(topics));

      ros::Time cur = view.getBeginTime();
      double total_time = ros::Duration(view.getEndTime() - view.getBeginTime()).toSec();
      double time_spent = 0;
      for(const rosbag::MessageInstance &m: view) {

        ros::spinOnce();
        ros::Duration wait((m.getTime() - cur).toSec());
        cur = m.getTime();
        time_spent += wait.toSec();
        if(verbosity > 0) {
          double percent = time_spent / total_time * 100.0;
          std::cout << "Replay: " << percent << "% done, waiting " << wait.toSec() << " seconds" << std::endl;
        }
        wait.sleep();
      }
    }

    void write_trajectory_topic(std::string &topic) {

      rosbag::View view(bag, rosbag::TopicQuery(topics));

      ros::Time cur = view.getBeginTime();
      double total_time = ros::Duration(view.getEndTime() - view.getBeginTime()).toSec();
      double time_spent = 0;
      for(const rosbag::MessageInstance &m: view) {

        ros::spinOnce();
        ros::Duration wait((m.getTime() - cur).toSec());
        cur = m.getTime();
        time_spent += wait.toSec();
        if(verbosity > 0) {
          double percent = time_spent / total_time * 100.0;
          std::cout << "Replay: " << percent << "% done, waiting " << wait.toSec() << " seconds" << std::endl;
        }
        wait.sleep();
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
          double percent = time_spent / (total_time / rate) * 100.0;
          std::cout << "Replay: " << percent << "% done, waiting " << wait.toSec() << " seconds" << std::endl;
        }
        wait.sleep();
      }
    }
  };

}

#endif
