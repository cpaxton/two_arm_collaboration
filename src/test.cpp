#include <ros/ros.h>

#include "read.hpp"

using namespace lcsr_replay;

int main(int argc, char **argv) {

  ros::init(argc, argv, "lcsr_replay_test");
  ros::NodeHandle nh("~");

  // define topics
  std::string bagfile;
  std::string arm1_topic;
  std::string arm2_topic;
  std::string bhand1_topic;
  std::string bhand2_topic;

  ROS_INFO("Reading topics...");
  nh.param("bag",bagfile,std::string("demo.bag"));
  nh.param("bhand1_topic",bhand1_topic,std::string("/gazebo/barrett_manager/hand/cmd"));
  nh.param("bhand2_topic",bhand2_topic,std::string("/gazebo/w2barrett_manager/hand/cmd"));
  nh.param("arm1_topic", arm1_topic,std::string("/wam/cmd"));
  nh.param("arm2_topic",arm2_topic,std::string("/wam2/cmd"));


  DemoReader dr(bagfile);
  dr.addDiscreteTopic(bhand1_topic);
  dr.addDiscreteTopic(bhand2_topic);
  dr.addTopic(arm1_topic);
  dr.addTopic(arm2_topic);

  dr.examine();

}
