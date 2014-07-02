#include <ros/ros.h>

#include "record.hpp"

using namespace lcsr_replay;

int main(int argc, char **argv) {

  ros::init(argc, argv, "lcsr_replay_recorder");
  ros::NodeHandle nh("~");

  // define topics
  std::string bagfile;
  std::string arm1_topic;
  std::string arm2_topic;
  std::string bhand1_topic;
  std::string bhand2_topic;
  int rate;

  ROS_INFO("Reading topics...");
  nh.param("bag",bagfile,std::string("demo.bag"));
  nh.param("bhand1_topic",bhand1_topic,std::string("/gazebo/barrett_manager/hand/cmd"));
  nh.param("bhand2_topic",bhand2_topic,std::string("/gazebo/w2barrett_manager/hand/cmd"));
  nh.param("arm1_topic", arm1_topic,std::string("/wam/cmd"));
  nh.param("arm2_topic",arm2_topic,std::string("/wam2/cmd"));
  nh.param("rate",rate,int(60));

  DemoWriter dw(bagfile, rate);
  dw.addTopic(arm1_topic);
  dw.addTopic(arm2_topic);
  dw.addDiscreteTopic(bhand1_topic);
  dw.addDiscreteTopic(bhand2_topic);

  dw.spin();
}

