#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

ros::Subscriber tf1_sub;
ros::Subscriber tf2_sub;

int use_both;

void tf1Callback(const geometry_msgs::TransformStampedPtr& msg){
  static tf::TransformBroadcaster br;
  tf::StampedTransform transform1;
  tf::transformStampedMsgToTF(*msg, transform1);

  br.sendTransform(transform1);
}

int main(int argc, char** argv){

  std::string topic_name, other_topic_name; 

  ros::init(argc, argv, "spacenav_tf_broadcaster");

  ROS_INFO("starting tf republisher node...");

  ros::NodeHandle node;
  ros::NodeHandle nh("~");
  nh.param("cmd_topic", topic_name, std::string("wam/cmd"));
  nh.param("cmd_topic2", other_topic_name, std::string("wam2/cmd"));
  nh.param("use_two_arms", use_both, int(0));

  tf1_sub = node.subscribe(topic_name.c_str(), 10, &tf1Callback);
  tf2_sub = node.subscribe(other_topic_name.c_str(), 10, &tf1Callback);

  ros::spin();
  return 0;
};
