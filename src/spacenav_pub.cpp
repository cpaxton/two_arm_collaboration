#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

ros::Subscriber tf1_sub;
ros::Subscriber tf2_sub;

int use_both;

bool recvd_tf1;
bool recvd_tf2;

tf::StampedTransform transform1;
tf::StampedTransform transform2;

void tf1Callback(const geometry_msgs::TransformStampedPtr& msg){
  static tf::TransformBroadcaster br;
  tf::transformStampedMsgToTF(*msg, transform1);

  recvd_tf1 = true;
  //br.sendTransform(transform1);
}

void tf2Callback(const geometry_msgs::TransformStampedPtr& msg){
  static tf::TransformBroadcaster br;
  tf::transformStampedMsgToTF(*msg, transform2);
  recvd_tf2 = true;
  //br.sendTransform(transform2);
}
int main(int argc, char** argv){

  std::string topic_name, other_topic_name; 
  recvd_tf1 = false;
  recvd_tf2 = false;

  ros::init(argc, argv, "spacenav_tf_broadcaster");

  ROS_INFO("starting tf republisher node...");

  ros::NodeHandle node;
  ros::NodeHandle nh("~");
  nh.param("cmd_topic", topic_name, std::string("wam/cmd"));
  nh.param("cmd_topic2", other_topic_name, std::string("wam2/cmd"));
  nh.param("use_two_arms", use_both, int(0));

  tf1_sub = node.subscribe(topic_name.c_str(), 60, &tf1Callback);
  if(use_both) {
    tf2_sub = node.subscribe(other_topic_name.c_str(), 60, &tf1Callback);
  }

  static tf::TransformBroadcaster br;
  ros::Rate r(60);

  while(ros::ok()) {
    ros::spinOnce();

    // ROS: wait to publish
    // send both the last received for arm 1 and the last for arm 2
    if(recvd_tf1) {
      transform1.stamp_ = ros::Time::now();
      br.sendTransform(transform1);
    }
    if(recvd_tf2) {
      transform2.stamp_ = ros::Time::now();
      br.sendTransform(transform2);
    }

    r.sleep();
  }
  return 0;
};
