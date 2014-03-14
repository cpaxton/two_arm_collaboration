#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>


void poseCallback(const geometry_msgs::TwistConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->linear.x, msg->linear.y, msg->linear.z) );
  tf::Quaternion q;
  q.setRPY(msg->angular.x, msg->angular.y, msg->angular.z);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "goal_frame"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "spacenav_tf_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("spacenav/twist", 10, &poseCallback);

  ros::spin();
  return 0;
};
