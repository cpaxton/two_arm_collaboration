#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

ros::Subscriber sub;

int use_both;
int current_arm;

ros::Time t;
tf::Transform transform1;
tf::Transform transform2;

double ox;
double oy;
double oz;

std::string topic_name;
std::string other_topic_name;

void poseCallback(const geometry_msgs::TwistConstPtr& msg){
  static tf::TransformBroadcaster br;

  ROS_INFO("callback received command");

  double x = 0, y = 0, z = 0;
  ros::Time t2 = ros::Time::now();
  ros::Duration d = t2 - t;
  t = t2;
  double dt = 1 + d.toSec();

  tf::Transform transform;

  if(current_arm==0 || !use_both) {
    transform = transform1;
  } else {
    transform = transform2;
  }

  x = transform.getOrigin().getX();
  y = transform.getOrigin().getY();
  z = transform.getOrigin().getZ();

  x = x + (msg->linear.x / 100000 / dt);
  y = y + (msg->linear.y / 100000 / dt);
  z = z + (msg->linear.z / 100000 / dt);

  transform.setOrigin( tf::Vector3(x, y, z) );
  tf::Quaternion q;
  double r = 0, p = 0, yaw = 0;
  tf::Matrix3x3(transform.getRotation()).getRPY(r, p, y);

  if(isnan(r)) r = 0;
  if(isnan(p)) p = 0;
  if(isnan(yaw)) yaw = 0;

  r = r + (msg->angular.x / 100000 / dt);
  p = p + (msg->angular.y / 100000 / dt);
  yaw = yaw + (msg->angular.z / 100000 / dt);

  q.setRPY(r, p, yaw);
  transform.setRotation(q);

  if(current_arm==0 || !use_both) {
    transform1 = transform;
  } else {
    transform2 = transform;
  }

  br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "world", topic_name.c_str()));
  if(use_both) {
    br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "world", other_topic_name.c_str()));
  }
  ROS_INFO("sending message from world to %s", topic_name.c_str());
}

int main(int argc, char** argv){
  ros::init(argc, argv, "spacenav_tf_broadcaster");

  ROS_INFO("starting...");

  topic_name = "/wam/cmd";
  current_arm = 0;

  ros::NodeHandle node;
  sub = node.subscribe("spacenav/twist", 10, &poseCallback);
  ros::NodeHandle nh("~");
  nh.param("cmd_topic", topic_name, std::string("wam/cmd"));
  nh.param("cmd_topic2", other_topic_name, std::string("wam2/cmd"));
  nh.param("use_two_arms", use_both, int(0));

  ROS_INFO("initialized.");

  t = ros::Time::now();

  ox = 0.8;
  oy = 0.46;
  oz = 1;

  transform1.setOrigin( tf::Vector3(ox, oy, oz));
  transform2.setOrigin( tf::Vector3(ox, -1*oy, oz));

  tf::Quaternion q;
  q.setRPY(M_PI, -1.*M_PI/2.,0);
  transform1.setRotation( q );
  transform2.setRotation( q );

  ros::spin();
  return 0;
};
