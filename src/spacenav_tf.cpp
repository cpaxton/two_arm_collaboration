#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>


ros::Time t;
tf::Transform transform;

double ox;
double oy;
double oz;

void poseCallback(const geometry_msgs::TwistConstPtr& msg){
  static tf::TransformBroadcaster br;

  double x = 0, y = 0, z = 0;
  ros::Time t2 = ros::Time::now();
  ros::Duration d = t2 - t;
  t = t2;
  double dt = d.toSec();
  dt = 1;

  x = ox + (msg->linear.x / 500 / dt);
  y = oy + (msg->linear.y / 500 / dt);
  z = oz + (msg->linear.z / 500 / dt);

  transform.setOrigin( tf::Vector3(x, y, z) );
  tf::Quaternion q;
  double r = 0, p = 0, yaw = 0;
  //tf::Matrix3x3(transform.getRotation()).getRPY(r, p, y);

  if(isnan(r)) r = 0;
  if(isnan(p)) p = 0;
  if(isnan(yaw)) yaw = 0;

  r = r + (msg->angular.x / 500 / dt);
  p = p + (msg->angular.y / 500 / dt);
  yaw = yaw + (msg->angular.z / 500 / dt);

  q.setRPY(r, p, yaw);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "wam/cmd"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "spacenav_tf_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("spacenav/twist", 10, &poseCallback);

  t = ros::Time::now();

  ox = 0.8;
  oy = 0.46;
  oz = 1;

  transform.setOrigin( tf::Vector3(0, 0, 0));
  transform.setRotation( tf::Quaternion(0, 0, 0, 0));

  ros::spin();
  return 0;
};
