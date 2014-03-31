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
  double dt = 1 + d.toSec();

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

  transform.setOrigin( tf::Vector3(ox, oy, oz));
  tf::Quaternion q;
  q.setRPY(M_PI, -1.*M_PI/2.,0);
  transform.setRotation( q );

  ros::spin();
  return 0;
};
