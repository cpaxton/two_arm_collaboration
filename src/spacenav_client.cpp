#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <oro_barrett_msgs/BHandCmd.h>

ros::Subscriber sub;
ros::Subscriber joy_sub;

ros::Publisher arm1_pub; // publish position messages to arm1's gripper
ros::Publisher arm2_pub; // publish position messages to arm2's gripper
ros::Publisher tf1_pub;
ros::Publisher tf2_pub;

int use_both;
int current_arm;

ros::Time t;
tf::Transform transform1;
tf::Transform transform2;

double ox;
double oy;
double oz;

double closed_position;

std::string topic_name;
std::string other_topic_name;

bool arm1_closed;
bool arm2_closed;
bool reset;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  if(reset) {
    if(joy->buttons[0]) {
      current_arm = !current_arm;
      reset = false;
    }
    if(!current_arm && joy->buttons[1]) {
      arm1_closed = !arm1_closed;
      reset = false;
    } else if (current_arm && joy->buttons[1]) {
      arm2_closed = !arm2_closed;
      reset = false;
    }
  }
  if(!joy->buttons[0] && !joy->buttons[1]) {
    reset = true;
  }

  oro_barrett_msgs::BHandCmd cmd1;
  oro_barrett_msgs::BHandCmd cmd2;
  for(int i = 0; i < 4; ++i) {
    cmd1.mode[i] = 3;
    cmd2.mode[i] = 3;
  }
  if(arm1_closed) {
    cmd1.cmd[0] = closed_position;
    cmd1.cmd[1] = closed_position;
    cmd1.cmd[2] = closed_position;
    cmd1.cmd[3] = 0;
  } else {
    cmd1.cmd[0] = 0.5;
    cmd1.cmd[1] = 0.5;
    cmd1.cmd[2] = 0.5;
    cmd1.cmd[3] = 0;
  }

  if(arm2_closed) {
    cmd2.cmd[0] = closed_position;
    cmd2.cmd[1] = closed_position;
    cmd2.cmd[2] = closed_position;
    cmd2.cmd[3] = 0;
  } else {
    cmd2.cmd[0] = 0.5;
    cmd2.cmd[1] = 0.5;
    cmd2.cmd[2] = 0.5;
    cmd2.cmd[3] = 0;
  }

  arm1_pub.publish(cmd1);
  arm2_pub.publish(cmd2);
}

double t1r;
double t2r;
double t1p;
double t2p;
double t1yaw;
double t2yaw;

void poseCallback(const geometry_msgs::TwistConstPtr& msg){

  //ROS_INFO("callback received command");

  double x = 0, y = 0, z = 0;
  ros::Time t2 = ros::Time::now();
  ros::Duration d = t2 - t;
  t = t2;
  double dt = 1 + d.toSec();

  tf::Transform transform;

  double r = 0, p = 0, yaw = 0;
  if(current_arm==0 || !use_both) {
    transform = transform1;
    r = t1r;
    p = t1p;
    yaw = t1yaw;
  } else {
    transform = transform2;
    r = t2r;
    p = t2p;
    yaw = t2yaw;
  }

  x = transform.getOrigin().getX();
  y = transform.getOrigin().getY();
  z = transform.getOrigin().getZ();


  if(fabs(msg->linear.x) > 100) x = x + (msg->linear.x / 100000 / dt);
  if(fabs(msg->linear.y) > 100) y = y + (msg->linear.y / 100000 / dt);
  if(fabs(msg->linear.z) > 100) z = z + (msg->linear.z / 100000 / dt);

  tf::Quaternion q;
  //tf::Matrix3x3(transform.getRotation()).getRPY(r, p, yaw);

  if(isnan(r)) r = 0;
  if(isnan(p)) p = 0;
  if(isnan(yaw)) yaw = 0;

  if(fabs(msg->angular.x) > 100) r = r + (msg->angular.x / 30000 / dt);
  if(fabs(msg->angular.y) > 100) p = p + (msg->angular.y / 30000 / dt);
  if(fabs(msg->angular.z) > 100) yaw = yaw + (msg->angular.z / 30000 / dt);

  q.setRPY(r, p, yaw);
  transform.setRotation(q);
  transform.setOrigin( tf::Vector3(x, y, z) );

  if(current_arm==0 || !use_both) {
    transform1 = transform;
    t1r = r;
    t1p = p;
    t1yaw = yaw;
  } else {
    transform2 = transform;
    t2r = r;
    t2p = p;
    t2yaw = yaw;
  }

  tf::StampedTransform stf1(transform1, ros::Time::now(), "world", topic_name.c_str());
  geometry_msgs::TransformStamped stf1_msg;
  tf::transformStampedTFToMsg(stf1, stf1_msg);
  tf1_pub.publish(stf1_msg);
  if(use_both) {
    tf::StampedTransform stf2(transform2, ros::Time::now(), "world", other_topic_name.c_str());
    geometry_msgs::TransformStamped stf2_msg;
    tf::transformStampedTFToMsg(stf2, stf2_msg);
    tf2_pub.publish(stf2_msg);
  }
  //ROS_INFO("sending message from world to %s", topic_name.c_str());
  //ROS_INFO("pos = (%f %f %f)", x, y, z);
  //ROS_INFO("rot = (%f %f %f)", r, p, yaw);
}

int main(int argc, char** argv){

  reset = false;
  arm1_closed = arm2_closed = false;

  std::string arm_topic_name, arm2_topic_name; // names of the topics to publish messages on to close/open grippers

  ros::init(argc, argv, "spacenav_client_node");

  ROS_INFO("starting spacenav_client...");

  topic_name = "/wam/cmd";
  current_arm = 0;

  ros::NodeHandle node;
  joy_sub = node.subscribe("spacenav/joy", 10, &joyCallback);
  sub = node.subscribe("spacenav/twist", 10, &poseCallback);
  ros::NodeHandle nh("~");
  nh.param("cmd_topic", topic_name, std::string("wam/cmd"));
  nh.param("cmd_topic2", other_topic_name, std::string("wam2/cmd"));
  nh.param("arm_topic", arm_topic_name, std::string("gazebo/barrett_manager/hand/cmd"));
  nh.param("arm2_topic", arm2_topic_name, std::string("gazebo/w2barrett_manager/hand/cmd"));
  nh.param("use_two_arms", use_both, int(0));
  nh.param("closed_position", closed_position, double(2));

  arm1_pub = node.advertise<oro_barrett_msgs::BHandCmd>(arm_topic_name, 10); 
  arm2_pub = node.advertise<oro_barrett_msgs::BHandCmd>(arm2_topic_name, 10); 
  tf1_pub = node.advertise<geometry_msgs::TransformStamped>(topic_name, 60);
  tf2_pub = node.advertise<geometry_msgs::TransformStamped>(other_topic_name, 60);
  

  //ROS_INFO("initialized.");

  t = ros::Time::now();

  ox = 0.8;
  oy = 0.46;
  oz = 1;

  double roll = M_PI;
  double pitch = -1.*M_PI/2.;
  double yaw = 0;

  t1r = roll;
  t2r = roll;
  t1p = pitch;
  t2p = pitch;
  t1yaw = yaw;
  t2yaw = yaw;

  transform1.setOrigin( tf::Vector3(ox, oy, oz));
  transform2.setOrigin( tf::Vector3(ox, -1*oy, oz));

  tf::Quaternion q;
  q.setRPY(M_PI, -1.*M_PI/2.,0);
  transform1.setRotation( q );
  transform2.setRotation( q );

  ros::spin();
  return 0;
};
