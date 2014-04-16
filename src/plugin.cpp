#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <stdio.h>
#include <iostream>
#include <sstream>

namespace gazebo
{  

  /**
   * ModelJointStatePublisher
   * Publishes rotation/translation between different objects.
   * Also publishes objects as a TF tree.
   * Why? Because the normal controllers don't publish floating joints!
   * **/
  class ModelJointStatePublisher : public ModelPlugin
  {

    ros::NodeHandle nh_; // ros node handle
    ros::Publisher pub_; // ros joint state publisher

    std::string ns; // namespace to publish tf frames under
    std::string ref; // tf reference frame name (world frame)
    int verbosity; // amount of output to print

    public: ModelJointStatePublisher() : ModelPlugin(), nh_(""), verbosity(1) {

      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      } else {
        pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 50, true);
      }
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) 
    {


      // Create the ROS topic publisher

      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelJointStatePublisher::OnUpdate, this, _1));

      verbosity = _sdf->GetElement("verbosity")->Get<int>();
      ns = _sdf->GetElement("namespace")->Get<std::string>();
      ref = _sdf->GetElement("reference_frame")->Get<std::string>();

      if(ns.size() == 0) {
        ns = "observed";
      }
      if(ref.size() == 0) {
        ref = "/world";
      }

      if(verbosity > 0) {
        ROS_INFO("Publishing under namespace \"%s\"", ns.c_str());
        ROS_INFO("TF reference frame: %s", ref.c_str());
        ROS_INFO("Verbosity level: %d", verbosity);
      }
    }

    /* helper function to create names for TF */
    static inline std::string getNameTF(std::string ns, std::string joint) {
      std::stringstream ss;
      ss << ns << "/" << joint;
      return ss.str();
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      static tf::TransformBroadcaster br;

      physics::Joint_V joints = this->model->GetJoints();
      physics::Link_V links = this->model->GetLinks();

      if(verbosity > 2) {
        ROS_INFO("Reading %u joints and %u links", joints.size(), links.size());
      }

      // iterate over list of joints
      for(typename physics::Joint_V::const_iterator it = joints.begin(); it != joints.end(); ++it) {
        tf::Transform t;
        physics::JointPtr ptr = (*it);

        double r = ptr->GetAngle(0).Radian();
        double p = ptr->GetAngle(1).Radian();
        double y = ptr->GetAngle(2).Radian();
        
        physics::LinkPtr parent = ptr->GetParent();
        physics::LinkPtr child = ptr->GetChild();
       
        std::string pname = getNameTF(ns, parent->GetName());
        std::string cname = getNameTF(ns, child->GetName());

        ROS_INFO("%s %s %f %f %f", pname.c_str(), cname.c_str(), r, p, y);

        tf::Quaternion q;
        q.setRPY(r, p, y);
        t.setRotation(q);

        br.sendTransform(tf::StampedTransform(t, ros::Time::now(), pname.c_str(), cname.c_str()));
      }
      for(typename physics::Link_V::const_iterator it = links.begin(); it != links.end(); ++it) {
        tf::Transform t;
        physics::LinkPtr ptr = *it;

        ROS_INFO("%s", getNameTF(ns, (*it)->GetName()).c_str());
      }

      ros::spinOnce();
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelJointStatePublisher)
}
