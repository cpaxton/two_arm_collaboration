#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <sensor_msgs/JointState.h>
#include <stdio.h>

namespace gazebo
{   
  class ModelJointStatePublisher : public ModelPlugin
  {

    ros::NodeHandle nh_; // ros node handle
    ros::Publisher pub_; // ros joint state publisher

    public: ModelJointStatePublisher() : ModelPlugin(), nh_("") {
      pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 50, true);
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
    {

      // Create the ROS topic publisher

      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelJointStatePublisher::OnUpdate, this, _1));
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {

      physics::Joint_V joints = this->model->GetJoints();

      // iterate over list of joints
      for(typename physics::Joint_V::const_iterator it = joints.begin(); it != joints.end(); ++it) {

      }
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelJointStatePublisher)
}
