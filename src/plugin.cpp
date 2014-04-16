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
#include <map>


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

    /**
     * Magnetic joint recorded between two components.
     * Joints must be stored here so they can be detached and gazebo will stop bothering with them.
     * **/
    struct magnetic_joint {
      std::string from;
      std::string to;
      bool latched;
    };


    ros::NodeHandle nh_; // ros node handle
    ros::Publisher pub_; // ros joint state publisher

    std::string ns; // namespace to publish tf frames under
    std::string ref; // tf reference frame name (world frame)
    int verbosity; // amount of output to print

    int enable_latch;
    double latch_strength;

    ros::Time last_update; // last time it was updated
    ros::Duration rate; // number of updates per second

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

    /* helper function to create names for TF */
    static inline std::string getNameTF(const std::string &ns, const std::string &joint) {
      std::stringstream ss;
      ss << ns << "/" << joint;
      return ss.str();
    }

    static inline tf::Transform poseToTF(const math::Pose &pose_) {
      tf::Transform model_tf_;
      model_tf_.setRotation( tf::Quaternion(pose_.rot.x,
                                          pose_.rot.y,
                                          pose_.rot.z,
                                          pose_.rot.w) );
      model_tf_.setOrigin( tf::Vector3(pose_.pos.x, pose_.pos.y, pose_.pos.z) );

      return model_tf_;
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

      enable_latch = _sdf->GetElement("enable_latch")->Get<int>();
      latch_strength = _sdf->GetElement("latch_strength")->Get<double>();
      verbosity = _sdf->GetElement("verbosity")->Get<int>();
      ns = _sdf->GetElement("namespace")->Get<std::string>();
      ref = _sdf->GetElement("reference_frame")->Get<std::string>();

      /* how often should we update the model? */
      double updates_per_second = _sdf->GetElement("updates_per_second")->Get<double>();
      if(updates_per_second > 0) {
        rate = ros::Duration(1.0 / updates_per_second);
      } else {
        rate = ros::Duration(0.1);
      }

      //if(ns.size() == 0) {
      //  ns = "observed";
      //}
      if(ref.size() == 0) {
        ref = "/world";
      }

      if(verbosity > 0) {
        ROS_INFO("Publishing under namespace \"%s\"", ns.c_str());
        ROS_INFO("TF reference frame: %s", ref.c_str());
        ROS_INFO("Verbosity level: %d", verbosity);
      }


      if(enable_latch) {
        const sdf::ElementPtr sdf_ = model->GetSDF();

        /* loop over sdf elements and look at floating joints */
        sdf::ElementPtr ptr = sdf_->GetElement("joint");
        while (ptr) {

          std::string joint_name_ = ptr->Get<std::string>("name");

          if(verbosity > 0) {
           ROS_INFO("%s: %s", joint_name_.c_str(), ptr->GetDescription().c_str());
          }


          model->GetJoint(joint_name_)->Detach();

          ptr = ptr->GetNextElement("joint");
        }
      }

      last_update = ros::Time::now();
    }


    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      static tf::TransformBroadcaster br;

      if(last_update + rate > ros::Time::now()) {
        return;
      } else {
        last_update = ros::Time::now();
      }

      physics::Link_V links = this->model->GetLinks();

      if(verbosity > 2) {
        physics::Joint_V joints = this->model->GetJoints();

        ROS_INFO("Reading %u joints and %u links", (unsigned int) joints.size(), (unsigned int) links.size());
      }

      /* print out transform from reference frame to observation */
      math::Pose pose_ = model->GetWorldPose();
      tf::Transform model_tf_ = poseToTF(pose_);
      br.sendTransform(tf::StampedTransform(model_tf_,
                                            ros::Time::now(),
                                            ref,
                                            getNameTF(ns, model->GetName())));

      for(typename physics::Link_V::const_iterator it = links.begin(); it != links.end(); ++it) {
        tf::Transform t;
        physics::LinkPtr ptr = *it;

        if(verbosity > 2) {
          ROS_INFO("%s %u %s", getNameTF(ns, (*it)->GetName()).c_str(),
                   (unsigned int)ptr->GetChildCount(),
                   ptr->GetParent()->GetName().c_str());
        }

        tf::Transform tf_ = poseToTF(ptr->GetRelativePose());
        std::string parentName = ptr->GetParent()->GetName();

        br.sendTransform(tf::StampedTransform(tf_,
                        ros::Time::now(),
                        getNameTF(ns, parentName),
                        getNameTF(ns, ptr->GetName())));
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
