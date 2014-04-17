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
      math::Pose pose; // where should these be relative to one another?
      physics::JointPtr ptr; // simulated joint for attachment
    };

    std::vector<magnetic_joint> mjoints; // list of points that latch together

    ros::NodeHandle nh_; // ros node handle
    ros::Publisher pub_; // ros joint state publisher

    std::string ns; // namespace to publish tf frames under
    std::string ref; // tf reference frame name (world frame)
    int verbosity; // amount of output to print

    int enable_latch; // is latching enabled for this model?
    double latch_strength; // how much force can joints withstand?
    double latch_distance; // how close before joints latch together?
    double latch_rotation; // rotation difference between joints

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

      if(enable_latch) {
        latch_strength = _sdf->GetElement("latch_strength")->Get<double>();
        latch_distance = _sdf->GetElement("latch_distance")->Get<double>();
        latch_rotation = _sdf->GetElement("latch_rotation")->Get<double>();
      } else {
        latch_strength = 0.0;
      }
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

        if(verbosity > 0) {
          ROS_INFO("Loading joints...");
        }

        /* loop over sdf elements and look at floating joints */
        sdf::ElementPtr ptr = sdf_->GetElement("joint");
        while (ptr) {

          std::string pose_str;
          std::string joint_name_ = ptr->Get<std::string>("name");

          if(verbosity > 0) {
           ROS_INFO("%s: %s", joint_name_.c_str(), ptr->GetDescription().c_str());
          }


          //model->GetJoint(joint_name_)->Detach();

          /* NOTE: we are reading an SDF, not an URDF! The format is different! */
          magnetic_joint jnt;
          jnt.from = ptr->GetElement("parent")->Get<std::string>();
          jnt.to = ptr->GetElement("child")->Get<std::string>();
          jnt.ptr= model->GetJoint(joint_name_);
          jnt.ptr->Detach();
          //jnt.ptr->Attach(model->GetLink(jnt.from), model->GetLink(jnt.to));

          pose_str = ptr->GetElement("pose")->Get<std::string>();

          if(verbosity > 1) {
            ROS_INFO("from=%s to=%s pose=(%s)",
                     jnt.from.c_str(),
                     jnt.to.c_str(),
                     pose_str.c_str());
          }

          ptr = ptr->GetNextElement("joint");
          mjoints.push_back(jnt);
        }


        //std::map<std::string, math::Pose> poses;

        /* Loop through links, find the offsets */
        //physics::Link_V links = this->model->GetLinks();
        //for(typename physics::Link_V::const_iterator it = links.begin(); it != links.end(); ++it) {
        //  physics::LinkPtr ptr = *it;
        //  poses[ptr->GetName()] = ptr->GetWorldCoGPose();
        //}

        /* loop through the joints, compute position differences from parent to child */
        for(std::vector<magnetic_joint>::iterator it = mjoints.begin(); it != mjoints.end(); ++it) {
          math::Pose parent = model->GetLink(it->from)->GetWorldCoGPose();
          math::Pose child = model->GetLink(it->to)->GetWorldCoGPose();

          it->pose = child - parent;
          it->latched = false; // start off without any joints latched

          if(verbosity > 1) {
            ROS_INFO("%s --> %s: (%f %f %f) (%f %f %f %f)", it->from.c_str(), it->to.c_str(),
                     it->pose.pos.x, it->pose.pos.y, it->pose.pos.z,
                     it->pose.rot.x, it->pose.rot.y, it->pose.rot.z, it->pose.rot.w);
          }
        }
      }

      last_update = ros::Time::now();
    }

    public: ~ModelJointStatePublisher() {
      //for(std::vector<magnetic_joint>::iterator it = mjoints.begin(); it != mjoints.end(); ++it) {
      //  delete &(*it->ptr);
      //}
    }

    static inline double vector3_norm(const double x, const double y, const double z) {
      return sqrt((x*x) + (y*y) + (z*z));
    }


    static inline double vector3_norm(const math::Vector3 vec) {
      double x = vec.x;
      double y = vec.y;
      double z = vec.z;
      return sqrt((x*x) + (y*y) + (z*z));
    }


    static inline double quaternion_norm(const double x, const double y, const double z, const double w) {
      return sqrt((x*x) + (y*y) + (z*z) + (w*w));
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

      /* should joints latch together? */
      if(enable_latch) {
        for(std::vector<magnetic_joint>::iterator it = mjoints.begin(); it != mjoints.end(); ++it) {
          // if not latched:
          // check to see if the distances are low
          // if they are: set world position = parent->pose + joint->pose
          // if latched:
          // check to see if force > threshold
          // if so, disable latch
          
          physics::LinkPtr parent = model->GetLink(it->from);
          physics::LinkPtr child = model->GetLink(it->to);
          
          math::Pose diff = child->GetWorldCoGPose() - parent->GetWorldCoGPose() - it->pose;

          // check relative position
          double tdist = vector3_norm(diff.pos.x, diff.pos.y, diff.pos.z);
          double rdist = quaternion_norm(diff.rot.x, diff.rot.y, diff.rot.z, diff.rot.w);

          if(!it->latched) {

            if(verbosity > 2) {
              ROS_INFO("Joint from %s to %s: (%f %f %f) (%f %f %f %f)", it->from.c_str(), it->to.c_str(),
                       diff.pos.x, diff.pos.y, diff.pos.z,
                       diff.rot.x, diff.rot.y, diff.rot.z, diff.rot.w);
            }


            if(tdist <= latch_distance && rdist <= latch_rotation) {
              if(verbosity > 1) {
                ROS_INFO("Latching joint between \"%s\" and \"%s\" now!", it->from.c_str(), it->to.c_str());
              }
              it->ptr->Attach(parent, child);
              it->latched = true;
            }

            // if they are touching, attach the joint again
          } else {
            math::Vector3 rf = child->GetWorldForce();
            math::Vector3 rt = child->GetWorldTorque();

            double force = vector3_norm(rf.x, rf.y, rf.z);
            double torque = vector3_norm(rt.x, rt.y, rt.z);

            if(verbosity > 3) {
              ROS_INFO("%f %f / %f %f", vector3_norm(child->GetWorldLinearAccel()),
                     vector3_norm(child->GetWorldCoGLinearVel()),
                     vector3_norm(parent->GetWorldLinearAccel()),
                     vector3_norm(parent->GetWorldCoGLinearVel()));
            }

            if(tdist > 2*latch_distance) {
              ROS_INFO("Detaching joint between \"%s\" and \"%s\" now!", it->from.c_str(), it->to.c_str());
            }
          }
          
          // anything we need to do?
          if (it->latched) {
            
          }
        }
      }


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
