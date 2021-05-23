#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <string>

#define DOF 6

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class JointPublisherPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: JointPublisherPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

      this->world = _model->GetWorld();
      //this->world->EnablePhysicsEngine(false);

      std::cout << "Loading JointPulisher \n";

      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, JointController plugin not loaded\n";
        return;
      }

      // Store the model pointer for convenience.
      this->model = _model;

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&JointPublisherPlugin::OnUpdate, this, _1));


      // Get the first joint. We are making an assumption about the model
      // having one joint that is the rotational joint.
      this->joint = _model->GetJoint("ur5::elbow_joint");

      this->joints.push_back(_model->GetJoint("ur5::elbow_joint"));
      this->joints.push_back(_model->GetJoint("ur5::shoulder_lift_joint"));
      this->joints.push_back(_model->GetJoint("ur5::shoulder_pan_joint"));
      this->joints.push_back(_model->GetJoint("ur5::wrist_1_joint"));
      this->joints.push_back(_model->GetJoint("ur5::wrist_2_joint"));
      this->joints.push_back(_model->GetJoint("ur5::wrist_3_joint"));

      std::cout << this->joint->GetScopedName() << std::endl;


      // Setup a P-controller, with a gain of 0.1.
      this->pid = common::PID(10, 0, 0);

      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetVelocityPID(this->joints[0]->GetScopedName(), this->pid);
      this->model->GetJointController()->SetVelocityPID(this->joints[1]->GetScopedName(), this->pid);
      this->model->GetJointController()->SetVelocityPID(this->joints[2]->GetScopedName(), this->pid);
      this->model->GetJointController()->SetVelocityPID(this->joints[3]->GetScopedName(), this->pid);
      this->model->GetJointController()->SetVelocityPID(this->joints[4]->GetScopedName(), this->pid);
      this->model->GetJointController()->SetVelocityPID(this->joints[5]->GetScopedName(), this->pid);


   	  this->model->GetJointController()->SetVelocityTarget(this->joints[0]->GetScopedName(), 0);
      this->model->GetJointController()->SetVelocityTarget(this->joints[1]->GetScopedName(), 0);
      this->model->GetJointController()->SetVelocityTarget(this->joints[2]->GetScopedName(), 0);
      this->model->GetJointController()->SetVelocityTarget(this->joints[3]->GetScopedName(), 0);
      this->model->GetJointController()->SetVelocityTarget(this->joints[4]->GetScopedName(), 0);
      this->model->GetJointController()->SetVelocityTarget(this->joints[5]->GetScopedName(), 0);

      // Create the node
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init("Model_Contoller_Plugin_Node");

      // Subscribe to the topic, and register a callback
      this->sub = this->node->Subscribe("~/joint_vel_cmd",&JointPublisherPlugin::OnMsg, this);
      this->pub = this->node->Advertise<gazebo::msgs::Wrench>("~/joint_states");
    }

    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      double angle_1 = this->joints[0]->GetAngle(0).Radian();
      double angle_2 = this->joints[1]->GetAngle(0).Radian();
      double angle_3 = this->joints[2]->GetAngle(0).Radian();
      double angle_4 = this->joints[3]->GetAngle(0).Radian();
      double angle_5 = this->joints[4]->GetAngle(0).Radian();
      double angle_6 = this->joints[5]->GetAngle(0).Radian();
      ignition::math::Vector3d angle1_3(angle_1,angle_2,angle_3);
      ignition::math::Vector3d angle4_6(angle_4,angle_5,angle_6);
      msgs::Wrench msg;
      msgs::Set(msg.mutable_force(),angle1_3);
      msgs::Set(msg.mutable_torque(),angle4_6);
      this->pub->Publish(msg);
    }

    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    private: void OnMsg(ConstWrenchPtr &msg)
    {
      std::cout << "---------------------------\n";
      std::cout << msg->force().x() << std::endl;
      std::cout << msg->force().y() << std::endl;
      std::cout << msg->force().z() << std::endl;
      std::cout << msg->torque().x() << std::endl;
      std::cout << msg->torque().y() << std::endl;
      std::cout << msg->torque().z() << std::endl;
      std::cout << "---------------------------\n";

      this->model->GetJointController()->SetVelocityTarget(this->joints[0]->GetScopedName(), msg->force().x());
      this->model->GetJointController()->SetVelocityTarget(this->joints[1]->GetScopedName(), msg->force().y());
      this->model->GetJointController()->SetVelocityTarget(this->joints[2]->GetScopedName(), msg->force().z());
      this->model->GetJointController()->SetVelocityTarget(this->joints[3]->GetScopedName(), msg->torque().x());
      this->model->GetJointController()->SetVelocityTarget(this->joints[4]->GetScopedName(), msg->torque().y());
      this->model->GetJointController()->SetVelocityTarget(this->joints[5]->GetScopedName(), msg->torque().z());
    }

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: gazebo::physics::WorldPtr world;

    /// \brief A node used for transport
    private: transport::NodePtr node;

    private: transport::PublisherPtr pub;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

    /// \Vector of joints
    private: std::vector<physics::JointPtr> joints;

    /// \brief A PID controller for the joint.
    private: common::PID pid;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(JointPublisherPlugin)
}

