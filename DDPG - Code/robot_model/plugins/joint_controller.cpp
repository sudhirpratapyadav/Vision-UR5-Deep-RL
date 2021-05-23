#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>


#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>


namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class JointControllerPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: JointControllerPlugin() : ModelPlugin()
    {
      printf("model plugin");
    }

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

      
      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, JointController plugin not loaded\n";
        return;
      }

      // Store the model pointer for convenience.
      this->model = _model;

      // Get the first joint. We are making an assumption about the model
      // having one joint that is the rotational joint.

      //this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&JointControllerPlugin::OnUpdate, this, _1));

      this->joints.push_back(_model->GetJoint("ur5::elbow_joint"));
      this->joints.push_back(_model->GetJoint("ur5::shoulder_lift_joint"));
      this->joints.push_back(_model->GetJoint("ur5::shoulder_pan_joint"));
      this->joints.push_back(_model->GetJoint("ur5::wrist_1_joint"));
      this->joints.push_back(_model->GetJoint("ur5::wrist_2_joint"));
      this->joints.push_back(_model->GetJoint("ur5::wrist_3_joint"));

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
      std::cout << "Position set" << std::endl;

      // Apply the P-controller to the joint.
      //this->model->GetJointController()->SetVelocityPID(this->joint->GetScopedName(), this->pid)

    }
/*
     public: void OnUpdate(const common::UpdateInfo & )
    {
      this->model->GetJointController()->SetVelocityTarget(this->joints[0]->GetScopedName(), 1);
      this->model->GetJointController()->SetVelocityTarget(this->joints[1]->GetScopedName(), 1);
      this->model->GetJointController()->SetVelocityTarget(this->joints[2]->GetScopedName(), 1);
      this->model->GetJointController()->SetVelocityTarget(this->joints[3]->GetScopedName(), 1);
      this->model->GetJointController()->SetVelocityTarget(this->joints[4]->GetScopedName(), 1);
      this->model->GetJointController()->SetVelocityTarget(this->joints[5]->GetScopedName(), 1);
      std::cout << "Position set" << std::endl;
    }
    */


    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: std::vector<physics::JointPtr> joints;

    /// \brief A PID controller for the joint.
    private: common::PID pid;

    private: event::ConnectionPtr updateConnection;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(JointControllerPlugin)
}
