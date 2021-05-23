#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <string>
#include <thread>

namespace gazebo
{
	class WorldPluginTutorial : public WorldPlugin
	{
		public: WorldPluginTutorial() : WorldPlugin()
		{
			printf("Hello World this is world!\n");
			this->joint_angles[0] = 0;
			this->joint_angles[1] = 0;
			this->joint_angles[2] = 0;
			this->joint_angles[3] = 0;
			this->joint_angles[4] = 0;
			this->joint_angles[5] = 0;
		}
		public: void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
		{
			//_world->EnablePhysicsEngine(false);
			this->model = _world->GetModel("robot_model").get();
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&WorldPluginTutorial::OnUpdate, this, _1));

			std::thread t1(boost::bind(&WorldPluginTutorial::start_routine, this, _1), "Hello"); 
			t1.join();
			

		}

		private: void start_routine(std::string msg)
		{
			int k =0;
			while(k<10)
			{
				k= k+1;
				std::cout<<"l"<<std::endl;
				gazebo::common::Time::MSleep(500);
			}
		}

		public: void OnUpdate(const common::UpdateInfo & /*_info*/)
		{
			this->joint_angles[0] = this->model->GetJoint("ur5::elbow_joint")->GetAngle(0).Radian();
			this->joint_angles[1] = this->model->GetJoint("ur5::shoulder_lift_joint")->GetAngle(0).Radian();
			this->joint_angles[2] = this->model->GetJoint("ur5::shoulder_pan_joint")->GetAngle(0).Radian();
			this->joint_angles[3] = this->model->GetJoint("ur5::wrist_1_joint")->GetAngle(0).Radian();
			this->joint_angles[4] = this->model->GetJoint("ur5::wrist_2_joint")->GetAngle(0).Radian();
			this->joint_angles[5] = this->model->GetJoint("ur5::wrist_3_joint")->GetAngle(0).Radian();

			std::cout<<"Angles Updated"<<std::endl;
		}

		private: void printJointAngles()
		{
			std::cout << "---------------------------\n";
			std::cout << this->joint_angles[0] << std::endl;
			std::cout << this->joint_angles[1] << std::endl;
			std::cout << this->joint_angles[2] << std::endl;
			std::cout << this->joint_angles[3] << std::endl;
			std::cout << this->joint_angles[4] << std::endl;
			std::cout << this->joint_angles[5] << std::endl;
			std::cout << "---------------------------\n";
		}

/*
		private: void OnMsg(ConstWrenchPtr &msg)
		{
			this->model->GetJointController()->SetVelocityTarget(this->model->GetJoint("ur5::elbow_joint")->GetScopedName(), msg->force().x());
			this->model->GetJointController()->SetVelocityTarget(this->model->GetJoint("ur5::shoulder_lift_joint")->GetScopedName(), msg->force().y());
			this->model->GetJointController()->SetVelocityTarget(this->model->GetJoint("ur5::shoulder_pan_joint")->GetScopedName(), msg->force().z());
			this->model->GetJointController()->SetVelocityTarget(this->model->GetJoint("ur5::wrist_1_joint")->GetScopedName(), msg->torque().x());
			this->model->GetJointController()->SetVelocityTarget(this->model->GetJoint("ur5::wrist_2_joint")->GetScopedName(), msg->torque().y());
			this->model->GetJointController()->SetVelocityTarget(this->model->GetJoint("ur5::wrist_3_joint")->GetScopedName(), msg->torque().z());
		}
*/
		private: gazebo::physics::Model *model;
		private: event::ConnectionPtr updateConnection;
		private: double joint_angles[6];
		private: transport::SubscriberPtr sub;
		private: transport::NodePtr node;
	};
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}