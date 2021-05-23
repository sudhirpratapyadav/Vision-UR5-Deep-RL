#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>


double joint_angles[6]; 

cv::Mat image;

void cb_camera(ConstImageStampedPtr &msg)
{
	memcpy(image.ptr(0), msg->image().data().c_str(), msg->image().data().length());
}

void cb_joint_state(ConstWrenchPtr &msg)
{
	joint_angles[0] = msg->force().x();
	joint_angles[1] = msg->force().x();
	joint_angles[2] = msg->force().x();
	joint_angles[3] = msg->torque().x();
	joint_angles[4] = msg->torque().y();
	joint_angles[5] = msg->torque().z();
}

void printJointAngles()
{
	std::cout << "---------------------------\n";
	std::cout << joint_angles[0] << std::endl;
	std::cout << joint_angles[1] << std::endl;
	std::cout << joint_angles[2] << std::endl;
	std::cout << joint_angles[3] << std::endl;
	std::cout << joint_angles[4] << std::endl;
	std::cout << joint_angles[5] << std::endl;
	std::cout << "---------------------------\n";
}

void publishVelocities(gazebo::transport::PublisherPtr pub, double vel[6])
{
	ignition::math::Vector3d angle1_3(vel[0],vel[1],vel[2]);
    ignition::math::Vector3d angle4_6(vel[3],vel[4],vel[5]);
    gazebo::msgs::Wrench msg;
    gazebo::msgs::Set(msg.mutable_force(),angle1_3);
    gazebo::msgs::Set(msg.mutable_torque(),angle4_6);
    pub->Publish(msg);
}

int main(int _argc, char **_argv)
{
	//Initialization
	image = cv::Mat(240,320, CV_8UC3, cv::Scalar(0,0,0));
	joint_angles[0] = 0;
	joint_angles[1] = 0;
	joint_angles[2] = 0;
	joint_angles[3] = 0;
	joint_angles[4] = 0;
	joint_angles[5] = 0;

	// Load gazebo
	gazebo::client::setup(_argc, _argv);

	// Create our node for communication
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();

	// Listen to Gazebo world_stats topic
	gazebo::transport::SubscriberPtr sub1 = node->Subscribe("/gazebo/default/robot_model/ur5/camera_link/camera/image", cb_camera);
	gazebo::transport::SubscriberPtr sub2 = node->Subscribe("/gazebo/Model_Contoller_Plugin_Node/joint_states", cb_joint_state);
	gazebo::transport::PublisherPtr  pub  = node->Advertise<gazebo::msgs::Wrench>("/gazebo/Model_Contoller_Plugin_Node/joint_vel_cmd");

	// Busy wait loop...replace with your own code as needed.
	while (true)
	{
		cv::imshow("camera", image);
		cv::waitKey(1);
		printJointAngles();
		double vel[6] = {0.1,0.1,0.1,0.1,0.1,0.1};
		publishVelocities(pub,vel);
		gazebo::common::Time::MSleep(10);
	}

	// Make sure to shut everything down.
	gazebo::client::shutdown();

	return 0;
}