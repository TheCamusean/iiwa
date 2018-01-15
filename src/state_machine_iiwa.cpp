#include <ros/ros.h>
#include <eigen_functionalities.h>
#include "arm_manager/arm_manager.h"
#include "std_msgs/String.h"
//#include <iiwa/state_machine_iiwa_helper.h>

// Action Server related
#include <actionlib/server/simple_action_server.h>
#include <iiwa/BinPickingAction.h>

// Service related
#include "iiwa/GetContainerPose.h"
#include "iiwa/GetPiecePose.h"

//Geometry_msgs
#include "geometry_msgs/Pose.h"

#include "iiwa/binPickingAction_class.h"

#include <ros/callback_queue.h>

#include "iiwa/GraspInfo.h"

bool is_free_ = true;
std::vector<geometry_msgs::Pose> leave_poses_;



bool grasp(iiwa::GraspInfo::Request  &req,
         iiwa::GraspInfo::Response &res)
{
  return true;
}

int main(int argc, char **argv)
{
	ros::init (argc, argv, "binpicking");
	ROS_INFO("Starting program...");


		
	// Initialize the action server
	ros::NodeHandle n("");


	BinPickingAction binpicking("binpicking" , is_free_, leave_poses_);

	ros::ServiceServer service = n.advertiseService("grasping", grasp);

	//ros::spin();
	ros::Rate r(10);
	while (ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	} 
		// ros::waitForShutdown();
	return 0;	
}
