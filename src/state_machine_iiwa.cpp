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



int main(int argc, char **argv)
{
	ros::init (argc, argv, "binpicking");
	ROS_INFO("Starting program...");


	bool is_free_ = true;
		
	// Initialize the action server
	ROS_INFO("%d",is_free_);

	BinPickingAction binpicking("binpicking" , is_free_);


	ROS_INFO("%d",is_free_);

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
