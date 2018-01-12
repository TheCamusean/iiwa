#include <ros/ros.h>
#include <eigen_functionalities.h>
#include "arm_manager/arm_manager.h"
#include "std_msgs/String.h"
#include <iiwa/state_machine_iiwa_helper.h>

// Action Server related
#include <actionlib/server/simple_action_server.h>
#include <iiwa/BinPickingAction.h>

// Service related
#include "iiwa/GetContainerPose.h"
#include "iiwa/GetPiecePose.h"

//Geometry_msgs
#include "geometry_msgs/Pose.h"

enum StateMachine
{
	END,
	CAM_CONTAINER_POSE,
	SELECT_TOOL,
	CAM_OBJECT_POSE,
	PICKING,
	LEAVE,
	BACK_HOME,
	
};

class GraspingAction
{
protected:

	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<iiwa::GraspingAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
	std::string action_name_;
	// create messages that are used to published feedback/result
	iiwa::GraspingFeedback feedback_;
	iiwa::GraspingResult result_;
	iiwa::GraspingGoal goal_;
	ros::ServiceClient container_client_;
	ros::ServiceClient piece_client_;

	// Pose of container & object
	geometry_msgs::Pose container_pose_;
	geometry_msgs::Pose piece_pose_;
	int find_container_trial_ = 0 ;
	int find_piece_trial_ = 0 ;
	int num_extracted_parts = 0;

	bool *is_free_;


private:

	ArmManager* arm_manager_;
	StateMachine state_;
	StateMachineIiwaHelper* state_machine_iiwa_helper_;
	bool initialization_done_;
	std::mutex state_mutex_;



public:

	BinPickingAction(std::string name, bool& is_free_);

	~BinPickingAction(void);

	void executeCB(const iiwa::GraspingGoalConstPtr &goal);
	
	bool executeCycle();

};	
