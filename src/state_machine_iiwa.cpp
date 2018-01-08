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

class BinPickingAction
{
protected:

	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<iiwa::BinPickingAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
	std::string action_name_;
	// create messages that are used to published feedback/result
	iiwa::BinPickingFeedback feedback_;
	iiwa::BinPickingResult result_;
	iiwa::BinPickingGoal goal_;
	ros::ServiceClient container_client_;
	ros::ServiceClient piece_client_;

	// Pose of container & object
	geometry_msgs::Pose container_pose_;
	geometry_msgs::Pose piece_pose_;
	int find_container_trial_ = 0 ;
	int find_piece_trial_ = 0 ;


private:

	ArmManager* arm_manager_;
	StateMachine state_;
	StateMachineIiwaHelper* state_machine_iiwa_helper_;
	bool initialization_done_;
	std::mutex state_mutex_;



public:

	BinPickingAction(std::string name) :
	as_(nh_, name, boost::bind(&BinPickingAction::executeCB, this, _1), false),
	action_name_(name)
	{
		ros::AsyncSpinner spinner(0);
		spinner.start();
	
		// ARM MANAGER
		arm_manager_ = new ArmManager("arm", "lbr_iiwa_joint_trajectory_position_controller","arm");
		arm_manager_->initManager();

		//IIWA HELPER
		state_machine_iiwa_helper_ = new StateMachineIiwaHelper(arm_manager_);

		//TO CAMERA SERVICES
		container_client_ = nh_.serviceClient<iiwa::GetContainerPose>("GetContainerPose");
		piece_client_ = nh_.serviceClient<iiwa::GetPiecePose>("GetPiecePose");


		std::unique_lock<std::mutex> lock(state_mutex_);
		initialization_done_ = false;
		state_ = END;
		lock.unlock();
		as_.start();
	}

	~BinPickingAction(void)
	{
	}

	void executeCB(const iiwa::BinPickingGoalConstPtr &goal)
	{
		// helper variables
		ros::Rate r(1);
		bool success = true;

		// Goal variables
		goal_.container = goal->container;
		goal_.piece_type = goal->piece_type;
		result_.num_extracted_parts = 0;

		// push_back the seeds for the fibonacci sequence
		feedback_.error_code = 0;
		feedback_.error_string = "init";
		as_.publishFeedback(feedback_);

		state_ = CAM_CONTAINER_POSE ;

		while(state_ != END)
		{
			success = executeCycle();
			as_.publishFeedback(feedback_);
		}	
		r.sleep();

		if (success)
		{
			ROS_INFO("%s: Succeeded", action_name_.c_str());
		}
		else
		{
			ROS_INFO("%s: Not Succeeded", action_name_.c_str());

		}
		// set the action state to succeeded
		as_.setSucceeded(result_);
	}
	
	bool executeCycle()
	{
		StateMachine state = state_;
		switch(state)
		{
			case END:
			{
				ROS_INFO_THROTTLE(5,"In END state");
			}
			break;

			case CAM_CONTAINER_POSE:
			{
				iiwa::GetContainerPose container_srv;
				container_srv.request.container = goal_.container;
				if (container_client_.call(container_srv))
				{
					if (container_srv.response.found == true)
					{
						container_pose_ = container_srv.response.container_pose;
						ROS_INFO("Container Found in Pose: x:%3.3f,y:%3.3f , z:%3.3f",container_pose_.position.x, container_pose_.position.y, container_pose_.position.z);
						feedback_.error_code = 0;
						feedback_.error_string = "Container found succesfully";
						state_ = SELECT_TOOL;
						find_container_trial_ = 0;
					}
					else if (find_container_trial_ < 5)
					{
						ROS_INFO("Container not found at position %d in trial %d",container_srv.request.container, find_container_trial_);
						feedback_.error_code = -1;
						feedback_.error_string = "Container not found";
						find_container_trial_+=1;
					}
					else
					{	
						find_container_trial_ = 0;
						ROS_INFO("Failed finding the container");
						ROS_INFO("Result : FAILED");
						state_ = END;
					}	
				}
				else
				{
					ROS_ERROR("Failed to call service GetContainerPose");
					state_ = END;
				}
				return 0;
			}
			break;

			case SELECT_TOOL:
			{
				state_ = CAM_OBJECT_POSE;
				feedback_.error_code = 0;
				feedback_.error_string = "correctly selected the tool";
			}
			break;

			case CAM_OBJECT_POSE:
			{
				iiwa::GetPiecePose piece_srv;
				piece_srv.request.container = goal_.container;
				piece_srv.request.piece_type = goal_.piece_type;

				if (piece_client_.call(piece_srv))
				{	
					if (piece_srv.response.piece_error == false && piece_srv.response.capture_error ==false)
					{
						piece_pose_ = piece_srv.response.piece_pose;
						ROS_INFO("Piece Found in Pose: x:%3.3f,y:%3.3f , z:%3.3f",piece_pose_.position.x, piece_pose_.position.y, piece_pose_.position.z);
						feedback_.error_code = 0;
						feedback_.error_string = "Piece found succesfully";
						state_ = PICKING;
						find_piece_trial_ = 0;
					}
					else if (find_piece_trial_ < 5)
					{
						ROS_INFO("Piece not found at position %d in trial %d",piece_srv.request.container,find_piece_trial_);
						if (piece_srv.response.piece_error == true)
						{
							feedback_.error_code = -2;
							feedback_.error_string = "Piece not found";
						}
						else
						{
							feedback_.error_code = -3;
							feedback_.error_string = "Capture error";
						}
						find_piece_trial_+=1;
					}
					else
					{	
						find_piece_trial_ = 0;
						ROS_INFO("Failed finding the piece");
						ROS_INFO("Result : FAILED");
						state_ = END;
					}	
				}
				else
				{
					ROS_ERROR("Failed to call service GetContainerPose");
					state_ = END;
				}
				return 0;
			}
			break;

			case PICKING:
			{
				ROS_INFO_THROTTLE(5,"In PICKING state");
				std::vector<double> xyz(3,0), rpy(3,0);

				Eigen::Quaternion<double> q = Eigen::Quaternion<double>(piece_pose_.orientation.w,piece_pose_.orientation.x,piece_pose_.orientation.y,piece_pose_.orientation.z);
				Eigen::Matrix3d mat_q(q);
				//eigen_functionalities::extractEulerAnglesZYX(mat_q, rpy[2], rpy[1], rpy[0]);

				
				// HERE WE SHOULD ADD THE TRANSFORMATION: FROM CAM TO BASE
				xyz[0] = 0.5; xyz[1] = 0.0; xyz[2] = 0.3;
				rpy[0] = 3.1415; rpy[1] = 0; rpy[2] = 0;
				std::vector<double> pose = {xyz[0],xyz[1],xyz[2],rpy[0],rpy[1],rpy[2]};

				int ret = state_machine_iiwa_helper_->GraspObject(pose);
				if(ret == -1)
				{
					state_machine_iiwa_helper_->moveSafePose();
					feedback_.error_code = -4;
					feedback_.error_string = "picking error";
					ROS_INFO("Movement unreachable");
					state_ = END;

				}
				else
				{
					feedback_.error_code = 0;
					feedback_.error_string = "picking correctly done";
					ROS_INFO("Movement finished");
					state_ = END;
				}
			}
			break;

			break;
			case LEAVE:
			{

			}
			break;
			case BACK_HOME:
			{

			}
			break;
		}

		return false;
	}
};	


int main(int argc, char **argv)
{
	ros::init (argc, argv, "binpicking");
	ROS_INFO("Starting program...");
		
	// Initialize the action server
	BinPickingAction binpicking("binpicking");

	 ros::spin(); 
	// ros::waitForShutdown();
	return 0;	
}
