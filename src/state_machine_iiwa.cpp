#include <ros/ros.h>
#include <eigen_functionalities.h>
#include "arm_manager/arm_manager.h"
#include "std_msgs/String.h"
#include <iiwa/state_machine_iiwa_helper.h>

// Action Server related
#include <actionlib/server/simple_action_server.h>
#include <iiwa/BinPickingAction.h>

enum StateMachine
{
	IDLE,
	//CAM_CONTAINER_POSE,
	PICKING,
	GO_TO_LEAVE_PLACE,
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

  	arm_manager_ = new ArmManager("arm", "lbr_iiwa_joint_trajectory_position_controller","arm");
  	arm_manager_->initManager();
  	state_machine_iiwa_helper_ = new StateMachineIiwaHelper(arm_manager_);
	std::unique_lock<std::mutex> lock(state_mutex_);
  	initialization_done_ = false;
  	state_ = IDLE;
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

    // push_back the seeds for the fibonacci sequence
    feedback_.error_code = 34;
    feedback_.error_string = "hola";
    as_.publishFeedback(feedback_);

      r.sleep();
    

    if(success)
    {
      result_.success = success;
      result_.num_extracted_parts = 4;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }

};	


/*bool executeCycle()
{
	StateMachine state = state_;
	switch(state)
	{
		case IDLE:
		{
		ROS_INFO_THROTTLE(5,"In IDLE state");
		}
		break;
		
		case PICKING:
		{
			ROS_INFO_THROTTLE(5,"In PICKING state");
			std::vector<double> xyz(3,0), rpy(3,0);
			
			// HERE WE SHOULD ADD THE TRANSFORMATION: FROM CAM TO BASE

			xyz[0] = 0.5; xyz[1] = 0.0; xyz[2] = 0.3;
			rpy[0] = 3.1415; rpy[1] = 0; rpy[2] = 0;

			std::vector<double> pose = {xyz[0],xyz[1],xyz[2],rpy[0],rpy[1],rpy[2]};

			int ret = state_machine_iiwa_helper_->GraspObject(pose);
			if(ret == -1)
			{
				state_machine_iiwa_helper_->moveSafePose();
				state_ = IDLE;
				ROS_INFO("Movement unreachable");
			}
			else
			{
				ROS_INFO("Movement finished");
				state_ = GO_TO_LEAVE_PLACE;
			}

		}
		break;
		case GO_TO_LEAVE_PLACE:
		{
				state_ = LEAVE;
		}
		break;
		case LEAVE:
		{
			ROS_INFO_THROTTLE(5,"In LEAVING state");

			int frame_id = 3;
			state_machine_iiwa_helper_->LeaveObject(frame_id);

			ROS_INFO("Movement finished");
			state_ = BACK_HOME;
		}
		break;
		case BACK_HOME:
		{
				state_ = IDLE;
		}
		break;
	}

	return false;
}

void run_task()
{
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	
	bool done = false;
	while(nh.ok())
	{
		done = executeCycle();
		if (done) 
			return;
			
		loop_rate.sleep();
	}
}*/


int main(int argc, char **argv)
{

	ros::init (argc, argv, "binpicking");
	ros::NodeHandle nh;
	
	ROS_INFO("Starting program...");
		
	ros::AsyncSpinner spinner(0);
	spinner.start();



	// Initialize the action server
	BinPickingAction binpicking("binpicking");

	ros::spin();
	
	/*run_task();
	 
	ros::waitForShutdown();

	ROS_INFO("Program finished");
   */
  return 0;	
}
