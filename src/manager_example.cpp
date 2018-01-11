#include <ros/ros.h>
#include <eigen_functionalities.h>

#include <arm_manager/arm_manager.h>

enum StateMachine
{
	IDLE,
	PICKING,
	DEVOLUTION,
	INITIALIZATION,
	PAUSE
	
};

StateMachine state_;
ArmManager* arm_manager_;
std::mutex state_mutex_;
bool initialization_done_ = false;


bool executeCycle()
{
	StateMachine state = state_;
	switch(state)
	{
		case IDLE:
		{
			ROS_INFO_THROTTLE(5,"In IDLE state");
			std::vector<double> xyz(3,0), rpy(3,0);
			xyz[0] = 0.0; xyz[1] = 0.5; xyz[2] = 0.5;
			rpy[0] = M_PI; rpy[1] = 0.0; rpy[2] = 0.0;  
	

			arm_manager_->movePose(xyz, rpy);

		}
		break;
		
		case PICKING:
		{

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
}







int main(int argc, char **argv)
{

	ros::init (argc, argv, "logimat_demo");
	ros::NodeHandle nh;
	
	ROS_INFO("Starting program...");
		
	ros::AsyncSpinner spinner(0);
	spinner.start();
	
	// Init arm_manager & manipulation_helper
	ROS_INFO("PRE INITIALIZATION ... (JULEN)");

	arm_manager_ = new ArmManager("manipulator", "iiwa/PositionJointInterface_trajectory_controller", "arm");
	// /lbr_iiwa_joint_trajectory_position_controller/follow_joint_trajectory/cancel

	ROS_INFO("well created -> INITIALIZATION ... (JULEN)");
	
	arm_manager_->initManager();
	ROS_INFO("Arm manager initialized");
	
	

  	std::unique_lock<std::mutex> lock(state_mutex_);
  	initialization_done_ = false;
	state_ = IDLE;
	lock.unlock();
	
	run_task();
	 
	ros::waitForShutdown();

	ROS_INFO("Program finished");
   
  return 0;	


}
