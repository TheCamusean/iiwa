#include <arm_manager/arm_manager.h>
#include "sensor_msgs/JointState.h"


class StateMachineIiwaHelper
{
	public:
	
		StateMachineIiwaHelper(ArmManager* arm_manager);
		~StateMachineIiwaHelper();


		//Grasping method
		int GraspObject(std::vector<double> pose);

		void LeaveObject(int frame);

		void getDeliveryPlace(int frame, std::vector<double>& xyz, std::vector<double>& rpy);

		void moveSafePose();

		bool releaseProduct();

	private:
		
		//Arm and gripper manager
		ArmManager* arm_manager_;

		ros::NodeHandle nh_;

		//FollowJointTrajectoryController topic subscriber and callback
		ros::Subscriber joint_state_sub_;



		bool sync_;
		

};