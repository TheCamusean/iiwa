#include <iiwa/state_machine_iiwa_helper.h>
#include <arm_manager/arm_manager.h>
#include <eigen_functionalities.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"


StateMachineIiwaHelper::StateMachineIiwaHelper(ArmManager* arm_manager):
sync_(false)
{
	arm_manager_ = arm_manager;

	ros::NodeHandle nh("~");

	std::vector<double> rpy(0,3);
	Eigen::Quaternion<double> q = Eigen::Quaternion<double>(0.5,0.5,0.5,0.5);
	Eigen::Matrix3d mat_q(q);
	eigen_functionalities::extractEulerAnglesZYX(mat_q, rpy[2], rpy[1], rpy[0]);

				

}



StateMachineIiwaHelper::~StateMachineIiwaHelper()
{
	delete(arm_manager_);
}

void StateMachineIiwaHelper::getDeliveryPlace(int frame, std::vector<double>& xyz, std::vector<double>& rpy)
{

	if(frame == 1){xyz[0] = 0.8;}
	else if(frame == 2){xyz[0] = 0.8;}
	else if(frame == 3){xyz[0] = 1.4;}
	else if(frame == 4){xyz[0] = 1.4;}
	else if(frame == 5){xyz[0] = 2.2;}
	else if(frame == 6){xyz[0] = 2.2;}

}

int StateMachineIiwaHelper::GraspObject(std::vector<double> pose)// This pose is given in base frame for now, afterwards we should modify to CAM frame
{	
	// *** Plan *** //
	std::vector<double> jVals(7,0);

	moveit::planning_interface::MoveGroupInterface::Plan planToMiddlePose, planToPrevPose, planToPose, planToBack;
	
	std::vector<double> xyz(3,0), rpy(3,0);

	//Phase I: Middle Pose
	xyz[0] = 0.6; xyz[1] = 0.0; xyz[2] = 0.5;
	rpy[0] = M_PI; rpy[1] = 0.0; rpy[2] = 0.0; 
	
	
	if(arm_manager_->planMovementCartesian(xyz,rpy,planToMiddlePose,true)!=PLANNING_SUCCEED)
	{
		ROS_INFO("middle box planning fails");
		return -1;
	}	
	int size = (int) planToMiddlePose.trajectory_.joint_trajectory.points.size();
	jVals = planToMiddlePose.trajectory_.joint_trajectory.points[size-1].positions;

	// Phase II: Go close to the object with a certain orientation

	Eigen::Transform<double, 3, Eigen::Affine> obj_base_H, prev_obj_H, prev_base_H;

	xyz[0] = pose[0]; xyz[1] = pose[1]; xyz[2] = pose[2];
	rpy[0] = pose[3]; rpy[1] = pose[4]; rpy[2] = pose[5];
	eigen_functionalities::createHomogeneousMatrix(xyz, rpy, obj_base_H);

	xyz[0] = 0.0; xyz[1] = 0.0; xyz[2] = -0.2;
	rpy[0] = 0.0; rpy[1] = 0.0; rpy[2] = 0.0;
	eigen_functionalities::createHomogeneousMatrix(xyz, rpy, prev_obj_H);

	eigen_functionalities::changeFrame(prev_obj_H, obj_base_H, prev_base_H);
	eigen_functionalities::extractXyzRpy(prev_base_H, xyz, rpy);

	ROS_INFO("Prev: %3.3f %3.3f %3.3f - %3.3f %3.3f %3.3f",xyz[0],xyz[1],xyz[2],rpy[0],rpy[1],rpy[2]);
	eigen_functionalities::createHomogeneousMatrix(xyz,rpy,prev_base_H);

	if(arm_manager_->planMovementCartesian(jVals,xyz,rpy,planToPrevPose,true)!=PLANNING_SUCCEED)
	{
		ROS_INFO("before the object planning fails");
		return -1;
	}	
	size = (int) planToPrevPose.trajectory_.joint_trajectory.points.size();
	jVals = planToPrevPose.trajectory_.joint_trajectory.points[size-1].positions;

	// Phase III :  GO to object

	eigen_functionalities::extractXyzRpy(obj_base_H, xyz, rpy);
	if(arm_manager_->planMovementCartesianLin(jVals,xyz,rpy,planToPose,true,1.5,0.050,0.050)!=PLANNING_SUCCEED)
	{
		ROS_INFO("plan to object planning fails");
		return -1;
	}

	// Eliminating the first element in order to avoid bugs

	int c1 = 0;
	int c2 = 0;
	if(planToPose.trajectory_.joint_trajectory.points[0].time_from_start == planToPose.trajectory_.joint_trajectory.points[1].time_from_start)
	{
		c1 = 1;
	}
	ROS_INFO("CheckPoint 1");
	size = planToPose.trajectory_.joint_trajectory.points.size();
	if(planToPose.trajectory_.joint_trajectory.points[size-1].time_from_start == planToPose.trajectory_.joint_trajectory.points[size -2].time_from_start)
	{
		c2 = 1;
	}
	ROS_INFO("CheckPoint 2");


	std::vector<trajectory_msgs::JointTrajectoryPoint> points(planToPose.trajectory_.joint_trajectory.points.size()-c1-c2);
	for(unsigned int i = 0; i<planToPose.trajectory_.joint_trajectory.points.size()-c1-c2; i++)
	{

		points[i] = planToPose.trajectory_.joint_trajectory.points[i+c1];

	}
	ROS_INFO("CheckPoint 3");

	planToPose.trajectory_.joint_trajectory.points = points;

	// Upto here

	size = (int) planToPose.trajectory_.joint_trajectory.points.size();
	jVals = planToPose.trajectory_.joint_trajectory.points[size-1].positions;
	
	// Phase IV : BAck to Safety Pose
	xyz[0] = 0.6; xyz[1] = 0.0; xyz[2] = 0.5;
	rpy[0] = M_PI; rpy[1] = 0.0; rpy[2] = 0.0; 
	if(arm_manager_->planMovementCartesian(jVals,xyz,rpy,planToBack,true)!=PLANNING_SUCCEED)
	{
		ROS_INFO("safety back planning fails");
		return -1;
	}	


	// ******** MOVE ************* //

	// To Middle pose
	ROS_INFO("To middle pose...");

	int counter = 0;
	arm_manager_->executePlan(planToMiddlePose, sync_);
	if(!sync_)
	{		
		while(!arm_manager_->movementFinished())
		{
			//ROS_INFO("Arm moving");
			sleep(0.05);
			counter +=1;
			if(counter == 100000)
			{
				
				ROS_INFO("Failes reaching middle pose");
				return -1;
			}
		}
	}
	


	// To previous of the object 
	ROS_INFO("To prev object pose...");
	counter = 0;
	arm_manager_->executePlan(planToPrevPose, sync_);
	if(!sync_)
	{		
		while(!arm_manager_->movementFinished())
		{
			//ROS_INFO("Arm moving to prev object");
			sleep(0.05);
			counter +=1;
			if(counter == 100000)
			{
				
				ROS_INFO("Failes reaching prev pose");
				return -1;
			}
		}
	}

	// To the object

	
	ROS_INFO("To to object pose...");
	counter = 0;
	arm_manager_->executePlan(planToPose, sync_);
	if(!sync_)
	{		
		while(!arm_manager_->movementFinished())
		{
			//ROS_INFO("Arm moving");
			sleep(0.05);
			counter +=1;
			if(counter == 100000)
			{
				
				ROS_INFO("Failes reaching object pose");
				return -1;
			}
		}
	}


	// Back To Safety Pose
	
	ROS_INFO("To backt pose...");
	counter = 0;
	arm_manager_->executePlan(planToBack, sync_);
	if(!sync_)
	{		
		while(!arm_manager_->movementFinished())
		{
			//~ ROS_INFO("Arm moving");
			sleep(0.05);
			counter +=1;
			if(counter == 100000)
			{
				ROS_INFO("Failes reaching back pose");
				return -1;

			}
		}
	}


	return 1;

}

void StateMachineIiwaHelper::LeaveObject(int frame)// This pose is given in base frame for now, afterwards we should modify to CAM frame
{	
	// Phase I: Go to the side of the box (Depends on the ID)

	std::vector<double> xyz(3,0), rpy(3,0);

	if(frame %2 == 0){

		xyz[0] = 0.6; xyz[1] = 0.4; xyz[2] = 0.5;
		rpy[0] = M_PI; rpy[1] = 0.0; rpy[2] = 0.0; 

	}else{
		xyz[0] = 0.6; xyz[1] = -0.4; xyz[2] = 0.5;
		rpy[0] = M_PI; rpy[1] = 0.0; rpy[2] = 0.0; 


	}


	arm_manager_->movePose(xyz, rpy);
	while(!arm_manager_->movementFinished())
	{
		sleep(0.05);
	}

	//Phase II : UnGrasp

	//Phase III : Back to Safety Pose

	xyz[0] = 0.6; xyz[1] = 0.0; xyz[2] = 0.5;
	rpy[0] = M_PI; rpy[1] = 0.0; rpy[2] = 0.0;

	arm_manager_->movePose(xyz, rpy);
	while(!arm_manager_->movementFinished())
	{
		sleep(0.05);
	}
	
	

}
void StateMachineIiwaHelper::moveSafePose()
{
	ROS_INFO("Back to Safety Pose");

	std::vector<double> xyz(3,0), rpy(3,0);

	xyz[0] = 0.6; xyz[1] = 0.0; xyz[2] = 0.5;
	rpy[0] = M_PI; rpy[1] = 0.0; rpy[2] = 0.0;

	arm_manager_->movePose(xyz, rpy);
	while(!arm_manager_->movementFinished())
	{
		sleep(0.05);
	}
}






