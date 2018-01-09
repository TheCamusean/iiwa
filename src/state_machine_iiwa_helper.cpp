#include <iiwa/state_machine_iiwa_helper.h>
#include <arm_manager/arm_manager.h>
#include <eigen_functionalities.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"



StateMachineIiwaHelper::StateMachineIiwaHelper(ArmManager* arm_manager):
sync_(false)
{
	arm_manager_ = arm_manager;

	ros::NodeHandle nh("~");

	ros::NodeHandle nh_robot("arm");

	nh_robot.getParam("leave_pose", leave_pose);
	nh_robot.getParam("safety_pose", safety_pose);// The position where the robot will remain when no movement is selected

	std::vector<double> gripper_tool_xyzrpy(6,0), vacuum_tool_xyzrpy(6,0);
	nh_robot.getParam("gripper_tool",gripper_tool_xyzrpy);
	nh_robot.getParam("vacuum_tool",vacuum_tool_xyzrpy);
	// Load TCP
	gripper_tool_xyz_.resize(3,0); gripper_tool_rpy_.resize(3,0);
	gripper_tool_xyz_[0]=gripper_tool_xyzrpy[0];gripper_tool_xyz_[1]=gripper_tool_xyzrpy[1];gripper_tool_xyz_[2]=gripper_tool_xyzrpy[2];
	gripper_tool_rpy_[0]=gripper_tool_xyzrpy[3];gripper_tool_rpy_[1]=gripper_tool_xyzrpy[4];gripper_tool_rpy_[2]=gripper_tool_xyzrpy[5];
	vacuum_tool_xyz_.resize(3,0); vacuum_tool_rpy_.resize(3,0);
	vacuum_tool_xyz_[0]=vacuum_tool_xyzrpy[0];vacuum_tool_xyz_[1]=vacuum_tool_xyzrpy[1];vacuum_tool_xyz_[2]=vacuum_tool_xyzrpy[2];
	vacuum_tool_rpy_[0]=vacuum_tool_xyzrpy[3];vacuum_tool_rpy_[1]=vacuum_tool_xyzrpy[4];vacuum_tool_rpy_[2]=vacuum_tool_xyzrpy[5];
	eigen_functionalities::createHomogeneousMatrix(gripper_tool_xyz_, gripper_tool_rpy_, gripper_tool_pose_H_);
	eigen_functionalities::createHomogeneousMatrix(vacuum_tool_xyz_, vacuum_tool_rpy_, vacuum_tool_pose_H_);
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

void StateMachineIiwaHelper::setTool(std::string tool_name)
{
	if(tool_name == "gripper"){	arm_manager_->setTool(gripper_tool_xyz_,gripper_tool_rpy_);}
	else if(tool_name == "vacuum"){	arm_manager_->setTool(vacuum_tool_xyz_,vacuum_tool_rpy_);}
}

std::vector<double>  StateMachineIiwaHelper::FromPose2Vector(geometry_msgs::Pose pose)
{
	std::vector<double> xyzrpy(6,0);
	Eigen::Quaternion<double> q = Eigen::Quaternion<double>(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
	Eigen::Matrix3d mat_q(q);
	eigen_functionalities::extractEulerAnglesZYX(mat_q, xyzrpy[5], xyzrpy[4], xyzrpy[3]);
	xyzrpy[0] = pose.position.x;
	xyzrpy[1] = pose.position.y;
	xyzrpy[2] = pose.position.z;
	return xyzrpy;

}

int StateMachineIiwaHelper::GraspObject(geometry_msgs::Pose pose)
{
				std::vector<double> xyzrpy(6,0);
				Eigen::Quaternion<double> q = Eigen::Quaternion<double>(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
				Eigen::Matrix3d mat_q(q);
				eigen_functionalities::extractEulerAnglesZYX(mat_q, xyzrpy[5], xyzrpy[4], xyzrpy[3]);
				xyzrpy[0] = pose.position.x;
				xyzrpy[1] = pose.position.y;
				xyzrpy[2] = pose.position.z;
				return this->GraspObject(xyzrpy);
}
int StateMachineIiwaHelper::GraspObject(std::vector<double> pose)// This pose is given in base frame for now, afterwards we should modify to CAM frame
{	
	// ------------------------------------- PLANIFICATION ------------------------------------- //
	std::vector<double> jVals(7,0);
	moveit::planning_interface::MoveGroupInterface::Plan planToMiddlePose, planToPrevPose, planToPose, planToBack;
	std::vector<double> xyz(3,0), rpy(3,0);
	//Phase I: Container Pose
	std::vector<double> xyzrpy(6,0);
	xyzrpy = FromPose2Vector(this->container_pose);
	xyz[0] = xyzrpy[0]; xyz[1] = xyzrpy[1]; xyz[2] = xyzrpy[2];
	rpy[0] = xyzrpy[3]; rpy[1] = xyzrpy[4]; rpy[2] = xyzrpy[5]; 
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
	// ----------------  Eliminating the first element in order to avoid bugs -------------------------//
	int c1 = 0;
	int c2 = 0;
	if(planToPose.trajectory_.joint_trajectory.points[0].time_from_start == planToPose.trajectory_.joint_trajectory.points[1].time_from_start)
	{
		c1 = 1;
	}
	size = planToPose.trajectory_.joint_trajectory.points.size();
	if(planToPose.trajectory_.joint_trajectory.points[size-1].time_from_start == planToPose.trajectory_.joint_trajectory.points[size -2].time_from_start)
	{
		c2 = 1;
	}
	std::vector<trajectory_msgs::JointTrajectoryPoint> points(planToPose.trajectory_.joint_trajectory.points.size()-c1-c2);
	for(unsigned int i = 0; i<planToPose.trajectory_.joint_trajectory.points.size()-c1-c2; i++)
	{

		points[i] = planToPose.trajectory_.joint_trajectory.points[i+c1];

	}
	planToPose.trajectory_.joint_trajectory.points = points;
	// -------------------------------------------------------------------------//
	size = (int) planToPose.trajectory_.joint_trajectory.points.size();
	jVals = planToPose.trajectory_.joint_trajectory.points[size-1].positions;
	// Phase IV : BAck to Safety Pose
	xyz[0] = xyzrpy[0]; xyz[1] = xyzrpy[1]; xyz[2] = xyzrpy[2];
	rpy[0] = xyzrpy[3]; rpy[1] = xyzrpy[4]; rpy[2] = xyzrpy[5]; 
	if(arm_manager_->planMovementCartesian(jVals,xyz,rpy, planToBack,true)!=PLANNING_SUCCEED)
	{
		ROS_INFO("safety back planning fails");
		return -1;
	}	
	// ------------------------------------ MOVING ------------------------------------ //
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
				ROS_INFO("Failed reaching middle pose");
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
				
				ROS_INFO("Failed reaching prev pose");
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
				
				ROS_INFO("Failed reaching object pose");
				return -1;
			}
		}
	}
	// Back To Safety Pose
	ROS_INFO("To back pose...");
	counter = 0;
	arm_manager_->executePlan(planToBack, sync_);
	if(!sync_)
	{		
		while(!arm_manager_->movementFinished())
		{
			//~ ROS_INFO("Arm moving");
			sleep(0.05);
		}
	}
	return 1;
}

int StateMachineIiwaHelper::LeaveObject(int num_extracted_parts)// This pose is given in base frame for now, afterwards we should modify to CAM frame
{	
	// ------------------------------------- PLANIFICATION ------------------------------------- //
	std::vector<double> jVals(7,0);
	moveit::planning_interface::MoveGroupInterface::Plan planToLeavePose, planToSafetyPose;
	//------------  Phase I: Leaving Pose  ------------//
	std::vector<double> xyz(3,0), rpy(3,0);
	xyz[0] = leave_pose[0]; xyz[1] = leave_pose[1] - 0.1*num_extracted_parts; xyz[2] = leave_pose[2];
	rpy[0] = leave_pose[3]; rpy[1] = leave_pose[4]; rpy[2] = leave_pose[5]; 
	if(arm_manager_->planMovementCartesian(xyz,rpy,planToLeavePose,true)!=PLANNING_SUCCEED)
	{
		ROS_INFO("leave pose planning fails");
		return -1;
	}	
	int size = (int) planToLeavePose.trajectory_.joint_trajectory.points.size();
	jVals = planToLeavePose.trajectory_.joint_trajectory.points[size-1].positions;
	//------------  Phase II: SAFETY Pose  ------------//
	std::vector<double> xyzrpy(6,0);
	xyzrpy = FromPose2Vector(this->container_pose);
	xyz[0] = xyzrpy[0]; xyz[1] = xyzrpy[1]; xyz[2] = xyzrpy[2];
	rpy[0] = xyzrpy[3]; rpy[1] = xyzrpy[4]; rpy[2] = xyzrpy[5]; 
	if(arm_manager_->planMovementCartesian(jVals,xyz,rpy, planToSafetyPose,true)!=PLANNING_SUCCEED)
	{
		ROS_INFO("safety pose planning fails");
		return -1;
	}
	// ------------------------------------ MOVING ------------------------------------ //
	//------------  Phase I: Leaving Pose  ------------//
	ROS_INFO("move to leaving pose...");
	arm_manager_->executePlan(planToLeavePose, sync_);
	if(!sync_)
	{		
		while(!arm_manager_->movementFinished())
		{
			sleep(0.05);
		}
	}
	//------------  Phase II: UNGRASP PIECE ------------//
	sleep(2);	
	//------------  Phase III: SAFETY Pose  ------------//	
	ROS_INFO("move to safety pose...");
	arm_manager_->executePlan(planToSafetyPose, sync_);
	if(!sync_)
	{		
		while(!arm_manager_->movementFinished())
		{
			sleep(0.05);
		}
	}
	return 1;
}

int StateMachineIiwaHelper::moveSafePose()
{
	std::vector<double> j_safe_pose_(7,0);
	j_safe_pose_=safety_pose; 
  	moveit::planning_interface::MoveGroupInterface::Plan planToSafetyPose;
  	if(arm_manager_->planMovementJoints(j_safe_pose_, planToSafetyPose)!=PLANNING_SUCCEED)
	{
		ROS_INFO("safety pose planning fails");
		return -1;
	}
	arm_manager_->executePlan(planToSafetyPose, sync_);
	if(!sync_)
	{		
		while(!arm_manager_->movementFinished())
		{
			sleep(0.05);
		}
	}
	return 1;
}






