#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>


typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction> TrajClient;

class RobotArm
{
private:
  TrajClient* traj_client_;

public:
  RobotArm() 
  {
    traj_client_ = new TrajClient("/iiwa/PositionJointInterface_trajectory_controller/follow_joint_trajectory/", true);

    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  ~RobotArm()
  {
    delete traj_client_;
  }

  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
  {
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
  }

  control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory()
  {
    //our goal variable
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory msg;

    // ADD THE JOINTS
    msg.joint_names.push_back("iiwa_joint_1");
    msg.joint_names.push_back("iiwa_joint_2");
    msg.joint_names.push_back("iiwa_joint_3");
    msg.joint_names.push_back("iiwa_joint_4");
    msg.joint_names.push_back("iiwa_joint_5");
    msg.joint_names.push_back("iiwa_joint_6");
    msg.joint_names.push_back("iiwa_joint_7");


    //POINTS
    std::vector<trajectory_msgs::JointTrajectoryPoint> points_n(4);

    float  init_pos[] = {-2.537144929704027e-07, -7.401515889071675e-07, 1.323184219081952e-06, 4.7646112566468446e-07, 
                          7.108163151414715e-07, -5.191803369442027e-07, 6.143755681975449e-07};
    
    //point1
    points_n[0].positions.resize(7, 0.0);
    points_n[0].velocities.resize(7, 0.0);
    points_n[0].accelerations.resize(7, 0.0);

    points_n[0].positions[0] = init_pos[0];
    points_n[0].positions[1] = init_pos[1];
    points_n[0].positions[2] = init_pos[2];
    points_n[0].positions[3] = init_pos[3];
    points_n[0].positions[4] = init_pos[4];
    points_n[0].positions[5] = init_pos[5];
    points_n[0].positions[6] = init_pos[6];

    points_n[0].time_from_start = ros::Duration(1.0);

    //point2
    points_n[1].positions.resize(7, 0.0);
    points_n[1].velocities.resize(7, 0.0);
    points_n[1].accelerations.resize(7, 0.0);

    points_n[1].positions[0] = init_pos[0] +0.1;
    points_n[1].positions[1] = init_pos[1] +0.1;
    points_n[1].positions[2] = init_pos[2] + 0.2;
    points_n[1].positions[3] = init_pos[3] + 0.1;
    points_n[1].positions[4] = init_pos[4] + 0.2;
    points_n[1].positions[5] = init_pos[5] + 0.2;
    points_n[1].positions[6] = init_pos[6] + 0.2;

    points_n[1].time_from_start = ros::Duration(3.0);

    //point3
    points_n[2].positions.resize(7, 0.0);
    points_n[2].velocities.resize(7, 0.0);
    points_n[2].accelerations.resize(7, 0.0);

    points_n[2].positions[0] = init_pos[0] + 0.1;
    points_n[2].positions[1] = init_pos[1] + 0.2;
    points_n[2].positions[2] = init_pos[2] + 0.1;
    points_n[2].positions[3] = init_pos[3] + 0.3;
    points_n[2].positions[4] = init_pos[4] + 0.4;
    points_n[2].positions[5] = init_pos[5] + 0.4;
    points_n[2].positions[6] = init_pos[6] + 0.4;

    points_n[2].time_from_start = ros::Duration(3.5);

    //point4
    points_n[3].positions.resize(7, 0.0);
    points_n[3].velocities.resize(7, 0.0);
    points_n[3].accelerations.resize(7, 0.0);

    points_n[3].positions[0] = init_pos[0] - 0.6;
    points_n[3].positions[1] = init_pos[1] + 0.8;
    points_n[3].positions[2] = init_pos[2] + 0.3;
    points_n[3].positions[3] = init_pos[3] + 0.6;
    points_n[3].positions[4] = init_pos[4] + 0.9;
    points_n[3].positions[5] = init_pos[5] + 0.4;
    points_n[3].positions[6] = init_pos[6] + 0.4;

    points_n[3].time_from_start = ros::Duration(9);

    msg.points = points_n;

    goal.trajectory = msg;  

    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }
 
};

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "robot_driver");

  RobotArm arm;
  // Start the trajectory
  arm.startTrajectory(arm.armExtensionTrajectory());
  printf("aqui estamos \n");
  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }
  return 0;
}