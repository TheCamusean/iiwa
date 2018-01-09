#include "ros/ros.h"
#include "iiwa/GetContainerPose.h"
#include "iiwa/GetPiecePose.h"

//Geometry_msgs
#include "geometry_msgs/Pose.h"


bool contpose(iiwa::GetContainerPose::Request  &req,
         iiwa::GetContainerPose::Response &res)
{
  res.found = true;

  geometry_msgs::Pose cont_pose;
  cont_pose.position.x = 0.5;
  cont_pose.position.y = 0;
  cont_pose.position.z = 0.4;
  cont_pose.orientation.x = 1;
  cont_pose.orientation.y = 0;
  cont_pose.orientation.z = 0;
  cont_pose.orientation.w = 0;
  res.container_pose = cont_pose;

  return true;
}

bool piecepose(iiwa::GetPiecePose::Request  &req,
         iiwa::GetPiecePose::Response &res)
{
  geometry_msgs::Pose piece_pose;
  piece_pose.position.x = 0.6;
  piece_pose.position.y = 0;
  piece_pose.position.z = 0.2;
  piece_pose.orientation.x = 1;
  piece_pose.orientation.y = 0;
  piece_pose.orientation.z = 0;
  piece_pose.orientation.w = 0;
  res.piece_pose = piece_pose;

  res.piece_error = false;
  res.capture_error = false;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "CameraServers");
  ros::NodeHandle n;

  ros::ServiceServer container_service = n.advertiseService("GetContainerPose", contpose);
  ros::ServiceServer piece_service = n.advertiseService("GetPiecePose", piecepose);
  
  ROS_INFO("Cam Services ready");
  ros::spin();

  return 0;
}