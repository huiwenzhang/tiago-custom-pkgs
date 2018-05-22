// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>
#include "geometry_msgs/PoseStamped.h"

// Services headers
#include "moveit_plan_service/CartesianArmPlan.h"

bool cartesianPlan(moveit_plan_service::CartesianArmPlan::Request  &req,
         moveit_plan_service::CartesianArmPlan::Response &res)
{
  ROS_INFO("make sure the input is PoseStamped type");
  geometry_msgs::PoseStamped goal_pose;
  goal_pose = req.goal_pose;

  // goal_pose.header.frame_id = "base_footprint";
  // goal_pose.pose.position.x = 0.4;
  // goal_pose.pose.position.y = -0.3;
  // goal_pose.pose.position.z = 0.26;
  // goal_pose.pose.orientation.w = 1;
  // goal_pose.pose.orientation.x = 0;
  // goal_pose.pose.orientation.y = 0;
  // goal_pose.pose.orientation.z = 0;


  std::vector<std::string> torso_arm_joint_names;
  //select group of joints
  moveit::planning_interface::MoveGroup group_arm_torso("arm_torso");
  //choose your preferred planner
  group_arm_torso.setPlannerId("SBLkConfigDefault");
  group_arm_torso.setPoseReferenceFrame("base_footprint");
  group_arm_torso.setPoseTarget(goal_pose);

  ROS_INFO_STREAM("Planning to move " <<
                  group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                  group_arm_torso.getPlanningFrame());

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1.0);


  moveit::planning_interface::MoveGroup::Plan my_plan;
  //set maximum time to find a plan
  group_arm_torso.setPlanningTime(5.0);
  ros::Duration(5).sleep();
  bool success = group_arm_torso.plan(my_plan);
  ROS_INFO("Plan found....");
  res.success = success;

  if ( !success )
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

  // Execute the plan
  ros::Time start = ros::Time::now();

  moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();
  
  if (!bool(e))
    throw std::runtime_error("Error executing plan");

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
  return res.success;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cartesian_arm_plan_server");
  ros::NodeHandle n;
  ros::Rate rate(1);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::ServiceServer service = n.advertiseService("cartesian_arm_plan", cartesianPlan);
  ROS_INFO("Ready to plan using trac-ik package in cartesian space.");

  while(ros::ok())
  {
    rate.sleep();
  }
  return 0;
}