/*
visualize the end effector trajectory online with rviz markers
*/
// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>
#include "geometry_msgs/PoseStamped.h"

#include <visualization_msgs/Marker.h>

#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "visual_ee_traj");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("ee_trajectory", 5);

  ros::Rate r(30);

  // return end effector pose using moveit
  moveit::planning_interface::MoveGroup *group;
  group = new moveit::planning_interface::MoveGroup("arm_torso");
  group->setPoseReferenceFrame("base_footprint");
  ROS_INFO_STREAM("Planning to move " <<
                  group->getEndEffectorLink() );

  visualization_msgs::Marker points, line_strip;
  points.header.frame_id = line_strip.header.frame_id =  "/base_footprint";
  points.header.stamp = line_strip.header.stamp = ros::Time::now();
  points.ns = line_strip.ns =  "points_and_lines";
  points.action = line_strip.action =  visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w =  1.0;

  points.id = 0;
  line_strip.id = 1;

  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.2;
  points.scale.y = 0.2;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1;


  // Points are green
  points.color.g = 1.0f;
  points.color.a = 1.0;

  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  while (ros::ok())
  {

    // group->setEndEffectorLink("hand_left");
    geometry_msgs::PoseStamped current_pose =  group->getCurrentPose();
    geometry_msgs::Point p = current_pose.pose.position;


    points.points.push_back(p);
    line_strip.points.push_back(p);

    marker_pub.publish(points);
    marker_pub.publish(line_strip);

    r.sleep();

  }
}
