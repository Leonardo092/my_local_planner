#ifndef OPERATIONAL_CHECKER_H_
#define OPERATIONAL_CHECKER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <std_msgs/String.h>
#include <sstream>
#include <cmath>
#include <algorithm>

//variables
geometry_msgs::PoseStamped robot;
move_base_msgs::MoveBaseActionGoal goal;
geometry_msgs::Twist vel;
bool goal_received;

//Callbacks
void RobotPoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
void GoalPoseCB(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg);
void VelocityCB(const geometry_msgs::Twist::ConstPtr& msg);

#endif