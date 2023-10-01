// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim
//
// Modified by Claude Sammut for COMP3431
// Use this code as the basis for a wall follower

#include "wall_follower/wall_follower.hpp"

#include <memory>

using namespace std::chrono_literals;

WallFollower::WallFollower()
: Node("wall_follower_node")
{
	/************************************************************
	** Initialise variables
	************************************************************/
	scan_data_[0] = 0.0;
	scan_data_[1] = 0.0;
	scan_data_[2] = 0.0;
	scan_data_[3] = 0.0;

	robot_pose_ = 0.0;
	prev_robot_pose_ = 0.0;

	deviation = 0.0; // The higher the distance deviated, the quicker the robot turns

	/************************************************************
	** Initialise ROS publishers and subscribers
	************************************************************/
	auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

	// Initialise publishers
	cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

	// Initialise subscribers
	scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"scan", \
		rclcpp::SensorDataQoS(), \
		std::bind(
			&WallFollower::scan_callback, \
			this, \
			std::placeholders::_1));
	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom", qos, std::bind(&WallFollower::odom_callback, this, std::placeholders::_1));

	/************************************************************
	** Initialise ROS timers
	************************************************************/
	update_timer_ = this->create_wall_timer(10ms, std::bind(&WallFollower::update_callback, this));

	RCLCPP_INFO(this->get_logger(), "Wall follower node has been initialised");
}

WallFollower::~WallFollower()
{
	RCLCPP_INFO(this->get_logger(), "Wall follower node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void WallFollower::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	tf2::Quaternion q(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	robot_pose_ = yaw;
}

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	uint16_t scan_angle[4] = {0, 90, 45, 270};

	for (int num = 0; num < 4; num++)
	{
		if (std::isinf(msg->ranges.at(scan_angle[num])))
		{
			scan_data_[num] = msg->range_max;
		}
		else
		{
			scan_data_[num] = msg->ranges.at(scan_angle[num]);
		}
	}
}

void WallFollower::update_cmd_vel(double linear, double angular)
{
	geometry_msgs::msg::Twist cmd_vel;
	cmd_vel.linear.x = linear;
	cmd_vel.angular.z = angular;

	if (cmd_vel.angular.z > 1) {
		RCLCPP_INFO(this->get_logger(), "%lf", cmd_vel.angular.z);
	}

	cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/
void WallFollower::update_callback()
{
	static uint8_t turtlebot3_state_num = 0;
	// This is the amount the robot turns before changing state.
	double escape_range = 5.0 * DEG2RAD; // Change this number!!!
	double min_wall_dist = 0.1; 
	double min_wall_dist_left = 0.3;
	double min_wall_dist_front = 0.3;
	double max_wall_dist_react = 0.8;
	double reverse_dist = 0.1;


	switch (turtlebot3_state_num)
	{
		case GET_TB3_DIRECTION:
			
			// No Wall in the front (Front Wall is far away)
			if (scan_data_[CENTER] > min_wall_dist_front)
			{
				// Too close to the left wall, turn right
				if (scan_data_[LEFT] < min_wall_dist_left || cos(45 * DEG2RAD) * scan_data_[LEFT_TOP_CORNER] < min_wall_dist_left) 
				{
					RCLCPP_INFO(this->get_logger(), "Too close to the LEFT WALL, Turn Right");
					prev_robot_pose_ = robot_pose_;
					turtlebot3_state_num = TB3_RIGHT_TURN;

					deviation = fabs(scan_data_[LEFT_TOP_CORNER] - min_wall_dist);
				}
				// A Left Wall with Normal distance
				if (scan_data_[LEFT] < min_wall_dist_left)
				{
					RCLCPP_INFO(this->get_logger(), "A LEFT WALL with normal distance, Go Straight");
					turtlebot3_state_num = TB3_DRIVE_FORWARD;
				}
				else if (scan_data_[LEFT] > min_wall_dist_left || cos(45 * DEG2RAD) * scan_data_[LEFT_TOP_CORNER] > min_wall_dist_left)
				{
					// The left wall has not been detected/too far. 

					//if the robot is far from anything, it will go straight
					if (scan_data_[CENTER] > max_wall_dist_react && scan_data_[LEFT] > max_wall_dist_react)
					{
						RCLCPP_INFO(this->get_logger(), "No wall detected ahead or left. GOING STRAIGHT");
						turtlebot3_state_num = TB3_DRIVE_FORWARD;
					}
					
					else {
						RCLCPP_INFO(this->get_logger(), "Left wall is too far. TURNING LEFT");
						prev_robot_pose_ = robot_pose_;
						turtlebot3_state_num = TB3_LEFT_TURN;
						deviation = fabs(scan_data_[LEFT] - min_wall_dist);
					}
				}
				else if (scan_data_[RIGHT] < min_wall_dist)
					// Robot turns right to avoid obstacles on the right.
				{
					RCLCPP_INFO(this->get_logger(), "too close to right wall. TURNING LEFT");
					prev_robot_pose_ = robot_pose_;
					turtlebot3_state_num = TB3_LEFT_TURN;

					deviation = fabs(scan_data_[RIGHT] - min_wall_dist);
				}
				else
				{
					turtlebot3_state_num = TB3_DRIVE_FORWARD;
				}
			}

			if (scan_data_[CENTER] < min_wall_dist_front)
			{
				// There is a wall in front of the robot.
				if (scan_data_[CENTER] < reverse_dist)
				{
					RCLCPP_INFO(this->get_logger(), "Obstacle in front is very close. REVERSING");
					prev_robot_pose_ = robot_pose_;
					turtlebot3_state_num = TB3_REVERSE;
				} 

				else 
				{
					RCLCPP_INFO(this->get_logger(), "Obstacle in front detected. TURNING SHARP RIGHT");
					prev_robot_pose_ = robot_pose_;
					turtlebot3_state_num = TB3_SHARP_RIGHT;
				}
				
			}
			break;

		case TB3_DRIVE_FORWARD:
			update_cmd_vel(LINEAR_VELOCITY, 0.0);
			turtlebot3_state_num = GET_TB3_DIRECTION;
			break;

		case TB3_REVERSE:
			update_cmd_vel(-LINEAR_VELOCITY, -ANGULAR_VELOCITY);
			turtlebot3_state_num = GET_TB3_DIRECTION;
			break;

		case TB3_RIGHT_TURN:
			if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range)
			{
				turtlebot3_state_num = GET_TB3_DIRECTION;
			}
			else
			{
				update_cmd_vel(LINEAR_VELOCITY, -1 * ANGULAR_VELOCITY * PROPORTIONAL_CONSTANT * deviation);
			}
			break;

		case TB3_SHARP_RIGHT:
			//if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range)
			//{
			//	turtlebot3_state_num = GET_TB3_DIRECTION;
			//}
			//else
			//{
				//To make a sharp right, reduce linear velocity and increase angular velocity
				update_cmd_vel(0, -1 * (ANGULAR_VELOCITY + 0.14 ));
			//}
			break;

		case TB3_LEFT_TURN:
			if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range)
			{
				turtlebot3_state_num = GET_TB3_DIRECTION;
			}
			else
			{
				update_cmd_vel(LINEAR_VELOCITY, ANGULAR_VELOCITY*PROPORTIONAL_CONSTANT * deviation);
			}
			break;

		default:
			turtlebot3_state_num = GET_TB3_DIRECTION;
			break;
	}
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WallFollower>());
	rclcpp::shutdown();

	return 0;
}
