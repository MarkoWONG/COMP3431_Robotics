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
	update_timer_ = this->create_wall_timer(10ms, std::bind(&WallFollower::update_callback, this)); //default value is 10ms

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
	uint16_t scan_angle[4] = {0, 90, 50, 270};

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

	cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/
void WallFollower::update_callback()
{
	static uint8_t turtlebot3_state_num = 0;
	double escape_range = 5 * DEG2RAD; // This is the amount the robot turns before changing states
	double frontal_obstacle_threshold = 0.6;
	double right_obstacle_threshold = 0.4;
	double wall_detection_threshold = 0.48;


	switch (turtlebot3_state_num)
	{
		case GET_TB3_DIRECTION:
			if (scan_data_[CENTER] > frontal_obstacle_threshold)
			// This case occurs when the front wall is far from the robot
			{

				if (scan_data_[LEFT_CORNER] < wall_detection_threshold)
				{
					// The robot is too close to the left wall. Turn right.
					RCLCPP_INFO(this->get_logger(), "Too close to left wall. TURNING RIGHT", scan_data_[LEFT]);
					prev_robot_pose_ = robot_pose_;
					turtlebot3_state_num = TB3_RIGHT_TURN;
				}
				else if (scan_data_[LEFT] < wall_detection_threshold)
				{
					// A wall has been detected on the left. Go straight.
					RCLCPP_INFO(this->get_logger(), "Wall detected on the left. GOING STRAIGHT");
					turtlebot3_state_num = TB3_DRIVE_FORWARD;
					// prev_robot_pose_ = robot_pose_;
					// turtlebot3_state_num = TB3_RIGHT_TURN;
				}
				else if (scan_data_[LEFT] > wall_detection_threshold)
				{
					// The left wall has not been detected/too far. Go left.
					RCLCPP_INFO(this->get_logger(), "No left wall detected/too far. TURNING LEFT", scan_data_[LEFT]);
					prev_robot_pose_ = robot_pose_;
					turtlebot3_state_num = TB3_LEFT_TURN;
				}
				
				else if (scan_data_[RIGHT] < right_obstacle_threshold)
					// Robot turns right to avoid obstacles on the right.
				{
					RCLCPP_INFO(this->get_logger(), "too close to right wall. TURNING LEFT");
					prev_robot_pose_ = robot_pose_;
					turtlebot3_state_num = TB3_LEFT_TURN;
				}
				else
				{
					turtlebot3_state_num = TB3_DRIVE_FORWARD;
				}
			}

			if (scan_data_[CENTER] < frontal_obstacle_threshold)
			{
				// There is a wall in front of the robot.
				RCLCPP_INFO(this->get_logger(), "Obstacle in front detected. TURNING SHARP RIGHT");
				prev_robot_pose_ = robot_pose_;
				turtlebot3_state_num = TB3_SHARP_RIGHT;
			}
			break;

		case TB3_DRIVE_FORWARD:
			update_cmd_vel(LINEAR_VELOCITY, 0.0);
			turtlebot3_state_num = GET_TB3_DIRECTION;
			break;

		case TB3_RIGHT_TURN:
			if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range)
			{
				turtlebot3_state_num = GET_TB3_DIRECTION;
			}
			else
			{
				update_cmd_vel(LINEAR_VELOCITY, -1 * ANGULAR_VELOCITY);
			}
			break;

		case TB3_SHARP_RIGHT:
			if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range)
			{
				turtlebot3_state_num = GET_TB3_DIRECTION;
			}
			else
			{
				//To make a sharp right, reduce linear velocity and increase angular velocity
				update_cmd_vel(0, -1 * (ANGULAR_VELOCITY + 0.14 ));
			}
			break;

		case TB3_LEFT_TURN:
			if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range)
			{
				turtlebot3_state_num = GET_TB3_DIRECTION;
			}
			else
			{
				update_cmd_vel(LINEAR_VELOCITY, ANGULAR_VELOCITY);
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
