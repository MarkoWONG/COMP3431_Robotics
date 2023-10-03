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

#ifndef WALL_FOLLOWER_HPP_
#define WALL_FOLLOWER_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

// #define CENTER 0
// #define LEFT   1
// #define LEFT_CORNER 2
// #define RIGHT 3

#define CENTER_RIGHT 0
#define CENTER 1
#define CENTER_LEFT 2
#define LEFT_FRONT 3
#define LEFT_MID 4
#define LEFT 5
#define RIGHT 6
#define BOTTOM_LEFT 7

// #define LINEAR_VELOCITY  0.07
// #define ANGULAR_VELOCITY 0.2

#define GET_TB3_DIRECTION 0
#define TB3_DRIVE_FORWARD 1
#define TB3_RIGHT_TURN    2
#define TB3_LEFT_TURN     3
#define TB3_SHARP_RIGHT   4
#define TB3_REVERSE       5
#define TB3_SHARP_LEFT    6
#define TB3_BOTTOM_LEFT   7

#define PROPORTIONAL_CONSTANT 2.85

class WallFollower : public rclcpp::Node
{
public:
  WallFollower();
  ~WallFollower();

private:
  // ROS topic publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // ROS topic subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Variables
  double robot_pose_;
  double prev_robot_pose_;
  double scan_data_[7];

  double LINEAR_VELOCITY;
  double ANGULAR_VELOCITY;
  bool leftStart;
	double distFromStartTheshold;

  double deviation;

  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Function prototypes
  void update_callback();
  void update_cmd_vel(double linear, double angular);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  bool obstacle_in_front(double frontal_obstacle_threshold);
  bool left_too_close(double side_obstacle_threshold);
  bool left_detected(double wall_detection_threshold);
  bool robot_in_empty_space(double empty_space_threshold, double side_obstacle_threshold);

};
#endif  // TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_
