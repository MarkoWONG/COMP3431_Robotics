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

// Wallfollower Node Constructor
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
	scan_data_[4] = 0.0;
	scan_data_[5] = 0.0;
	scan_data_[6] = 0.0;
	scan_data_[7] = 0.0;

	// pose = the position and orientation of an object
	robot_pose_ = 0.0;
	prev_robot_pose_ = 0.0;

	LINEAR_VELOCITY = 0.08;
	ANGULAR_VELOCITY = 0.22;
	distFromStartTheshold = 2;
	distFromStartThesholdStop = 0.6;
	leftStart = false;
	STOP = false;

	startingX = 3.2;
	startingY = 43;

	deviation = 0.0; // The higher the distance deviated, the more quickly the robot turns

	/************************************************************
	** Initialise ROS publishers and subscribers
	************************************************************/
	auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

	// Initialise publishers
	cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);
	cylinder_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("detected_cylinders", qos);

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

	// Initialise wallfollower vars
	leftStart = false;
	needStartingPos = true;

	RCLCPP_INFO(this->get_logger(), "Wall follower node has been initialised");
}

// Wallfollower Node Destructor
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

	// Set starting position of robot
	if (needStartingPos == true){
		startingX = msg->pose.pose.position.x;
		startingY = msg->pose.pose.position.y;
		std::string debugline3 = "Starting position x: ";
		debugline3.append(std::to_string(startingX));
		debugline3.append(", Y: ");
		debugline3.append(std::to_string(startingY));
		RCLCPP_INFO(this->get_logger(), debugline3);
		needStartingPos = false;
	}

	// Check if the robot has returned to the start pose
	double distance = sqrt(pow(msg->pose.pose.position.x - startingX, 2) +
							pow(msg->pose.pose.position.y - startingY, 2));
	
	std::string debugline1 = "distance from the start is: ";
	debugline1.append(std::to_string(distance));
	RCLCPP_INFO(this->get_logger(), debugline1);

	// std::string debugline2 = "x: ";
	// debugline2.append(std::to_string(msg->pose.pose.position.x));
	// debugline2.append("y: ");
	// debugline2.append(std::to_string(msg->pose.pose.position.y));
	// RCLCPP_INFO(this->get_logger(), debugline2);
	
	// Once robot is X distance away from start we can check when to stop robot
	if (distance > distFromStartTheshold) {
		
		leftStart = true;
	}
	else if (leftStart == true && distance <= distFromStartThesholdStop){
		RCLCPP_INFO(this->get_logger(), "STOPPING ROBOT");
		update_cmd_vel(0,0);
		STOP = true;
	};
}


void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	//Center right, center, center left, left front, left mid, left, right
	uint16_t scan_angle[8] = {340, 0, 20, 35, 55, 90, 270, 145};

	for (int num = 0; num < 8; num++)
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
	// Convert scan data to Cartesian coordinates
    std::vector<std::pair<float, float>> cartesian_coords = convertToCartesian(msg);

    // Convert pairs to Point structure for ransacCircleFitting
    std::vector<Point> points;
    for (const auto& coord : cartesian_coords) {
        points.push_back({coord.first, coord.second});
    }
	
	// Detect circles using RANSAC
    int iterations = 100; // Adjust as needed
    float distance_threshold = 0.1; // Adjust as needed
    std::vector<Circle> detected_circles = ransacCircleFitting(points, iterations, distance_threshold);


    // Create a PoseArray message
    visualization_msgs::msg::MarkerArray cylinder_poses;

    // Fill the PoseArray message with the detected cylinder positions
    for (const auto& circle : detected_circles) {
        visualization_msgs::msg::Marker marker;
		
		marker.header.frame_id = "map"; // Set the appropriate frame ID
		marker.id = detected_circles.size() + 1;
		marker.type = marker.CYLINDER;
		marker.action = marker.ADD;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.pose.position.x = circle.center.x;
		marker.pose.position.y = circle.center.y;
		marker.pose.position.z = 0;
		marker.scale.x = 0.14; 
		marker.scale.y = 0.14;
		marker.scale.z = 0.2; 
		marker.color.a = 1.0;
		// if pink_on_top is False:
		// rgb = (255,192,203)
		// elif color == self.YELLOW:
		// rgb = (255,234,0)
		// elif color == self.BLUE:
		// rgb = (0,191,255)
		// else:
		// rgb = (0,100,0)
		// make pink
		marker.color.r = 255.0 / 255.0;
		marker.color.g = 192 / 255.0;
		marker.color.b = 203 / 255.0;
		RCLCPP_INFO(this->get_logger(), "Detected circle at x: %f, y: %f, radius: %f", circle.center.x, circle.center.y, circle.radius);

        cylinder_poses.markers.push_back(marker);
    }

    // Publish the PoseArray message
    cylinder_pub_->publish(cylinder_poses);
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
	double escape_range = 1.25 * DEG2RAD; // This is the amount the robot turns before changing states
	double frontal_obstacle_threshold = 0.35;
	double side_obstacle_threshold = 0.29;
	double wall_detection_threshold = 0.32;
	double empty_space_threshold = 0.34;
	//double reverse_threshold = 0.1; // if the robot is closer than the reverse threshold, it will reverse to avoid obstacles.


	// std::string test2 = "vel is: ";
	// test2.append(std::to_string(LINEAR_VELOCITY));
	// test2.append("command is: ");
	// test2.append(std::to_string(turtlebot3_state_num));
	// RCLCPP_INFO(this->get_logger(), test2);
	if (!STOP){
	switch (turtlebot3_state_num)
	{
		case GET_TB3_DIRECTION:
			if (!obstacle_in_front(frontal_obstacle_threshold))
			// This case occurs when the front wall is far from the robot
			{

				if (left_too_close(side_obstacle_threshold))
				{
					// The robot is too close to the left wall. Turn right.
					RCLCPP_INFO(this->get_logger(), "Too close to left wall. TURNING RIGHT", scan_data_[LEFT]);
					prev_robot_pose_ = robot_pose_;
					turtlebot3_state_num = TB3_RIGHT_TURN;

					double temp = fmin(scan_data_[LEFT], cos(55 * DEG2RAD) * scan_data_[LEFT_FRONT]);
					double left_dist = fmin(temp, cos(35 * DEG2RAD) * scan_data_[LEFT_MID]);
					deviation = fabs(left_dist - wall_detection_threshold);
				}
				
				else if (left_detected(wall_detection_threshold))
				{
					// A wall has been detected on the left. Go straight.
					RCLCPP_INFO(this->get_logger(), "Wall detected on the left. GOING STRAIGHT");
					turtlebot3_state_num = TB3_DRIVE_FORWARD;
					// prev_robot_pose_ = robot_pose_;
					// turtlebot3_state_num = TB3_RIGHT_TURN;
				}
				else if (!left_detected(wall_detection_threshold))
				{
					// The left wall has not been detected/is too far. 

					//if the robot is far from anything, it will go straight
					if (robot_in_empty_space(empty_space_threshold)) //, side_obstacle_threshold
					{
						// RCLCPP_INFO(this->get_logger(), "No wall detected ahead or left. GOING STRAIGHT");
						// turtlebot3_state_num = TB3_DRIVE_FORWARD;
						RCLCPP_INFO(this->get_logger(), "No wall detected ahead or left. TURNING SHARP LEFT");
						prev_robot_pose_ = robot_pose_;
						turtlebot3_state_num = TB3_SHARP_LEFT;
					}
					else {
						RCLCPP_INFO(this->get_logger(), "Left wall is too far. TURNING LEFT");
						prev_robot_pose_ = robot_pose_;
						turtlebot3_state_num = TB3_LEFT_TURN;

						double temp = fmax(scan_data_[LEFT], cos(55 * DEG2RAD) * scan_data_[LEFT_FRONT]);
						double left_dist = fmax(temp, cos(35 * DEG2RAD) * scan_data_[LEFT_MID]);
						deviation = fabs(left_dist - wall_detection_threshold);
					}	
				}
				
				else if (scan_data_[RIGHT] < side_obstacle_threshold)
					// Robot turns right to avoid obstacles on the right.
				{
					RCLCPP_INFO(this->get_logger(), "too close to right wall. TURNING LEFT");
					prev_robot_pose_ = robot_pose_;
					turtlebot3_state_num = TB3_LEFT_TURN;

					deviation = fabs(scan_data_[RIGHT] - side_obstacle_threshold);
				}
				else
				{
					turtlebot3_state_num = TB3_DRIVE_FORWARD;
				}
			}

			if (obstacle_in_front(frontal_obstacle_threshold))
			{
				// There is a wall in front of the robot.
				if (!left_detected(wall_detection_threshold)) {
					RCLCPP_INFO(this->get_logger(), "Obstacle in the front but no wall on LEFT");
					prev_robot_pose_ = robot_pose_;
					turtlebot3_state_num = TB3_LEFT_TURN;
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
			update_cmd_vel(-LINEAR_VELOCITY, ANGULAR_VELOCITY);
			turtlebot3_state_num = GET_TB3_DIRECTION;
			break;
		
		case TB3_RIGHT_TURN:
			if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range)
			{
				turtlebot3_state_num = GET_TB3_DIRECTION;
			}
			else
			{
				update_cmd_vel(LINEAR_VELOCITY, -1 * (ANGULAR_VELOCITY + 0.26 ) * PROPORTIONAL_CONSTANT * deviation);
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
				update_cmd_vel(0, -1 * (ANGULAR_VELOCITY + 0.18 ));
			}
			break;

		case TB3_LEFT_TURN:
			if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range)
			{
				turtlebot3_state_num = GET_TB3_DIRECTION;
			}
			else
			{
				update_cmd_vel(LINEAR_VELOCITY, ANGULAR_VELOCITY * PROPORTIONAL_CONSTANT * deviation);
			}
			break;
		
		case TB3_SHARP_LEFT:
			if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range)
			{
				turtlebot3_state_num = GET_TB3_DIRECTION;
			}
			else
			{
				//To make a sharp left, reduce linear velocity and increase angular velocity
				update_cmd_vel(LINEAR_VELOCITY / 2,ANGULAR_VELOCITY + 0.2);
			}
			break;
		

		default:
			turtlebot3_state_num = GET_TB3_DIRECTION;
			break;
	}
	}
}

bool WallFollower::obstacle_in_front(double frontal_obstacle_threshold)
{
	return (
		scan_data_[CENTER] < frontal_obstacle_threshold || 
		cos(20 * DEG2RAD) * scan_data_[CENTER_LEFT] < frontal_obstacle_threshold || 
		cos(20 * DEG2RAD) * scan_data_[CENTER_RIGHT] < frontal_obstacle_threshold
	);
}

bool WallFollower::left_too_close(double side_obstacle_threshold)
{
	return (
		scan_data_[LEFT] < side_obstacle_threshold || 
		cos(55 * DEG2RAD) * scan_data_[LEFT_FRONT] < side_obstacle_threshold || 
		cos(35 * DEG2RAD) * scan_data_[LEFT_MID] < side_obstacle_threshold
	);
}

bool WallFollower::left_detected(double wall_detection_threshold)
{
	return (
		scan_data_[LEFT] < wall_detection_threshold || 
		cos(55 * DEG2RAD) * scan_data_[LEFT_FRONT] < wall_detection_threshold || 
		cos(35 * DEG2RAD) * scan_data_[LEFT_MID] < wall_detection_threshold
	);
}

bool WallFollower::robot_in_empty_space(double empty_space_threshold) 
//, double side_obstacle_threshold
{
	// return (!obstacle_in_front(empty_space_threshold) && !left_detected(empty_space_threshold) && (cos(55 * DEG2RAD) * scan_data_[BOTTOM_LEFT] < side_obstacle_threshold));
	return (!obstacle_in_front(empty_space_threshold) && !left_detected(empty_space_threshold));

}

std::vector<std::pair<float, float>> WallFollower::convertToCartesian(const sensor_msgs::msg::LaserScan::SharedPtr& msg) {
    std::vector<std::pair<float, float>> cartesian_coords;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float angle = msg->angle_min + i * msg->angle_increment;
        float range = msg->ranges[i];
        if (!std::isinf(range) && !std::isnan(range)) {
            float x = range * cos(angle);
            float y = range * sin(angle);
            cartesian_coords.emplace_back(x, y);
        }
    }
    return cartesian_coords;
}

// Function to calculate the circle given three points
// This is a geometric approach to find the circumcenter of a triangle formed by three points
Circle WallFollower::calculateCircle(const Point &p1, const Point &p2, const Point &p3) {
    float ax = p1.x, ay = p1.y;
    float bx = p2.x, by = p2.y;
    float cx = p3.x, cy = p3.y;

    float ox = (std::min({ax, bx, cx}) + std::max({ax, bx, cx})) / 2;
    float oy = (std::min({ay, by, cy}) + std::max({ay, by, cy})) / 2;
    ax -= ox; ay -= oy;
    bx -= ox; by -= oy;
    cx -= ox; cy -= oy;

    float d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    if (std::abs(d) < std::numeric_limits<float>::epsilon()) {
        return { {0, 0}, 0 };
    }

    float x = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d;
    float y = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d;
    float r = std::sqrt(x * x + y * y + ((ax - bx) * (ax - bx) + (ay - by) * (ay - by)) / 2);

    x += ox;
    y += oy;

    return { {x, y}, r };
}

// RANSAC for Circle Fitting
// distance_threshold should be in meters (depends on polar coordinates)
// iterations should be at least 1 - no. of times algo will run - more iterations = more accuracy + computational time
std::vector<Circle> WallFollower::ransacCircleFitting(const std::vector<Point>& points, int iterations, float distance_threshold) {
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_int_distribution<size_t> uni(0, points.size() - 1);

    std::vector<Circle> detected_circles;
    int best_inliers = 0;
    Circle best_circle;

    for (int it = 0; it < iterations; ++it) {
        // Randomly select 3 points
        Point p1 = points[uni(rng)];
        Point p2 = points[uni(rng)];
        Point p3 = points[uni(rng)];

        Circle circle = calculateCircle(p1, p2, p3);

        int inliers = 0;
        for (const auto& p : points) {
            float dist = std::sqrt(std::pow(p.x - circle.center.x, 2) + std::pow(p.y - circle.center.y, 2));
            if (std::abs(dist - circle.radius) < distance_threshold) {
                ++inliers;
            }
        }

        if (inliers > best_inliers) {
            best_inliers = inliers;
            best_circle = circle;
        }
    }

    if (best_inliers > 0) {
        detected_circles.push_back(best_circle);
    }

    return detected_circles;
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