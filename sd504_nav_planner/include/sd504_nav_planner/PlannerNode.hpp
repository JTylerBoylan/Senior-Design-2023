#ifndef SD504_PLANNER_NODE_HPP_
#define SD504_PLANNER_NODE_HPP_

#include <chrono>
#include <ctime>

#include <sd504_nav_planner/NavigationPlanner.hpp>

namespace senior_design {

using namespace std::chrono_literals;

class PlannerNode : public rclcpp::Node {

	public:

	PlannerNode() : Node("planner_node") {

		// Planner object
		planner_ = std::make_shared<NavigationPlanner>(this);

		// Subscribers
		slice_sub_ = this->create_subscription<nvblox_msgs::msg::DistanceMapSlice>(
			"/nvblox_node/map_slice", // Topic
			10, // Queue size
			std::bind(&NavigationPlanner::update_map, planner_, std::placeholders::_1) // Callback function
		);
		odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
			"/visual_slam/tracking/odometry",
			10,
			std::bind(&NavigationPlanner::update_state, planner_, std::placeholders::_1)
		);
		goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
			"/nav/goal/global",
			10,
			std::bind(&NavigationPlanner::update_goal, planner_, std::placeholders::_1)
		);
		encoder_sub_ = this->create_subscription<std_msgs::msg::Int8>(
			"/motors/encoder",
			10,
			std::bind(&NavigationPlanner::update_steering_angle, planner_, std::placeholders::_1)
		);

		// Publishers
		global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/nav/path/global", 10);
		local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/nav/path/local", 10);
		local_goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/nav/goal/local", 10);

		steer_motor_pub_ = this->create_publisher<std_msgs::msg::Int8>("/motors/steering_angle", 10);
		drive_motor_pub_ = this->create_publisher<std_msgs::msg::Int8>("/motors/drive", 10);

		// Plan Timer
		planner_timer_ = this->create_wall_timer(150ms,
			std::bind(&PlannerNode::planner_callback, this));

		RCLCPP_INFO(this->get_logger(), "Planner node initialized.");
	}

	private:

	void planner_callback() {
		const int plan = planner_->plan();

		if (plan == 0) {
			// No plan found, publish empty messages
			global_path_pub_->publish(nav_msgs::msg::Path());
			local_goal_pub_->publish(geometry_msgs::msg::PointStamped());
			local_path_pub_->publish(nav_msgs::msg::Path());
			steer_motor_pub_->publish(std_msgs::msg::Int8());
			drive_motor_pub_->publish(std_msgs::msg::Int8());
		} else {
			// Plan found, publish global and local paths and control commands
			global_path_pub_->publish(planner_->global_path());

			if (plan == 1) {
				// Only global plan available, publish empty messages for local plan and control commands
				local_goal_pub_->publish(geometry_msgs::msg::PointStamped());
				local_path_pub_->publish(nav_msgs::msg::Path());
				steer_motor_pub_->publish(std_msgs::msg::Int8());
				drive_motor_pub_->publish(std_msgs::msg::Int8());
			} else {
				// Local plan available, publish local goal, local path, and control commands
				local_goal_pub_->publish(planner_->local_goal_point());
				local_path_pub_->publish(planner_->local_path());
				steer_motor_pub_->publish(planner_->next_turn_angle());
				drive_motor_pub_->publish(planner_->next_drive_acceleration());
			}
		}
	}


	// ROS Subscribers
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Subscription<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr slice_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
	rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr encoder_sub_;

	// ROS Publishers
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr local_goal_pub_;

	// Controls publisher
	rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr steer_motor_pub_;
	rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr drive_motor_pub_;

	// ROS Timers
	rclcpp::TimerBase::SharedPtr planner_timer_;

	// Navigation Planner
	std::shared_ptr<NavigationPlanner> planner_;

};

}

#endif