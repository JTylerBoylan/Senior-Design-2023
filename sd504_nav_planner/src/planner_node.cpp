#include <math.h>
#include <chrono>
#include <ctime>

#include "sd504_nav_planner/CarModelLocal.hpp"
#include "sd504_nav_planner/CarModelGlobal.hpp"
#include "sd504_nav_planner/NavigationPlanner.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace senior_design;

class PlannerNode : public rclcpp::Node {

	public:

		PlannerNode() : Node("planner_node") {

			// Create Navigation Util
			planner_ = std::make_shared<NavigationPlanner>(this);

			// Create distance map subscriber
			slice_sub_ = this->create_subscription<nvblox_msgs::msg::DistanceMapSlice>(
				"/nvblox_node/map_slice", // Topic
				10, // Queue size
				std::bind(&PlannerNode::map_slice_callback, this, std::placeholders::_1) // Callback function
			);

			// Create odometry subscriber
			odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
				"/visual_slam/tracking/odometry", // Topic
				10, // Queue size
				std::bind(&PlannerNode::odometry_callback, this, std::placeholders::_1) // Callback function
			);

			// Create goal point subscriber
			goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
				"/nav/goal", // Topic
				10, // Queue size
				std::bind(&PlannerNode::goal_point_callback, this, std::placeholders::_1) // Callback function
			);

			// Create path publisher
			path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/nav/path/global", 10);

			// Start global planner loop
			timer_global_ = this->create_wall_timer(500ms,
				std::bind(&PlannerNode::global_planner_callback, this));

			// Start local planner loop
			timer_local_ = this->create_wall_timer(500ms,
				std::bind(&PlannerNode::local_planner_callback, this));

			RCLCPP_INFO(this->get_logger(), "Planner node initialized.");
		}

	private:

		void map_slice_callback(const nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr slice) {
			NavigationUtil::distance_map_slice = slice;
		}

		void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
			NavigationUtil::odometry = odom;
		}

		void goal_point_callback(const geometry_msgs::msg::PointStamped::ConstSharedPtr goal) {
			NavigationUtil::goal_point = goal;
		}

		void global_planner_callback() {
			planner_->run_global();
			path_pub_->publish(planner_->global_path());
		}

		void local_planner_callback() {
			planner_->run_local();
		}

		// ROS Subscribers
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
		rclcpp::Subscription<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr slice_sub_;
		rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;

		// ROS Publishers
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

		// ROS Timers
		rclcpp::TimerBase::SharedPtr timer_local_, timer_global_;

		// Navigation Planner
		std::shared_ptr<NavigationPlanner> planner_;

};

int main(int argc, char * argv[])
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<PlannerNode>());
	rclcpp::shutdown();
	return 0;
}
