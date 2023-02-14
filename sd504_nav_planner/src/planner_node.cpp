#include <math.h>
#include <chrono>
#include <ctime>

#include "sd504_nav_planner/CarModelLocal.hpp"
#include "sd504_nav_planner/CarModelGlobal.hpp"
#include "sd504_nav_planner/NavigationUtil.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace senior_design;

class PlannerNode : public rclcpp::Node {

	public:

		PlannerNode() : Node("planner_node") {

			// Create distance map subscriber
			slice_sub_ = this->create_subscription<nvblox_msgs::msg::DistanceMapSlice>(
				"/nvblox_node/map_slice", // Topic
				1, // Queue size
				std::bind(&PlannerNode::map_slice_callback, this, std::placeholders::_1) // Callback function
			);

			// Create odometry subscriber
			odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
				"/visual_slam/tracking/odometry", // Topic
				10, // Queue size
				std::bind(&PlannerNode::odometry_callback, this, std::placeholders::_1)
			);

			// Create goal point subscriber
			goal_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
				"/nav/goal", // Topic
				1, // Queue size
				std::bind(&PlannerNode::goal_point_callback, this, std::placeholders::_1)
			);

			// Create Navigation Util
			nav_util_ = std::make_shared<NavigationUtil>();

			// Create SBMPO objects
			global_sbmpo_ = std::make_shared<sbmpo::SBMPO>();

			// Create global planner object
			global_car_model_ = std::make_shared<CarModelGlobal>(*this, nav_util_);

			// Create local planner object
			local_car_model_ = std::make_shared<CarModelLocal>(*this, nav_util_);

			// Start global planner loop
			timer_global_ = this->create_wall_timer(500ms,
				std::bind(&PlannerNode::global_planner_callback, this));

			// Start local planner loop
			timer_local_ = this->create_wall_timer(100ms,
				std::bind(&PlannerNode::local_planner_callback, this));

			RCLCPP_INFO(this->get_logger(), "Planner node initialized.");
		}

	private:

		void map_slice_callback(const nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr slice) {
			map_slice_ = slice;
			nav_util_->update_map_slice(slice);
		}

		void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
			odom_ = odom;
			nav_util_->update_odometry(odom);
		}

		void goal_point_callback(const geometry_msgs::msg::Point::ConstSharedPtr goal) {
			goal_ = goal;
			nav_util_->update_goal_point(goal);
		}

		void global_planner_callback() {
			if(!global_car_model_->update()) {
				RCLCPP_INFO(this->get_logger(), "Global not initialized.");
				return;
			}
			RCLCPP_INFO(this->get_logger(), "Running global planner...");
			std::clock_t clockStart = std::clock();
			global_sbmpo_->run(*global_car_model_, global_car_model_->parameters());
			std::clock_t clockEnd = std::clock();
			//nav_util_->update_global(*global_sbmpo_);
			RCLCPP_INFO(this->get_logger(), "Global Path:");
			for (sbmpo::State state : global_sbmpo_->state_path()) {
				RCLCPP_INFO(this->get_logger(), "X: %.2f, Y: %.2f", state[0], state[1]);
			}
			RCLCPP_INFO(this->get_logger(), "Ran in %.5f ms", float(clockEnd - clockStart) / CLOCKS_PER_SEC * 1000);
		}

		void local_planner_callback() {
			if (!local_car_model_->update())
				return;
			RCLCPP_INFO(this->get_logger(), "Running local planner...");
			local_sbmpo_->run(*local_car_model_, local_car_model_->parameters());
			nav_util_->update_local(*local_sbmpo_);
				/*
					TODO: Motor outputs
				*/
		}

		// ROS Subscribers
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
		rclcpp::Subscription<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr slice_sub_;
		rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub_;

		// ROS Timers
		rclcpp::TimerBase::SharedPtr timer_local_, timer_global_;

		nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr map_slice_;
		nav_msgs::msg::Odometry::ConstSharedPtr odom_;
		geometry_msgs::msg::Point::ConstSharedPtr goal_;

		// Planner models
		std::shared_ptr<CarModelGlobal> global_car_model_;
		std::shared_ptr<CarModelLocal> local_car_model_;

		// Navigation Util
		std::shared_ptr<NavigationUtil> nav_util_;

		// SBMPO classes
		std::shared_ptr<sbmpo::SBMPO> local_sbmpo_, global_sbmpo_;

};

int main(int argc, char * argv[])
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<PlannerNode>());
	rclcpp::shutdown();
	return 0;
}
