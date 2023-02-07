#include <math.h>
#include <chrono>

#include "senior_design_504/CarModelLocal.hpp"
#include "senior_design_504/CarModelGlobal.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nvblox_msgs/msg/distance_map_slice.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class PlannerNode : public rclcpp::Node {

	public:

		PlannerNode() : Node("planner_node") {

			this->declare_parameter("global_update_rate");
			this->declare_parameter("local_update_rate");
			this->declare_parameter("global_max_iterations");
			this->declare_parameter("local_max_iterations");
			this->declare_parameter("global_max_generations");
			this->declare_parameter("local_max_generations");
			this->declare_parameter("global_sample_time");
			this->declare_parameter("local_sample_time");
			this->declare_parameter("global_grid_states");
			this->declare_parameter("local_grid_states");
			this->declare_parameter("global_grid_resolution");
			this->declare_parameter("local_grid_resolution");
			this->declare_parameter("global_controls");
			this->declare_parameter("local_controls");
			this->declare_parameter("global_plan_div_point");

			global_update_rate_ = this->get_parameter("global_update_rate").as_double();
			local_update_rate_ = this->get_parameter("global_update_rate").as_double();
			global_max_iterations_ = this->get_parameter("global_max_iterations").as_int();
			local_max_iterations_ = this->get_parameter("local_max_iterations").as_int();
			global_max_generations_ = this->get_parameter("global_max_generations").as_int();
			local_max_generations_ = this->get_parameter("local_max_generations").as_int();
			global_sample_time_ = this->get_parameter("global_sample_time").as_double();
			local_sample_time_ = this->get_parameter("local_sample_time").as_double();
			/*
				TODO: REST OF PARAMS
			*/

			RCLCPP_INFO(this->get_logger(), "Planner node initialized.");

			// Create distance map subscriber
			slice_sub_ = this->create_subscription<nvblox_msgs::msg::DistanceMapSlice>(
				"/nvblox_node/map_slice", // Topic
				1, // Queue size
				std::bind(&PlannerNode::map_slice_callback, this, std::placeholders::_1) // Callback function
			);

			// Create odometry subscriber
			odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
				"/nvblox_node/visual_slam/tracking/odometry", // Topic
				1, // Queue size
				std::bind(&PlannerNode::odometry_callback, this, std::placeholders::_1)
			);

			// Create local planner object
			global_car_model_ = std::make_shared<senior_design::CarModelGlobal>(slice_);

			// Create local planner object
			local_car_model_ = std::make_shared<senior_design::CarModelLocal>(slice_);

			// Start global planner loop
			timer_global_ = this->create_wall_timer(500ms,
				std::bind(&PlannerNode::global_planner_callback, this));

			// Start local planner loop
			timer_local_ = this->create_wall_timer(100ms,
				std::bind(&PlannerNode::local_planner_callback, this));

		}

	private:

		void map_slice_callback(const nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr slice) {
			slice_ = slice;
		}

		void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
			odom_ = odom;
		}

		void global_planner_callback() {
			/*
				RUN GLOBAL PLANNER
			*/
		}

		void local_planner_callback() {
			/*
				RUN LOCAL PLANNER
			*/
		}

		// ROS Subscribers
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
		rclcpp::Subscription<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr slice_sub_;

		// ROS Timers
		rclcpp::TimerBase::SharedPtr timer_local_;
		rclcpp::TimerBase::SharedPtr timer_global_;

		// ROS Msg variables
		geometry_msgs::msg::Point::ConstSharedPtr goal_;
		nav_msgs::msg::Odometry::ConstSharedPtr odom_;
		nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr slice_;

		// Planner models
		std::shared_ptr<senior_design::CarModelGlobal> global_car_model_;
		std::shared_ptr<senior_design::CarModelLocal> local_car_model_;

		// Parameters
		std::chrono::milliseconds global_update_rate_, local_update_rate_;
		int global_max_iterations_, local_max_iterations_;
		int global_max_generations_, local_max_generations_;
		float global_sample_time_, local_sample_time_;
		std::vector<bool> global_grid_states_, local_grid_states_;
		std::vector<float> global_grid_resolution_, local_grid_resolution_;
		std::vector<sbmpo::Control> global_controls_, local_controls;
		int global_plan_div_point_;

};

int main(int argc, char * argv[])
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<PlannerNode>());
	rclcpp::shutdown();
	return 0;
}
