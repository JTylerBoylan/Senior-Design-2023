#include <math.h>
#include <chrono>

#include "senior_design_504/CarModelLocal.hpp"
#include "senior_design_504/CarModelGlobal.hpp"
#include "senior_design_504/sd_util.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nvblox_msgs/msg/distance_map_slice.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;
using namespace senior_design;

class PlannerNode : public rclcpp::Node {

	public:

		PlannerNode() : Node("planner_node") {

			// Create global planner object
			global_car_model_ = std::make_shared<CarModelGlobal>(slice_);

			// Global params
			global_update_rate_ = declare_parameter<int>("global_update_rate") * 1ms;
			global_params_.max_iterations = declare_parameter<int>("global_max_iterations");
			global_params_.max_generations = declare_parameter<int>("global_max_generations");
			global_params_.sample_time = declare_parameter<float>("global_sample_time");
			global_params_.grid_states = declare_parameter<std::vector<bool>>("global_grid_states");
			std::vector<double> global_resolution = declare_parameter<std::vector<double>>("global_grid_resolution");
			global_params_.grid_resolution = std::vector<float>(global_resolution.begin(), global_resolution.end());
			std::vector<double> global_controls = declare_parameter<std::vector<double>>("global_controls");
			global_params_.samples = array_to_controls(global_controls, global_car_model_->NUM_CONTROLS);


			// Create local planner object
			local_car_model_ = std::make_shared<CarModelLocal>(slice_);

			// Local params
			local_update_rate_ = declare_parameter<int>("local_update_rate") * 1ms;
			local_params_.max_iterations = declare_parameter<int>("local_max_iterations");
			local_params_.max_generations = declare_parameter<int>("local_max_generations");
			local_params_.sample_time = declare_parameter<float>("local_sample_time");
			local_params_.grid_states = declare_parameter<std::vector<bool>>("local_grid_states");
			std::vector<double> local_resolution = declare_parameter<std::vector<double>>("local_grid_resolution");
			local_params_.grid_resolution = std::vector<float>(local_resolution.begin(), local_resolution.end());
			std::vector<double> local_controls = declare_parameter<std::vector<double>>("local_controls");
			local_params_.samples = array_to_controls(local_controls, local_car_model_->NUM_CONTROLS);

			// Global plan point to give as local goal
			global_plan_div_point_ = declare_parameter<int>("global_plan_div_point");


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
			slice_ = slice;
		}

		void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
			odom_ = odom;
		}

		void global_planner_callback() {
			/*
				RUN GLOBAL PLANNER
			*/
			// Set start & goal
			// Run
		}

		void local_planner_callback() {
			/*
				RUN LOCAL PLANNER
			*/
			// Set start & goal
			// Run
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
		std::shared_ptr<CarModelGlobal> global_car_model_;
		std::shared_ptr<CarModelLocal> local_car_model_;

		// Parameters
		std::chrono::milliseconds global_update_rate_, local_update_rate_;
		sbmpo::Parameters global_params_, local_params_;
		int global_plan_div_point_;

};

int main(int argc, char * argv[])
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<PlannerNode>());
	rclcpp::shutdown();
	return 0;
}
