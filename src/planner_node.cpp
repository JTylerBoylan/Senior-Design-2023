#include <math.h>
#include <chrono>

#include "senior_design_504/CarModelLocal.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nvblox_msgs/msg/distance_map_slice.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class PlannerNode : public rclcpp::Node {
	public:

		PlannerNode() : Node("planner_node") {

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
			local_car_model_ = std::make_shared<senior_design::CarModelLocal>(slice_);

			// Start planner loop
			timer_ = this->create_wall_timer(500ms,
				std::bind(&PlannerNode::timer_callback, this));

		}

	private:

		void map_slice_callback(const nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr slice) {
			slice_ = slice;
		}

		void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
			odom_ = odom;
		}

		void timer_callback() {
			/*
				RUN PLANNER
			*/
		}

		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
		rclcpp::Subscription<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr slice_sub_;
		rclcpp::TimerBase::SharedPtr timer_;

		geometry_msgs::msg::Point::ConstSharedPtr goal_;
		nav_msgs::msg::Odometry::ConstSharedPtr odom_;
		nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr slice_;

		std::shared_ptr<senior_design::CarModelLocal> local_car_model_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<PlannerNode>());
	rclcpp::shutdown();
	return 0;
}
