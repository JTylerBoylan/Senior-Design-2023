#include <math.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nvblox_msgs/msg/distance_map_slice.hpp"

using namespace std::chrono_literals;

class PlannerNode : public rclcpp::Node {
	public:

		PlannerNode() : Node("planner_node") {

			RCLCPP_INFO(this->get_logger(), "Planner node initialized.");

			// Create distance map subscriber
			slice_sub_ = this->create_subscription<nvblox_msgs::msg::DistanceMapSlice>(
				"/nvblox_node/map_slice", // Topic
				1, // Queue size
				std::bind(&PlannerNode::map_slice_callback, this, std::placeholders::_1)); // Callback function

			// Start planner loop
			timer_ = this->create_wall_timer(500ms,
				std::bind(&PlannerNode::timer_callback, this));

		}

	private:

		void map_slice_callback(const nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr slice) {
			slice_ = slice;
		}

		void timer_callback() {
			/*
				RUN PLANNER
			*/
		}

		// Add odometry subscriber
		// Add goal subscriber
		rclcpp::Subscription<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr slice_sub_;
		rclcpp::TimerBase::SharedPtr timer_;
		nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr slice_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<PlannerNode>());
	rclcpp::shutdown();
	return 0;
}
