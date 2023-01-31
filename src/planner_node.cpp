#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "nvblox_msgs/msg/distance_map_slice.hpp"

class PlannerNode : public rclcpp::Node
{
	public:

		PlannerNode() : Node("planner_node") {

			RCLCPP_INFO(this->get_logger(), "Planner node initialized.");

			// Create distance map subscriber
			slice_sub_ = this->create_subscription<nvblox_msgs::msg::DistanceMapSlice>(
				"/nvblox_node/map_slice", // Topic
				1, // Queue size
				std::bind(&PlannerNode::map_slice_callback, this, std::placeholders::_1)); // Callback function

		}

	private:

		void map_slice_callback(const nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr slice) {
			slice_ = slice;
		}

		// Map lookup function (Move this to model)
		float map_lookup(const float x, const float y) {
			// Will return -1 if invalid

			// See if slice exists
			if (slice_ == nullptr)
				return -1;

			// Get the map indices
			float x_index = round((x - slice_->origin.x) / slice_->resolution);
			float y_index = round((y - slice_->origin.y) / slice_->resolution);

			// Check map bounds
			if (x_index < 0 || x_index >= static_cast<int>(slice_->width) ||
				y_index < 0 || y_index >= static_cast<int>(slice_->height))
				return -1;

			// Convert to index
			size_t index = x_index * slice_->width + y_index;

			// Grab value from slice
			float distance = slice_->data[index];

			// Check if unknown
			if (distance == slice_->unknown_value)
				return -1;
			
			return distance;
		}

		rclcpp::Subscription<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr slice_sub_;

		nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr slice_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<PlannerNode>());
	rclcpp::shutdown();
	return 0;
}
