#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using namespace std::chrono_literals;

class GoalPublisher : public rclcpp::Node {

	public:

		GoalPublisher() : Node("goal_publisher") {

            // Set goal point
            goal_.header.frame_id = "map";
            goal_.point.x = 50;
            goal_.point.y = 0.0;
            goal_.point.z = 0.0;

			// Create path publisher
			goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/nav/goal/global", 10);

			// Start global planner loop
			timer_ = this->create_wall_timer(500ms,
				std::bind(&GoalPublisher::timer_callback, this));

			RCLCPP_INFO(this->get_logger(), "Goal publisher node initialized.");
		}

	private:

		void timer_callback() {
            goal_.header.stamp = rclcpp::Clock().now();
            goal_pub_->publish(goal_);
		}

		// ROS Publishers
		rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;

		// ROS Timers
		rclcpp::TimerBase::SharedPtr timer_;

        // Goal point
        geometry_msgs::msg::PointStamped goal_;

};

int main(int argc, char * argv[])
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<GoalPublisher>());
	rclcpp::shutdown();
	return 0;
}
