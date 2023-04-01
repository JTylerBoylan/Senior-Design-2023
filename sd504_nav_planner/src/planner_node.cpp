#include <sd504_nav_planner/PlannerNode.hpp>

int main(int argc, char * argv[])
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<senior_design::PlannerNode>());
	rclcpp::shutdown();
	return 0;
}
