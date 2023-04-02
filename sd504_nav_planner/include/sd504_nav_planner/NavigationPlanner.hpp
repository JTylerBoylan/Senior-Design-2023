#ifndef SD504_NAV_PLANNER_HPP_
#define SD504_NAV_PLANNER_HPP_

#include <sd504_nav_planner/NavigationUtil.hpp>
#include <sd504_nav_planner/SD504Model.hpp>

#include <sbmpo_models/Grid2D.hpp>
#include <sbmpo_models/AckermannSteering.hpp>

namespace senior_design {

using namespace sbmpo;

// General Parameters
const int GLOBAL_DIV_POINT = 10;

class NavigationPlanner {

    public:

    NavigationPlanner(rclcpp::Node * node) {

        node_ = node;

        /*
            GLOBAL PARAMETERS
        */
        global_params_.max_iterations = 25000;
        global_params_.max_generations = 100;
        global_params_.sample_time = 1.0;
        global_params_.grid_resolution = {0.5, 0.5};
        global_params_.samples = {
            {1, 0}, {1, 1}, {0, 1}, {-1, 1},
            {-1, 0}, {-1, -1}, {0, -1}, {1, -1}
        };

        /*
            LOCAL PARAMETERS
        */
        local_params_.max_iterations = 10000;
        local_params_.max_generations = 40;
        local_params_.sample_time = 0.5;
        local_params_.grid_resolution = {0.04, 0.04, 0.015, 0.3, 0.12};
        local_params_.samples = {
            {2.5, 0.523}, {2.5, 0}, {2.5, -0.523},
            {0, 0.523}, {0, 0}, {0, -0.523},
            {-1.25, 0.523}, {-1.25, 0}, {-1.25, -0.523}
        };

        global_params_.start_state = State(0);
        global_params_.goal_state = State(0);
        local_params_.start_state = State(0);
        local_params_.goal_state = State(0);

    }

    ~NavigationPlanner() {
        delete node_;
    }

    // Run planner{} 
    int plan() {

        // Check if ready to run
        if (global_params_.goal_state == State(0)) {
            return 0;
        }

        // Run global sbmpo
        global_sbmpo_ = std::make_shared<SBMPO>(global_model_, global_params_);
        global_sbmpo_->run();
        this->print_global_run(*global_sbmpo_);

        // Check valid global path
        if (global_sbmpo_->exit_code() != 0) {
            return 0;
        }

        // Update local goal
        State local_goal = global_sbmpo_->state_path().size() <= GLOBAL_DIV_POINT ?
                        global_sbmpo_->state_path().back() :
                        global_sbmpo_->state_path()[GLOBAL_DIV_POINT];
        local_params_.goal_state = {local_goal[0], local_goal[1], 0, 0, 0};

        // Run local sbmpo
        local_sbmpo_ = std::make_shared<SBMPO>(local_model_, local_params_);
        local_sbmpo_->run();
        this->print_local_run(*local_sbmpo_);

        // Check valid local path
        if (local_sbmpo_->exit_code() != 0) {
            return 1;
        }

        return 2;
    }

    void update_state(const nav_msgs::msg::Odometry::ConstSharedPtr odometry) {
        global_params_.start_state = {
                float(odometry->pose.pose.position.x),
                float(odometry->pose.pose.position.y)
            };
        local_params_.start_state = {
                float(odometry->pose.pose.position.x),
                float(odometry->pose.pose.position.y),
                NavigationUtil::quaternion_to_pitch(odometry->pose.pose.orientation),
                float(odometry->twist.twist.linear.x),
                0.0f
                // NavigationUtil::rotation_to_ackermann(odometry->twist.twist.angular.z, odometry->twist.twist.linear.x, WHEEL_BASE_LENGTH)
            };
    }

    void update_goal(const geometry_msgs::msg::PointStamped::ConstSharedPtr goal_point) {
        global_params_.goal_state = {
                float(goal_point->point.x),
                float(goal_point->point.y)
            };
    }

    void update_map(const nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr slice) {
        global_model_.set_map(slice);
        local_model_.set_map(slice);
    }

    nav_msgs::msg::Path global_path() {
        return NavigationUtil::convert_XYQVG_path_to_path(NavigationUtil::XY_path_to_XYQVG_path(global_sbmpo_->state_path(), global_sbmpo_->control_path()));
    }

    nav_msgs::msg::Path local_path() {
        return NavigationUtil::convert_XYQVG_path_to_path(local_sbmpo_->state_path());
    }

    geometry_msgs::msg::PointStamped local_goal_point() {
        return NavigationUtil::convert_state_to_point(local_params_.goal_state);
    }

    std_msgs::msg::Int8 next_drive_acceleration() {
        std_msgs::msg::Int8 msg;
        const float drive_acc = local_sbmpo_->control_path().size() > 0 ? local_sbmpo_->control_path()[0][0] : 0;
        msg.data = int((127.0f/3.75f)*(drive_acc + 1.25f));
        return msg;
    }

    std_msgs::msg::Int8 next_turn_angle() {
        std_msgs::msg::Int8 msg;
        const float turn_angle = local_sbmpo_->control_path().size() > 0 ? local_sbmpo_->control_path()[0][1] : 0;
        msg.data = int((127.0f/0.523f)*turn_angle);
        return msg;
    }

    private:

    // Node
    rclcpp::Node * node_;

    // Models
    SD504Model<sbmpo_models::Grid2DModel> global_model_;
    SD504Model<sbmpo_models::AckermannSteeringModel> local_model_;

    // Params
    SBMPOParameters global_params_;
    SBMPOParameters local_params_;

    // SBMPO
    std::shared_ptr<SBMPO> global_sbmpo_;
    std::shared_ptr<SBMPO> local_sbmpo_;

    /*
        PRINTING FUNCTIONS
    */

    void print_global_run(SBMPO &run) {
        RCLCPP_INFO(node_->get_logger(), "----- GLOBAL RUN -----");
        RCLCPP_INFO(node_->get_logger(), "-- Parameters --");
        this->print_parameters(global_params_);
        RCLCPP_INFO(node_->get_logger(), "-- Results --");
        this->print_results(run);
        RCLCPP_INFO(node_->get_logger(), "----- ----- -----\n");
    }

    void print_local_run(SBMPO &run) {
        RCLCPP_INFO(node_->get_logger(), "----- LOCAL RUN -----");
        RCLCPP_INFO(node_->get_logger(), "-- Parameters --");
        this->print_parameters(local_params_);
        RCLCPP_INFO(node_->get_logger(), "-- Results --");
        this->print_results(run);
        RCLCPP_INFO(node_->get_logger(), "----- ----- -----\n");
    }
    
    void print_parameters(const SBMPOParameters &params) {
        RCLCPP_INFO(node_->get_logger(), " Max Iterations: %d", params.max_iterations);
        RCLCPP_INFO(node_->get_logger(), " Max Generations: %d", params.max_generations);
        RCLCPP_INFO(node_->get_logger(), " Sample Time: %.2f", params.sample_time);
        RCLCPP_INFO(node_->get_logger(), " Number of States: %lu", params.grid_resolution.size());
        RCLCPP_INFO(node_->get_logger(), " Number of Controls: %lu", params.samples.empty() ? 0 : params.samples[0].size());
        RCLCPP_INFO(node_->get_logger(), " Number of Samples: %lu", params.samples.size());
    }

    void print_results(SBMPO &run) {
        RCLCPP_INFO(node_->get_logger(), " Exit code: %d", run.exit_code());
        RCLCPP_INFO(node_->get_logger(), " Time (us): %lu", run.time_us());
        RCLCPP_INFO(node_->get_logger(), " Buffer Size: %lu", run.size());
        RCLCPP_INFO(node_->get_logger(), " Path Size: %lu", run.state_path().size());
        RCLCPP_INFO(node_->get_logger(), " - Path -");
        for (size_t p = 0; p < run.state_path().size(); p++) {
            const State state = run.state_path()[p];
            const Control control = p == run.state_path().size() ? Control(0) : run.control_path()[p];
            std::string state_str, control_str;
            for (size_t s = 0; s < state.size(); s++)
                state_str += std::to_string(state[s]) + " ";
            for (size_t c = 0; c < control.size(); c++)
                control_str += std::to_string(control[c]) + " ";
            RCLCPP_INFO(node_->get_logger(), " (%lu) [ %s] [ %s]", p++, state_str.c_str(), control_str.c_str());
        }
    }

};

}

#endif