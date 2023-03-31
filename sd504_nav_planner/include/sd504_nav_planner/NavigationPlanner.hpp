#ifndef SD_NAV_PLANNER_HPP
#define SD_NAV_PLANNER_HPP

#include "sd504_nav_planner/NavigationUtil.hpp"
#include "sd504_nav_planner/CarModelLocal.hpp"
#include "sd504_nav_planner/CarModelGlobal.hpp"

#include "std_msgs/msg/float32.hpp"

namespace senior_design {

using namespace sbmpo;

    // General Parameters
    const int GLOBAL_DIV_POINT = 4;
    
    // Global Parameters
    const int GLOBAL_MAX_ITERATIONS = 1E4;
    const int GLOBAL_MAX_GENERATIONS = 1E3;
    const float GLOBAL_SAMPLE_TIME = 1.0f;
    const float GLOBAL_GRID_RESOLUTION = 0.25f;
    const std::vector<State> GLOBAL_SAMPLES = {
        {1,0}, {1,1}, {0,1}, {-1,1},
        {-1,0}, {-1,-1}, {0,-1}, {1,-1}
    };

    // Local Parameters
    const int LOCAL_MAX_ITERATIONS = 5E3;
    const int LOCAL_MAX_GENERATIONS = 50;
    const float LOCAL_SAMPLE_TIME = 0.5f;
    const float LOCAL_GRID_RESOLUTION_XY = 0.25f;
    const float LOCAL_GRID_RESOLUTION_Q = 0.08727f;
    const float LOCAL_GRID_RESOLUTION_V = 0.50f;
    const float LOCAL_GRID_RESOLUTION_G = 0.13090f;
    const std::vector<State> LOCAL_SAMPLES = {
        {2.45, 0.436}, {2.45, 0.218}, {2.45, 0}, {2.45, -0.218}, {2.45, -0.436},
        {1.23, 0.436}, {1.23, 0.218}, {1.23, 0}, {1.23, -0.218}, {1.23, -0.436},
        {0, 0.436}, {0, 0.218}, {0, 0}, {0, -0.218}, {0, -0.436},
        {-1.23, 0.436}, {-1.23, 0.218}, {-1.23, 0}, {-1.23, -0.218}, {-1.23, -0.436}
    };


    class NavigationPlanner {

        public:

        NavigationPlanner(rclcpp::Node * node) {
            node_ = node;
        }

        ~NavigationPlanner() {
            delete node_;
        }

        bool run_global() {

            // Check if ready to run
            if (!is_global_ready())
                return false;

            // Create global model
            CarModelGlobal global_model(NavigationUtil::current_XY(), NavigationUtil::goal_XY());

            /* GLOBAL PLANNER RUN */
            global_run_ = SBMPO::run(global_model, global_parameters());
            /* GLOBAL PLANNER END */

            // Print results
            this->print_global_run(global_run_);

            return true;
        }

        bool run_local() {

            // Check if ready to run
            if (!is_local_ready())
                return false;

            // Create local model
            CarModelLocal local_model(NavigationUtil::current_XYQVG(), this->local_goal());

            /* LOCAL PLANNER RUN */
            local_run_ = SBMPO::run(local_model, local_parameters());
            /* LOCAL PLANNER END */

            // Print results
            this->print_local_run(local_run_);

            return true;
        }

        Parameters global_parameters() {
            Parameters params;
            params.max_iterations = GLOBAL_MAX_ITERATIONS;
            params.max_generations = GLOBAL_MAX_GENERATIONS;
            params.sample_time = GLOBAL_SAMPLE_TIME;
            params.grid_states = {true, true};
            params.grid_resolution = {GLOBAL_GRID_RESOLUTION, GLOBAL_GRID_RESOLUTION};
            params.samples = GLOBAL_SAMPLES;
            return params;
        }

        Parameters local_parameters() {
            Parameters params;
            params.max_iterations = LOCAL_MAX_ITERATIONS;
            params.max_generations = LOCAL_MAX_GENERATIONS;
            params.sample_time = LOCAL_SAMPLE_TIME;
            params.grid_states = {true, true, true, true, true};
            params.grid_resolution = {LOCAL_GRID_RESOLUTION_XY,
                                        LOCAL_GRID_RESOLUTION_XY,
                                        LOCAL_GRID_RESOLUTION_Q,
                                        LOCAL_GRID_RESOLUTION_V,
                                        LOCAL_GRID_RESOLUTION_G};
            params.samples = LOCAL_SAMPLES;
            return params;
        }

        nav_msgs::msg::Path global_path() {
            return NavigationUtil::convert_XYQVG_path_to_path(NavigationUtil::XY_path_to_XYQVG_path(global_run_.state_path(), global_run_.control_path()));
        }

        nav_msgs::msg::Path local_path() {
            return NavigationUtil::convert_XYQVG_path_to_path(local_run_.state_path());
        }

        geometry_msgs::msg::PointStamped local_goal_point() {
            return NavigationUtil::convert_state_to_point(local_goal());
        }

        std_msgs::msg::Float32 next_drive_acceleration() {
            std_msgs::msg::Float32 msg;
            msg.data = local_run_.control_path().size() > 0 ? local_run_.control_path()[0][0]: 0;
            return msg;
        }

        std_msgs::msg::Float32 next_turn_angle() {
            std_msgs::msg::Float32 msg;
            msg.data = local_run_.control_path().size() > 0 ? local_run_.control_path()[0][1] : 0;
            return msg;
        }

        private:

        // Node
        rclcpp::Node * node_;

        // SBMPO Runs
        SBMPORun global_run_;
        SBMPORun local_run_;

        bool is_global_ready() {
            return NavigationUtil::distance_map_slice != nullptr
                && NavigationUtil::odometry != nullptr
                && NavigationUtil::goal_point != nullptr;
        }

        bool is_local_ready() {
            return !global_run_.state_path().empty();
        }

        State local_goal() {
            std::vector<State> xyqvg_path = NavigationUtil::XY_path_to_XYQVG_path(global_run_.state_path(), global_run_.control_path());
            int div_idx = xyqvg_path.size() <= GLOBAL_DIV_POINT ? xyqvg_path.size() - 1 : GLOBAL_DIV_POINT;
            return xyqvg_path[div_idx];
        }

        void print_global_run(SBMPORun &run) {
            RCLCPP_INFO(node_->get_logger(), "----- GLOBAL RUN -----");

            // Print parameters
            RCLCPP_INFO(node_->get_logger(), "-- Parameters --");
            this->print_parameters(global_parameters());
            
            // Print results
            RCLCPP_INFO(node_->get_logger(), "-- Results --");
            this->print_results(run);

            RCLCPP_INFO(node_->get_logger(), "----- ----- -----\n");
        }

        void print_local_run(SBMPORun &run) {
            RCLCPP_INFO(node_->get_logger(), "----- LOCAL RUN -----");

            // Print parameters
            RCLCPP_INFO(node_->get_logger(), "-- Parameters --");
            this->print_parameters(local_parameters());
            
            // Print results
            RCLCPP_INFO(node_->get_logger(), "-- Results --");
            this->print_results(run);

            RCLCPP_INFO(node_->get_logger(), "----- ----- -----\n");
        }
        
        void print_parameters(const Parameters &params) {
            RCLCPP_INFO(node_->get_logger(), " Max Iterations: %d", params.max_iterations);
            RCLCPP_INFO(node_->get_logger(), " Max Generations: %d", params.max_generations);
            RCLCPP_INFO(node_->get_logger(), " Sample Time: %.2f", params.sample_time);
            RCLCPP_INFO(node_->get_logger(), " Number of States: %lu", params.grid_resolution.size());
            RCLCPP_INFO(node_->get_logger(), " Number of Controls: %lu", params.samples.empty() ? 0 : params.samples[0].size());
            RCLCPP_INFO(node_->get_logger(), " Number of Samples: %lu", params.samples.size());
        }

        void print_results(SBMPORun &run) {
            RCLCPP_INFO(node_->get_logger(), " Exit code: %d", run.exit_code());
            RCLCPP_INFO(node_->get_logger(), " Time (us): %lu", run.time_us());
            RCLCPP_INFO(node_->get_logger(), " Buffer Size: %lu", run.size());
            RCLCPP_INFO(node_->get_logger(), " Path Size: %lu", run.state_path().size());
            RCLCPP_INFO(node_->get_logger(), " - Path -");
            int p = 1;
            for (State state : run.state_path()) {
                float dist_xy = NavigationUtil::map_lookup(state[0], state[1]);
                std::string coord = std::to_string(state[0]);
                for (size_t s = 1; s < state.size(); s++)
                    coord += ", " + std::to_string(state[s]);
                RCLCPP_INFO(node_->get_logger(), " (%d) (%s) - f(x,y)= %.2f", p++, coord.c_str(), dist_xy == INVALID_DISTANCE ? -1.0 : dist_xy);
            }
        }

    };

}

#endif