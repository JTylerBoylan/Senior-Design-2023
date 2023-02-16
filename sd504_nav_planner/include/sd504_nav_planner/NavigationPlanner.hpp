#ifndef SD_NAV_PLANNER_HPP
#define SD_NAV_PLANNER_HPP

#include "sd504_nav_planner/NavigationUtil.hpp"

namespace senior_design {

using namespace sbmpo;

    // General Parameters
    const int GLOBAL_DIV_POINT = 4;
    
    // Global Parameters
    const int GLOBAL_MAX_ITERATIONS = 1E4;
    const int GLOBAL_MAX_GENERATIONS = 1E3;
    const float GLOBAL_SAMPLE_TIME = 1.0f;
    const float GLOBAL_GRID_RESOLUTION = 0.5f;
    const std::vector<State> GLOBAL_SAMPLES = {
        {1,0}, {1,1}, {0,1}, {-1,1},
        {-1,0}, {-1,-1}, {0,-1}, {1,-1}
    };

    // Local Parameters
    const int LOCAL_MAX_ITERATIONS = 1E3;
    const int LOCAL_MAX_GENERATIONS = 50;
    const float LOCAL_SAMPLE_TIME = 1.0f;
    const float LOCAL_GRID_RESOLUTION_XY = 0.1f;
    const float LOCAL_GRID_RESOLUTION_Q = 0.08727f;
    const float LOCAL_GRID_RESOLUTION_V = 0.25f;
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

        void run_global() {

            // Check if ready to run
            if (!is_global_ready())
                return;

            // Create global model
            CarModelGlobal global_model(NavigationUtil::current_XY(), NavigationUtil::goal_XY());

            RCLCPP_INFO(node_->get_logger(), "Running global planner...");

            /* GLOBAL PLANNER RUN */
            global_run_ = SBMPO::run(global_model, global_parameters());
            /* GLOBAL PLANNER END */

            // Print results
            RCLCPP_INFO(node_->get_logger(), "Global Plan (%.2f ms):", global_run_.time_us() / 1E3);
            for (State state : global_run_.state_path()) 
                RCLCPP_INFO(node_->get_logger(), "  X: %.2f, Y: %.2f", state[0], state[1]);
        }

        void run_local() {

            // Check if ready to run
            if (!is_local_ready())
                return;

            // Create local model
            CarModelLocal local_model(NavigationUtil::current_XYQVG(), this->local_goal());

            RCLCPP_INFO(node_->get_logger(), "Running local planner...");

            /* LOCAL PLANNER RUN */
            local_run_ = SBMPO::run(local_model, local_parameters());
            /* LOCAL PLANNER END */

            RCLCPP_INFO(node_->get_logger(), " Max iters: %d", local_parameters().max_iterations);

            // Print results
            RCLCPP_INFO(node_->get_logger(), "Local Plan (%.2f ms):", local_run_.time_us() / 1E3);
            RCLCPP_INFO(node_->get_logger(), "Exit code: %d, Buffer size: %d", int(local_run_.exit_code()), int(local_run_.size()));
            for (State state : local_run_.state_path()) 
                RCLCPP_INFO(node_->get_logger(), "  X: %.2f, Y: %.2f, Q: %.2f, V: %.2f, G: %.2f", state[0], state[1], state[2], state[3], state[4]);
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

    };

}

#endif