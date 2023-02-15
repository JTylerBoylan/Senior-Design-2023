#ifndef SD_NAV_PLANNER_HPP
#define SD_NAV_PLANNER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sbmpo/sbmpo.hpp"

#include "sd504_nav_planner/NavigationUtil.hpp"

namespace senior_design {

using namespace sbmpo;

    // General Parameters
    const float INVALID_DISTANCE = -1.0f;
    const float WHEEL_BASE_LENGTH = 1.0f;
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
    const int LOCAL_MAX_ITERATIONS = 1E4;
    const int LOCAL_MAX_GENERATIONS = 1E3;
    const float LOCAL_SAMPLE_TIME = 1.0f;
    const float LOCAL_GRID_RESOLUTION_XY = 0.25f;
    const float LOCAL_GRID_RESOLUTION_Q = 0.08727f;
    const float LOCAL_GRID_RESOLUTION_V = 0.5f;
    const float LOCAL_GRID_RESOLUTION_G = 0.13090f;
    const std::vector<State> LOCAL_SAMPLES = {
        {2.45, 0.436}, {2.45, 0.218}, {2.45, 0}, {2.45, -0.218}, {2.45, -0.436},
        {1.23, 0.436}, {1.23, 0.218}, {1.23, 0}, {1.23, -0.218}, {1.23, -0.436},
        {0, 0.436}, {0, 0.218}, {0, 0}, {0, -0.218}, {0, -0.436},
        {-1.23, 0.436}, {-1.23, 0.218}, {-1.23, 0}, {-1.23, -0.218}, {-1.23, -0.436}
    };


    class NavigationPlanner {

        public:

        NavigationPlanner(rclcpp::Node &node) {
            node_ = node;
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

            // Print results
            RCLCPP_INFO(node_->get_logger(), "Local Plan (%.2f ms):", local_run_.time_us() / 1E3);
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

        private:

        // Node
        rclcpp::Node &node_;

        // SBMPO Runs
        SBMPORun global_run_;
        SBMPORun local_run_;

        bool is_global_ready() {
            return map_slice_ != nullptr
                && odom_ != nullptr
                && goal_ != nullptr;
        }

        bool is_local_ready() {
            return !local_goal().empty();
        }

        State local_goal() {
            int plan_size = global_run_.state_path().size();
            if (plan_size < 2) {
                return State(0);
            }
            
            bool is_short = GLOBAL_DIV_POINT + 1 >= plan_size;
            int ref_index = is_short ? plan_size - 1 : GLOBAL_DIV_POINT;
            
            State ref_state = global_run_.state_path()[ref_index];
            State ref_state_back = global_run_.state_path()[ref_index - 1];
            State ref_state_for = is_short ? State(0) : global_run_.state_path()[ref_index + 1];

            Control ref_control = global_run_.control_path()[ref_index-1];
            Control ref_control_for = is_short ? Control(0) : global_run_.control_path()[ref_index];
            
            float theta_i1 = atan2(ref_state[1] - ref_state_back[1], ref_state[0] - ref_state_back[0]);
            float theta_i2 = is_short ? theta_i1 : atan2(ref_state_for[1] - ref_state[1], ref_state_for[0] - ref_state[0]);
            float theta = 0.5f * (theta_i1 + theta_i2);

            float velocity_i1 = sqrtf(ref_control[0]*ref_control[0] + ref_control[1]*ref_control[1]);
            float velocity_i2 = is_short ? velocity_i1 : sqrtf(ref_control_for[0]*ref_control_for[0]+ref_control_for[1]*ref_control[1]);
            float velocity = 0.5f * (velocity_i1 + velocity_i2);

            float omega = (theta_i2 - theta_i1) / GLOBAL_SAMPLE_TIME;

            return {
                ref_state[0], // X
                ref_state[1], // Y
                theta, // Q
                velocity, // V
                rotation_to_ackermann(omega, velocity, WHEEL_BASE_LENGTH) // G
            };
        }

    };

}

#endif