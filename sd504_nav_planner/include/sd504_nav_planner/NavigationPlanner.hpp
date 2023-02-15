#ifndef SD_NAV_PLANNER_HPP
#define SD_NAV_PLANNER_HPP

#include <math.h>
#include <chrono>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "nvblox_msgs/msg/distance_map_slice.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sbmpo/sbmpo.hpp"

namespace senior_design {

using namespace sbmpo;
using namespace std::chrono;

    // General Parameters
    const float INVALID_DISTANCE = -1.0f;
    float WHEEL_BASE_LENGTH = 1.0f;
    int GLOBAL_DIV_POINT = 4;
    
    // Global Parameters
    const int GLOBAL_MAX_ITERATIONS = 1E4;
    const int GLOBAL_MAX_GENERATIONS = 1E3;
    const float GLOBAL_SAMPLE_TIME = 1.0f;
    const float GLOBAL_GRID_RESOLUTION = 0.5f;
    const std::vector<std::vector<float>> GLOBAL_SAMPLES = {
        {1,0}, {1,1}, {0,1}, {-1,1},
        {-1,0}, {-1,-1}, {0,-1}, {1,-1}
    };

    // Local Parameters
    const int LOCAL_MAX_ITERATIONS = 1E4;
    const int LOCAL_MAX_GENERATIONS = 1E3;
    const float LOCAL_SAMPLE_TIME = 1.0f;
    /*
        TODO:
        - Resolution Q, V, G
        - Samples
    */


    class NavigationPlanner {

        public:

        NavigationPlanner(rclcpp::Node &node) {
            node_ = node;
            global_model_ = CarModelGlobal(*this);
            local_model_ = CarModelLocal(*this);
        }

        void run_global() {

            // Check if ready to run
            if (!is_global_ready())
                return;

            // Update start and goal points
            update_global_state();
            update_global_goal();

            RCLCPP_INFO(this->get_logger(), "Running global planner...");

            // Start timer
            high_resolution_clock::time_point t1 = high_resolution_clock::now();

            /* GLOBAL PLANNER RUN */
            global_run_ = SBMPO::run(global_model_, global_parameters());
            /* GLOBAL PLANNER END */

            // End timer
            high_resolution_clock::time_point t2 = high_resolution_clock::now();
            duration<double> time_span = duration_cast<duration<double>>(t2-t1);

            // Print results
            RCLCPP_INFO(this->get_logger(), "Global Plan (%.2f ms):", float(time_span.count() * 1E3));
            for (State state : global_run_.state_path()) 
                RCLCPP_INFO(this->get_logger(), "  X: %.2f, Y: %.2f", state[0], state[1]);
        }

        void run_local() {

            // Check if ready to run
            if (!is_local_ready())
                return;

            // Update start and goal points
            update_local_state();
            update_local_goal();

            RCLCPP_INFO(this->get_logger(), "Running local planner...");

            // Start timer
            high_resolution_clock::time_point t1 = high_resolution_clock::now();

            /* LOCAL PLANNER RUN */
            local_run_ = SBMPO::run(local_model_, local_parameters());
            /* LOCAL PLANNER END */

            // End timer
            high_resolution_clock::time_point t2 = high_resolution_clock::now();
            duration<double> time_span = duration_cast<duration<double>>(t2-t1);

            // Print results
            RCLCPP_INFO(this->get_logger(), "Local Plan (%.2f ms):", float(time_span.count() * 1E3));
            for (State state : local_run_.state_path()) 
                RCLCPP_INFO(this->get_logger(), "  X: %.2f, Y: %.2f, Q: %.2f, V: %.2f, G: %.2f", state[0], state[1], state[2], state[3], state[4]);
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
            // TODO: Local Params
            return params;
        }

        void update_map_slice(nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr nvblox_map_slice) {
            map_slice_ = nvblox_map_slice;
        }

        void update_odometry(nav_msgs::msg::Odometry::ConstSharedPtr odometry) {
            odom_ = odometry;
        }

        void update_goal_point(geometry_msgs::msg::Point::ConstSharedPtr goal_point) {
            goal_ = goal_point;
        }

        // Map lookup function
        float map_lookup(const float x, const float y) {

            // See if slice exists
            if (map_slice_ == nullptr)
                return INVALID_DISTANCE;

            // Get the map indices
            float x_index = round((x - map_slice_->origin.x) / map_slice_->resolution);
            float y_index = round((y - map_slice_->origin.y) / map_slice_->resolution);

            // Check map bounds
            if (x_index < 0 || x_index >= static_cast<int>(map_slice_->width) ||
                y_index < 0 || y_index >= static_cast<int>(map_slice_->height))
                return INVALID_DISTANCE;

            // Convert to index
            size_t index = x_index * map_slice_->width + y_index;

            // Grab value from slice
            float distance = map_slice_->data[index];

            // Check if unknown
            if (distance == map_slice_->unknown_value)
                return INVALID_DISTANCE;
            
            return distance;
        }

        private:

        // Node
        rclcpp::Node &node_;

        // SBMPO Runs
        SBMPORun global_run_;
        SBMPORun local_run_;

        // Car Models
        CarModelGlobal global_model_;
        CarModelLocal local_model_;

        // ROS msgs
        nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr map_slice_;
        nav_msgs::msg::Odometry::ConstSharedPtr odom_;
        geometry_msgs::msg::Point::ConstSharedPtr goal_;

        bool is_global_ready() {
            return map_slice_ != nullptr
                && odom_ != nullptr
                && goal_ != nullptr;
        }

        bool is_local_ready() {
            return global_run_.state_path().size() > 1;
        }

        void update_global_state() {
            if (odom_ == nullptr)
                return;
            State state(2);
            state[0] = odom_->pose.pose.position.x;
            state[1] = odom_->pose.pose.position.y;
            global_model_.set_initial_state(state);
        }

        void update_local_state() {
            if (odom_ == nullptr)
                return;
            State state(5);
            state[0] = odom_->pose.pose.position.x;
            state[1] = odom_->pose.pose.position.y;
            state[2] = quaternion_to_pitch(odom_->pose.pose.orientation);
            state[3] = odom_->twist.twist.linear.x;
            state[4] = rotation_to_ackermann(odom_->twist.twist.angular.z, state[3], WHEEL_BASE_LENGTH);
            /* ^ Think about replacing this with encoder data */
            local_model_.set_initial_state(state);
        }

        void update_global_goal() {
            if (goal_ == nullptr)
                return;
            State state(2);
            state[0] = goal_->x;
            state[1] = goal_->y;
            global_model_.set_goal_state(state);
        }

        void update_local_goal() {
            int plan_size = global_state_plan_.size();
            if (plan_size < 2)
                return;
            bool shrt = plan_size + 1 <= GLOBAL_DIV_POINT;
            int ref = shrt ? plan_size - 1: GLOBAL_DIV_POINT;
            State state(5);
            State ref_state = local_state_plan_[ref];
            State ref_state_back = local_state_plan_[ref-1];
            State ref_state_for = shrt ? State(0) : local_state_plan_[ref+1];
            Control ref_control = global_state_plan_[ref];
            state[0] = ref_state[0];
            state[1] = ref_state[1];
            float theta_i1 = atan2(ref_state[1] - ref_state_back[1], ref_state[0] - ref_state_back[0]);
            float theta_i2 = shrt ? theta_i1 : atan2(ref_state_for[1] - ref_state[1], ref_state_for[0] - ref_state[0]);
            state[2] = 0.5f * (theta_i1 + theta_i2);
            state[3] = sqrtf(ref_control[0]*ref_control[0] + ref_control[1]*ref_control[1]);
            state[4] = rotation_to_ackermann((theta_i2-theta_i1)/SAMPLE_TIME,state[3],WHEEL_BASE_LENGTH);
            local_model_.set_goal_state(state);
        }

        float quaternion_to_pitch(geometry_msgs::msg::Quaternion quat){
            double w = quat.w;
            double x = quat.x;
            double y = quat.y;
            double z = quat.z;
            double siny_cosp = 2 * (w * z + x * y);
            double cosy_cosp = 1 - 2 * (y * y + z * z);
            return atan2(siny_cosp, cosy_cosp);
        }

        float rotation_to_ackermann(float omega, float v, float L) {
            return atan2(omega*L,v);
        }

    };

}

#endif