#ifndef SD_LOCAL_CAR_MODEL_SBMPO_HPP
#define SD_LOCAL_CAR_MODEL_SBMPO_HPP

#include "sbmpo/model.hpp"
#include "nvblox_msgs/msg/distance_map_slice.hpp"
#include "nav_msgs/msg/odometry.hpp"

#define M_2PI 6.283185307179586f
#define INVALID_DISTANCE -1.0f

namespace senior_design {

using namespace sbmpo;

    class CarModelLocal : Model {

        // Goal state
        State start = {/* TODO */};
        State goal = {/* TODO */};

        // Parameters
        const int INTEGRATION_SIZE = 5;
        const float INVERSE_WHEEL_BASE_LENGTH = 2.0; // m

        // Constraints
        const float MIN_DISTANCE_TO_OBSTACLES = 1.0f; // m
        const float X_MAX = 100; // m
        const float X_MIN = -100; // m
        const float Y_MAX = 100; // m
        const float Y_MIN = -100; // m
        const float VELOCITY_MAX = 2.5; // m/s
        const float VELOCITY_MIN = 0; // m/s
        const float TURN_ANGLE_MAX = M_PI / 6.0; // rad
        const float TURN_ANGLE_MIN = M_PI / 6.0; // rad
        const float TURN_ACCELERATION_MAX = 5.0; // m/s^2

        // Costs
        const float LIN_ACCELERATION_COST_COEFF = 0; // s^2/m
        const float TURN_ACCELERATION_COST_COEFF = 0; // s/rad
        const float OBSTACLE_COST_COEFF_A = -10; // m^-1
        const float OBSTACLE_COST_COEFF_B = 30; // m^-1

        // Goal Thresholds
        const float INVERSE_X_GOAL_THRESHOLD = 1.0; // m^-1
        const float INVERSE_Y_GOAL_THRESHOLD = 1.0; // m^-1
        const float INVERSE_Q_GOAL_THRESHOLD = 6.0 / M_PI; // rad^-1
        const float INVERSE_V_GOAL_THRESHOLD = 1.0; // s/m
        const float INVERSE_G_GOAL_THRESHOLD = 6.0 / M_PI; // s/rad
        const float GOAL_THRESHOLD_FACTOR = 1.0; // m

        public:
        
        CarModelLocal(nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr map_slice) {
            map_slice_ = map_slice;
        }

        // Return initial state
        State initial_state() {
            return start;
        }

        // Evaluate a node with a control
        void next_state(State& state, const Control& control, const float time_span) {

            // Integrate control into state (Euler)
            float time_increment = time_span / INTEGRATION_SIZE;
            for (int i = 0; i < INTEGRATION_SIZE; i++) {
                state[0] += cosf(state[2]) * state[3] * time_increment;
                state[1] += sinf(state[2]) * state[3] * time_increment;
                state[2] += tanf(state[4]) * state[3] * time_increment * INVERSE_WHEEL_BASE_LENGTH;
                state[3] += control[0] * time_increment;
                state[4] += control[1] * time_increment;
                if (!is_valid(state))
                    return;
            }

            // Angle wrap
            while (state[2] >= M_2PI)  state[2] -= M_2PI;
            while (state[2] < 0)       state[2] += M_2PI;

        }

        // Get the cost of a control
        float cost(const State& state2, const State& state1, const Control& control, const float time_span) {
            float cost_time = time_span;
            float cost_accel = LIN_ACCELERATION_COST_COEFF * control[0] * time_span;
            float cost_turn = TURN_ACCELERATION_COST_COEFF * control[1] * time_span; 
            float cost_obstacles = cost_map(state2[0], state2[1]);
            return cost_time + cost_accel + cost_turn + cost_obstacles;
        }

        // Get the heuristic of a state
        float heuristic(const State& state) {
            float dx = (goal[0] - state[0]) * INVERSE_X_GOAL_THRESHOLD;
            float dy = (goal[1] - state[1]) * INVERSE_Y_GOAL_THRESHOLD;
            float dq = (goal[2] - state[2]) * INVERSE_Q_GOAL_THRESHOLD;
            float dv = (goal[3] - state[3]) * INVERSE_V_GOAL_THRESHOLD;
            float dg = (goal[4] - state[4]) * INVERSE_G_GOAL_THRESHOLD;
            return sqrt(dx*dx + dy*dy + dq*dq + dv*dv + dg*dg);
        }

        // Determine if state is goal
        bool is_goal(const State& state) {
            return heuristic(state) <= GOAL_THRESHOLD_FACTOR;
        }

        // Determine if state is valid
        bool is_valid(const State& state) {
            return  state[0] - X_MAX <= 0 && 
                    X_MIN - state[0] <= 0 &&
                    state[1] - Y_MAX <= 0 && 
                    Y_MIN - state[1] <= 0 &&
                    state[3] - VELOCITY_MAX <= 0 && 
                    VELOCITY_MIN - state[3] <= 0 &&
                    state[4] - TURN_ANGLE_MAX <= 0 && 
                    TURN_ANGLE_MIN - state[4] <= 0 &&
                    state[3]*state[3]*INVERSE_WHEEL_BASE_LENGTH*tan(state[2]) - TURN_ACCELERATION_MAX <= 0 &&
                    map_lookup(map_slice_, state[0], state[1]) - MIN_DISTANCE_TO_OBSTACLES <= 0;
        }

        float cost_map(const float x, const float y) {

            float distance = map_lookup(map_slice_, x, y);

            // Check if valid lookup
            if (distance == INVALID_DISTANCE)
                return 0;
                
            float cost = OBSTACLE_COST_COEFF_A * distance + OBSTACLE_COST_COEFF_B;
            return cost >= 0 ? cost : 0;
        }

        private:

        nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr map_slice_;

    };

    // Map lookup function (Move this to model)
    float map_lookup(const nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr slice_, const const float x, const float y) {

        // See if slice exists
        if (slice_ == nullptr)
            return INVALID_DISTANCE;

        // Get the map indices
        float x_index = round((x - slice_->origin.x) / slice_->resolution);
        float y_index = round((y - slice_->origin.y) / slice_->resolution);

        // Check map bounds
        if (x_index < 0 || x_index >= static_cast<int>(slice_->width) ||
            y_index < 0 || y_index >= static_cast<int>(slice_->height))
            return INVALID_DISTANCE;

        // Convert to index
        size_t index = x_index * slice_->width + y_index;

        // Grab value from slice
        float distance = slice_->data[index];

        // Check if unknown
        if (distance == slice_->unknown_value)
            return INVALID_DISTANCE;
        
        return distance;
    }

}

#endif