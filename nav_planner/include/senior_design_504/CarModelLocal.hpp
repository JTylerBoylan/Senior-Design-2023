#ifndef SD_LOCAL_CAR_MODEL_SBMPO_HPP
#define SD_LOCAL_CAR_MODEL_SBMPO_HPP

#include "sbmpo/model.hpp"
#include "senior_design_504/map_slice_util.hpp"
#include "nvblox_msgs/msg/distance_map_slice.hpp"
#include "nav_msgs/msg/odometry.hpp"

#define M_2PI 6.283185307179586f

namespace senior_design {

using namespace sbmpo;

    class CarModelLocal : Model {

        public:

        // States used for this model
        const int NUM_STATES = 5;
        enum STATES {X, Y, Q, V, G};

        // Controls used for this model
        const int NUM_CONTROLS = 2;
        enum CONTROLS {dVdt, dGdt};

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
        
        // Constructor
        CarModelLocal(nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr map_slice) {
            map_slice_ = map_slice;
            start_ = std::make_shared<State>(NUM_STATES);
            goal_ = std::make_shared<State>(NUM_CONTROLS);
        }

        // Get start state pointer
        std::shared_ptr<State> start_state() { return start_; }

        // Get goal state pointer
        std::shared_ptr<State> goal_state() { return goal_; }

        // Return initial state
        State initial_state() { return *start_; }

        // Evaluate a node with a control
        void next_state(State& state, const Control& control, const float time_span) {

            // Integrate control into state (Euler)
            float time_increment = time_span / INTEGRATION_SIZE;
            for (int i = 0; i < INTEGRATION_SIZE; i++) {
                state[X] += cosf(state[Q]) * state[V] * time_increment;
                state[Y] += sinf(state[Q]) * state[V] * time_increment;
                state[Q] += tanf(state[G]) * state[V] * time_increment * INVERSE_WHEEL_BASE_LENGTH;
                state[V] += control[dVdt] * time_increment;
                state[G] += control[dGdt] * time_increment;
                if (!is_valid(state))
                    return;
            }

            // Angle wrap
            while (state[Q] >= M_2PI)  state[Q] -= M_2PI;
            while (state[Q] < 0)       state[Q] += M_2PI;

        }

        // Get the cost of a control
        float cost(const State& state2, const State& state1, const Control& control, const float time_span) {
            float cost_time = time_span;
            float cost_accel = LIN_ACCELERATION_COST_COEFF * abs(control[dVdt]) * time_span;
            float cost_turn = TURN_ACCELERATION_COST_COEFF * abs(control[dGdt]) * time_span; 
            float cost_obstacles = cost_map(state2[X], state2[Y]);
            return cost_time + cost_accel + cost_turn + cost_obstacles;
        }

        // Get the heuristic of a state
        float heuristic(const State& state) {
            float dx = ((*goal_)[X] - state[X]) * INVERSE_X_GOAL_THRESHOLD;
            float dy = ((*goal_)[Y] - state[Y]) * INVERSE_Y_GOAL_THRESHOLD;
            float dq = ((*goal_)[Q] - state[Q]) * INVERSE_Q_GOAL_THRESHOLD;
            float dv = ((*goal_)[V] - state[V]) * INVERSE_V_GOAL_THRESHOLD;
            float dg = ((*goal_)[G] - state[G]) * INVERSE_G_GOAL_THRESHOLD;
            return sqrt(dx*dx + dy*dy + dq*dq + dv*dv + dg*dg);
        }

        // Determine if state is goal
        bool is_goal(const State& state) {
            return heuristic(state) <= GOAL_THRESHOLD_FACTOR;
        }

        // Determine if state is valid
        bool is_valid(const State& state) {
            return  state[X] - X_MAX <= 0 && 
                    X_MIN - state[X] <= 0 &&
                    state[Y] - Y_MAX <= 0 && 
                    Y_MIN - state[Y] <= 0 &&
                    state[V] - VELOCITY_MAX <= 0 && 
                    VELOCITY_MIN - state[V] <= 0 &&
                    state[G] - TURN_ANGLE_MAX <= 0 && 
                    TURN_ANGLE_MIN - state[G] <= 0 &&
                    state[V]*state[V]*INVERSE_WHEEL_BASE_LENGTH*tan(state[G]) - TURN_ACCELERATION_MAX <= 0 &&
                    map_lookup(map_slice_, state[X], state[Y]) - MIN_DISTANCE_TO_OBSTACLES <= 0;
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
        std::shared_ptr<State> start_, goal_;

    };

}

#endif