#ifndef SD_GLOBAL_CAR_MODEL_SBMPO_HPP
#define SD_GLOBAL_CAR_MODEL_SBMPO_HPP

#include "sbmpo/model.hpp"
#include "senior_design_504/map_slice_util.hpp"
#include "nvblox_msgs/msg/distance_map_slice.hpp"
#include "nav_msgs/msg/odometry.hpp"

#define M_2PI 6.283185307179586f

namespace senior_design {

using namespace sbmpo;

    class CarModelGlobal : Model {

        public:
        
        // States used for this model
        const int NUM_STATES = 2;
        enum STATES {X, Y};

        // Controls used for this model
        const int NUM_CONTROLS = 2;
        enum CONTROLS {dXdt, dYdt};

        // Parameters
        const int INTEGRATION_SIZE = 5;

        // Constraints
        const float MIN_DISTANCE_TO_OBSTACLES = 1.0f; // m
        const float X_MAX = 100; // m
        const float X_MIN = -100; // m
        const float Y_MAX = 100; // m
        const float Y_MIN = -100; // m

        // Costs
        const float OBSTACLE_COST_COEFF_A = -10; // m^-1
        const float OBSTACLE_COST_COEFF_B = 30; // m^-1

        // Goal Thresholds
        const float INVERSE_X_GOAL_THRESHOLD = 1.0; // m^-1
        const float INVERSE_Y_GOAL_THRESHOLD = 1.0; // m^-1
        const float GOAL_THRESHOLD_FACTOR = 1.0; // m
        
        // Constructor
        CarModelGlobal(nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr map_slice) {
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
                state[X] += control[dXdt] * time_increment;
                state[Y] += control[dYdt] * time_increment;
                if (!is_valid(state))
                    return;
            }

        }

        // Get the cost of a control
        float cost(const State& state2, const State& state1, const Control& control, const float time_span) {
            float cost_distance = sqrtf(control[dXdt]*control[dXdt] + control[dYdt]*control[dYdt]);
            float cost_obstacles = cost_map(state2[X], state2[Y]);
            return cost_distance + cost_obstacles;
        }

        // Get the heuristic of a state
        float heuristic(const State& state) {
            float dx = ((*goal_)[X] - state[X]) * INVERSE_X_GOAL_THRESHOLD;
            float dy = ((*goal_)[Y] - state[Y]) * INVERSE_Y_GOAL_THRESHOLD;
            return sqrt(dx*dx + dy*dy);
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