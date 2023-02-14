#ifndef SD_GLOBAL_CAR_MODEL_SBMPO_HPP
#define SD_GLOBAL_CAR_MODEL_SBMPO_HPP

#include "sbmpo/model.hpp"
#include "sd504_nav_planner/NavigationUtil.hpp"

#define M_2PI 6.283185307179586f

namespace senior_design {

using namespace sbmpo;

    class CarModelGlobal : public Model {

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
        CarModelGlobal(rclcpp::Node& node, std::shared_ptr<NavigationUtil> nav_util) 
        : node_(node)
        {
            nav_util_ = nav_util;
            parameters_.grid_resolution = {0.1, 0.1};
            parameters_.grid_states = {true, true};
            parameters_.max_generations = 100;
            parameters_.max_iterations = 1000000;
            parameters_.sample_time = 1;
            parameters_.samples = {
                {1, 0},
                {-1, 0},
                {0, 1},
                {0, -1},
                {1, 1},
                {1, -1},
                {-1, 1},
                {-1, -1}
            };
        }

        // Get parameters
        Parameters parameters() { return parameters_; }

        // Update start and goal points
        bool update() {
            start_ = nav_util_->current_state_XY();
            goal_ = nav_util_->goal_state_XY();
            return nav_util_->isInitialized();
        }

        // Return initial state
        State initial_state() { 
            return start_; 
        }

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
            float cost_distance = sqrtf(control[dXdt]*control[dXdt] + control[dYdt]*control[dYdt]) * time_span;
            float cost_obstacles = cost_map(state2[X], state2[Y]);
            return cost_distance + cost_obstacles;
        }

        // Get the heuristic of a state
        float heuristic(const State& state) {
            float dx = (goal_[X] - state[X]) * INVERSE_X_GOAL_THRESHOLD;
            float dy = (goal_[Y] - state[Y]) * INVERSE_Y_GOAL_THRESHOLD;
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
                    nav_util_->map_lookup(state[X], state[Y]) - MIN_DISTANCE_TO_OBSTACLES <= 0;
        }

        float cost_map(const float x, const float y) {

            float distance = nav_util_->map_lookup(x, y);

            // Check if valid lookup
            if (distance == INVALID_DISTANCE)
                return 0;
                
            float cost = OBSTACLE_COST_COEFF_A * distance + OBSTACLE_COST_COEFF_B;
            return cost >= 0 ? cost : 0;
        }

        private:

        rclcpp::Node& node_;
        std::shared_ptr<NavigationUtil> nav_util_;

        State start_, goal_;
        Parameters parameters_;

    };

}

#endif