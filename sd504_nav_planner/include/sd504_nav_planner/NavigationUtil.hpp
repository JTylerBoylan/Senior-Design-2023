#ifndef SD_NAV_UTIL_HPP
#define SD_NAV_UTIL_HPP

#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "nvblox_msgs/msg/distance_map_slice.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sbmpo/sbmpo.hpp"

namespace senior_design {

using namespace sbmpo;

    const float INVALID_DISTANCE = -1.0f;
    float WHEEL_BASE_LENGTH = 1.0f;
    float SAMPLE_TIME = 1.0f;
    int GLOBAL_DIV_POINT = 4;

    class NavigationUtil {

        public:

        NavigationUtil(nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr nvblox_map_slice,
                        nav_msgs::msg::Odometry::ConstSharedPtr odometry,
                        geometry_msgs::msg::Point::ConstSharedPtr goal_point) {
            map_slice_ = nvblox_map_slice;
            odom_ = odometry;
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

        // Convert odometry msg to global state
        State current_state_XY() {
            State state(2);
            if (odom_ != nullptr) {
                state[0] = odom_->pose.pose.position.x;
                state[1] = odom_->pose.pose.position.y;
            }
            return state;
        }

        State current_state_XYQVG() {
            State state(5);
            if (odom_ != nullptr) {
                state[0] = odom_->pose.pose.position.x;
                state[1] = odom_->pose.pose.position.y;
                state[2] = quaternion_to_pitch(odom_->pose.pose.orientation);
                state[3] = odom_->twist.twist.linear.x;
                state[4] = rotation_to_ackermann(odom_->twist.twist.angular.z, state[3], WHEEL_BASE_LENGTH);
                /* ^ Think about replacing this with encoder data */
            }
            return state;
        }

        State goal_state_XY() {
            State state(2);
            if (goal_ != nullptr) {
                state[0] = goal_->x;
                state[1] = goal_->y;
            }
            return state;
        }

        State goal_state_XYQVG() {
            int plan_size = global_state_plan_.size();
            if (plan_size < 2)
                return State(0);
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
            return state;
        }

        bool isInitialized() {
            return map_slice_ != nullptr
                    && odom_ != nullptr
                    && goal_ != nullptr;
        }

        void update_local(SBMPO sbmpo) {
            local_state_plan_ = sbmpo.state_path();
            local_control_plan_ = sbmpo.control_path();
        }

        void update_global(SBMPO sbmpo) {
            global_state_plan_ = sbmpo.state_path();
            global_control_plan_ = sbmpo.control_path();
        }

        private:

        nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr map_slice_;
        nav_msgs::msg::Odometry::ConstSharedPtr odom_;
        geometry_msgs::msg::Point::ConstSharedPtr goal_;

        // SBMPO results
        std::vector<State> local_state_plan_;
        std::vector<Control> local_control_plan_;
        std::vector<State> global_state_plan_;
        std::vector<Control> global_control_plan_;

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