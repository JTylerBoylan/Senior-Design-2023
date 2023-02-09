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
        float map_lookup(const const float x, const float y) {

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
            if (isInitialized()) {
                state[0] = odom_->pose.pose.position.x;
                state[1] = odom_->pose.pose.position.y;
            }
            return state;
        }

        State current_state_XYQVG() {
            State state(5);
            if (isInitialized()) {
                state[0] = odom_->pose.pose.position.x;
                state[1] = odom_->pose.pose.position.y;
                //state[2] = convert odom orientation to yaw
                state[3] = odom_->twist.twist.linear.x;
                //state[4] = convert odom ang. twist to ackerman angle
                // or get from motor encoder data
            }
            return state;
        }

        State goal_state_XY() {
            State state(2);
            // TODO
            return state;
        }

        State goal_state_XYQVG() {
            State state(5);
            // TODO
            return state;
        }

        bool isInitialized() {
            return map_slice_ != nullptr
                    && odom_ != nullptr
                    && goal_ != nullptr;
        }

        private:

        nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr map_slice_;
        nav_msgs::msg::Odometry::ConstSharedPtr odom_;
        geometry_msgs::msg::Point::ConstSharedPtr goal_;

    };

}

#endif