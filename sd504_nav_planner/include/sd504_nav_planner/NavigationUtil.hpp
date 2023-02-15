#ifndef SD_NAV_UTIL_HPP
#define SD_NAV_UTIL_HPP

#include <math.h>

#include "nvblox_msgs/msg/distance_map_slice.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace senior_design {

    class NavigationUtil {

        public:

        // Distance map slice msg
        static nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr distance_map_slice;

        // Odometry msg
        static nav_msgs::msg::Odometry::ConstSharedPtr odometry;

        // Goal point msg
        static geometry_msgs::msg::Point::ConstSharedPtr goal_point;

        // Map lookup function
        static float map_lookup(const float x, const float y) {

            // See if slice exists
            if (distance_map_slice == nullptr)
                return INVALID_DISTANCE;

            // Get the map indices
            float x_index = round((x - distance_map_slice->origin.x) / distance_map_slice->resolution);
            float y_index = round((y - distance_map_slice->origin.y) / distance_map_slice->resolution);

            // Check map bounds
            if (x_index < 0 || x_index >= static_cast<int>(distance_map_slice->width) ||
                y_index < 0 || y_index >= static_cast<int>(distance_map_slice->height))
                return INVALID_DISTANCE;

            // Convert to index
            size_t index = x_index * distance_map_slice->width + y_index;

            // Grab value from slice
            float distance = distance_map_slice->data[index];

            // Check if unknown
            if (distance == distance_map_slice->unknown_value)
                return INVALID_DISTANCE;
            
            return distance;
        }

        // Convert odometry to x,y coordinates
        static std::vector<float> current_XY() {
            if (odometry == nullptr)
                return State(0);
            return {
                odometry->pose.pose.position.x,
                odometry->pose.pose.position.y
            };
        }

        // Convert goal point to x,y coordinates
        static std::vector<float> goal_XY() {
            if (goal_point == nullptr)
                return State(0);
            return {
                goal_point->x,
                goal_point->y
            };
        }

        // Convert odometry to x,y,q,v,g coordinates
        static std::vector<float> current_XYQVG() {
            if (odometry == nullptr)
                return State(0);
            return {
                odometry->pose.pose.position.x,
                odometry->pose.pose.position.y;
                quaternion_to_pitch(odometry->pose.pose.orientation),
                odometry->twist.twist.linear.x,
                rotation_to_ackermann(odometry->twist.twist.angular.z, state[3], WHEEL_BASE_LENGTH)
                /* ^ Think about replacing this with encoder data */
            };
        }
        
        // Convert quaternion msg to theta
        static float quaternion_to_pitch(geometry_msgs::msg::Quaternion quat){
            double w = quat.w;
            double x = quat.x;
            double y = quat.y;
            double z = quat.z;
            double siny_cosp = 2 * (w * z + x * y);
            double cosy_cosp = 1 - 2 * (y * y + z * z);
            return atan2(siny_cosp, cosy_cosp);
        }

        // Convert rotation and velocity to steering angle
        static float rotation_to_ackermann(float omega, float v, float L) {
            return atan2(omega*L,v);
        }

        private:

        NavigationUtil();

    }

}

#endif