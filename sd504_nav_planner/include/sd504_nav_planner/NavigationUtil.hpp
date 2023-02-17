#ifndef SD_NAV_UTIL_HPP
#define SD_NAV_UTIL_HPP

#include <iostream>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sbmpo/sbmpo.hpp"

#include "nvblox_msgs/msg/distance_map_slice.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace senior_design {

#define INVALID_DISTANCE 3.402823466E38f

using namespace sbmpo;

    const float WHEEL_BASE_LENGTH = 1.0f;
    const float SAMPLE_TIME = 1.0f;
    const float PATH_VISUALIZATION_ELEVATION = 0.0f;

    class NavigationUtil {

        public:

        // Distance map slice msg
        static nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr distance_map_slice;

        // Odometry msg
        static nav_msgs::msg::Odometry::ConstSharedPtr odometry;

        // Goal point msg
        static geometry_msgs::msg::PointStamped::ConstSharedPtr goal_point;

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
            size_t index = y_index * distance_map_slice->width + x_index;

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
                float(odometry->pose.pose.position.x),
                float(odometry->pose.pose.position.y)
            };
        }

        // Convert goal point to x,y coordinates
        static std::vector<float> goal_XY() {
            if (goal_point == nullptr)
                return State(0);
            return {
                float(goal_point->point.x),
                float(goal_point->point.y)
            };
        }

        // Convert odometry to x,y,q,v,g coordinates
        static std::vector<float> current_XYQVG() {
            if (odometry == nullptr)
                return State(0);
            return {
                float(odometry->pose.pose.position.x),
                float(odometry->pose.pose.position.y),
                quaternion_to_pitch(odometry->pose.pose.orientation),
                float(odometry->twist.twist.linear.x),
                0.0f
                // rotation_to_ackermann(odometry->twist.twist.angular.z, odometry->twist.twist.linear.x, WHEEL_BASE_LENGTH)
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

        // Convert global state & control path to a local state path
        static std::vector<State> XY_path_to_XYQVG_path(const std::vector<State> global_state_path, 
                                                            const std::vector<Control> global_control_path) {

            std::vector<State> local_state_path(global_state_path.size());

            for (size_t i = 0; i < global_state_path.size(); i++) {

                bool is_first = i == 0;
                bool is_last = i == global_state_path.size() - 1;

                State state0 = is_first ? State(0) : global_state_path[i - 1];
                State state1 = global_state_path[i];
                State state2 = is_last ? State(0) : global_state_path[i + 1];

                Control control0 = is_first ? Control(0) : global_control_path[i-1];
                Control control1 = is_last ? Control(0) : global_control_path[i];
                
                float theta01 = is_first ? 0.0f : atan2(state1[1] - state0[1], state1[0] - state0[0]);
                float theta12 = is_last ? theta01 : atan2(state2[1] - state1[1], state2[0] - state1[0]);
                float theta = is_first ? theta12 : 0.5f * (theta01 + theta12);

                float velocity0 = is_first ? 0.0f : sqrtf(control0[0]*control0[0] + control0[1]*control0[1]);
                float velocity1 = is_last ? velocity0 : sqrtf(control1[0]*control1[0]+control1[1]*control1[1]);
                float velocity = is_first ? velocity1 : 0.5f * (velocity0 + velocity1);

                float omega = (theta12 - theta01) / SAMPLE_TIME;

                local_state_path[i] = {
                    state1[0], // X
                    state1[1], // Y
                    theta, // Q
                    velocity, // V
                    rotation_to_ackermann(omega, velocity, WHEEL_BASE_LENGTH) // G
                };
            }

            return local_state_path;
        }

        static geometry_msgs::msg::Quaternion yaw_to_quaterion(float yaw) {
            geometry_msgs::msg::Quaternion quat;
            quat.x = 0;
            quat.y = 0;
            quat.z = sinf(yaw/2.0f);
            quat.w = cosf(yaw/2.0f);
            return quat;
        }

        static geometry_msgs::msg::Pose convert_XYQVG_state_to_pose(State xyqvg_state) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = xyqvg_state[0];
            pose.position.y = xyqvg_state[1];
            pose.position.z = PATH_VISUALIZATION_ELEVATION;
            pose.orientation = yaw_to_quaterion(xyqvg_state[3]);
            return pose;
        }

        static nav_msgs::msg::Path convert_XYQVG_path_to_path(std::vector<State> state_path) {
            nav_msgs::msg::Path path;
            for (State state : state_path) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose = convert_XYQVG_state_to_pose(state);
                pose.header.frame_id = "map";
                pose.header.stamp = rclcpp::Clock().now();
                path.poses.push_back(pose);
            }
            path.header.frame_id = "map";
            path.header.stamp = rclcpp::Clock().now();
            return path;
        }

        static geometry_msgs::msg::PointStamped convert_state_to_point(const State &state) {
            geometry_msgs::msg::PointStamped point;
            point.header.frame_id = "map";
            point.header.stamp = rclcpp::Clock().now();
            point.point.x = state[0];
            point.point.y = state[1];
            point.point.z = PATH_VISUALIZATION_ELEVATION;
            return point;
        }

        private:

        NavigationUtil();

    };

    nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr NavigationUtil::distance_map_slice = nullptr;
    nav_msgs::msg::Odometry::ConstSharedPtr NavigationUtil::odometry = nullptr;
    geometry_msgs::msg::PointStamped::ConstSharedPtr NavigationUtil::goal_point = nullptr;

}

#endif