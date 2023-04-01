#ifndef SD_NAV_UTIL_HPP
#define SD_NAV_UTIL_HPP

#include <rclcpp/rclcpp.hpp>

#include <math.h>

#include <sbmpo/sbmpo.hpp>

#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nvblox_msgs/msg/distance_map_slice.hpp>

namespace senior_design {

#define INVALID_DISTANCE 3.402823466E38f

using namespace sbmpo;

    const float WHEEL_BASE_LENGTH = 1.0f;
    const float SAMPLE_TIME = 1.0f;
    const float PATH_VISUALIZATION_ELEVATION = 0.0f;

    class NavigationUtil {

        public:
        
        // Convert quaternion msg to theta
        static float quaternion_to_pitch(geometry_msgs::msg::Quaternion quat){
            const double w = quat.w;
            const double x = quat.x;
            const double y = quat.y;
            const double z = quat.z;
            const double siny_cosp = 2 * (w * z + x * y);
            const double cosy_cosp = 1 - 2 * (y * y + z * z);
            return atan2(siny_cosp, cosy_cosp);
        }

        // Convert rotation and velocity to steering angle
        static float rotation_to_ackermann(float omega, float v, float L) {
            return atan2f(omega*L,v);
        }

        // Convert global state & control path to a local state path
        static std::vector<State> XY_path_to_XYQVG_path(const std::vector<State>& global_state_path, 
                                                            const std::vector<Control>& global_control_path) {

            std::vector<State> local_state_path(global_state_path.size());

            for (size_t i = 0; i < global_state_path.size(); i++) {

                const bool is_first = i == 0;
                const bool is_last = i == global_state_path.size() - 1;

                const State state0 = is_first ? State(0) : global_state_path[i - 1];
                const State state1 = global_state_path[i];
                const State state2 = is_last ? State(0) : global_state_path[i + 1];

                const Control control0 = is_first ? Control(0) : global_control_path[i-1];
                const Control control1 = is_last ? Control(0) : global_control_path[i];
                
                const float theta01 = is_first ? 0.0f : std::atan2(state1[1] - state0[1], state1[0] - state0[0]);
                const float theta12 = is_last ? theta01 : std::atan2(state2[1] - state1[1], state2[0] - state1[0]);
                const float theta = is_first ? theta12 : 0.5f * (theta01 + theta12);

                const float velocity0 = is_first ? 0.0f : sqrtf(control0[0]*control0[0] + control0[1]*control0[1]);
                const float velocity1 = is_last ? velocity0 : sqrtf(control1[0]*control1[0]+control1[1]*control1[1]);
                const float velocity = is_first ? velocity1 : 0.5f * (velocity0 + velocity1);

                const float omega = (theta12 - theta01) / SAMPLE_TIME;

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


    };

}

#endif