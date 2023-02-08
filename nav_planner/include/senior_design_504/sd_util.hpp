#ifndef SD_UTIL_HPP
#define SD_UTIL_HPP

#include <math.h>

#include "nvblox_msgs/msg/distance_map_slice.hpp"
#include "sbmpo/types.hpp"

namespace senior_design {

    const float INVALID_DISTANCE = -1.0f;

    // Map lookup function
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

    // Convert float array into controls (for ROS params)
    std::vector<sbmpo::Control> array_to_controls(const std::vector<double> array, int num_controls) {
        int size = array.size() / num_controls;
        std::vector<sbmpo::Control> controls(size);
        for (int i = 0; i < size; i++) {
            sbmpo::Control control(num_controls);
            for (int j = 0; j < num_controls; j++)
                control.push_back(float(array[num_controls*i + j]));
            controls.push_back(control);
        }
    }

}

#endif