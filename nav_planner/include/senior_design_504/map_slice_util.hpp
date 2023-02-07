#ifndef SD_NVBLOX_UTIL_HPP
#define SD_NVBLOX_UTIL_HPP

#include <math.h>

#include "nvblox_msgs/msg/distance_map_slice.hpp"

namespace senior_design {

    const float INVALID_DISTANCE = -1.0f;

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