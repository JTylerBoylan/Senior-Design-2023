#ifndef SD504_MODEL_NVBLOX_HPP_
#define SD504_MODEL_NVBLOX_HPP_

#include <sbmpo/sbmpo.hpp>
#include <nvblox_msgs/msg/distance_map_slice.hpp>

#define INVALID_DISTANCE 3.402823466E38f

namespace senior_design {

using namespace sbmpo;

template<typename ModelType>
class SD504Model : public ModelType {
static_assert(std::is_base_of<sbmpo::Model, ModelType>::value, "ModelType must derive from sbmpo::Model");

    public:

    SD504Model() {
        body_radius_ = 1.0f;
        map_bounds_ = {-100.0f, -100.0f, 100.0f, 100.0f};
    }

    // Determine if node is valid (with obstacles and map bounds)
    bool is_valid(const State& state) override {
        
        // Check model function
        if (!ModelType::is_valid(state))
            return false;

        // Bound check
        if (state[0] - map_bounds_[0] < body_radius_ ||
            state[1] - map_bounds_[1] < body_radius_ ||
            map_bounds_[2] - state[0] < body_radius_ ||
            map_bounds_[3] - state[1] < body_radius_)
            return false;

        // Obstacle check
        if (map_lookup_(state[0], state[1]) < body_radius_)
            return false;

        return true;
    }

    /// @brief Change the body dimensions
    /// @param body_radius New body radius value
    void set_body_radius(float body_radius) {
        body_radius_ = body_radius;
    }

    /// @brief Change the map boundaries
    /// @param map_bounds Array of 4 boundary values ([xmin ymin xmax ymax])
    void set_map_bounds(std::array<float, 4> map_bounds) {
        map_bounds_ = map_bounds;
    }

    /// @brief Change the distance map slice
    /// @param map_slice Const shared pointer to map slice message
    void set_map_bounds(nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr map_slice) {
        distance_map_slice_ = map_slice;
    }

    protected:

    float body_radius_;
    std::array<float, 4> map_bounds_;

    nvblox_msgs::msg::DistanceMapSlice::ConstSharedPtr distance_map_slice_;

    // Map lookup function
    static float map_lookup_(const float x, const float y) {

        // See if slice exists
        if (distance_map_slice_ == nullptr)
            return INVALID_DISTANCE;

        // Get the map indices
        float x_index = round((x - distance_map_slice_->origin.x) / distance_map_slice_->resolution);
        float y_index = round((y - distance_map_slice_->origin.y) / distance_map_slice_->resolution);

        // Check map bounds
        if (x_index < 0 || x_index >= static_cast<int>(distance_map_slice_->width) ||
            y_index < 0 || y_index >= static_cast<int>(distance_map_slice_->height))
            return INVALID_DISTANCE;

        // Convert to index
        size_t index = y_index * distance_map_slice_->width + x_index;

        // Grab value from slice
        float distance = distance_map_slice_->data[index];

        // Check if unknown
        if (distance == distance_map_slice_->unknown_value)
            return INVALID_DISTANCE;
        
        return distance;
    }

};

}

#endif