/*
 * InflationFilter.cpp
 *
 *  Created on: August 19, 2024
 *      Author: Your Name
 *   Institute: Your Institution
 */

#include <string>
#include <cmath>

#include "grid_map_filters/InflationFilter.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "grid_map_cv/utilities.hpp"

namespace grid_map
{

template<typename T>
InflationFilter<T>::InflationFilter()
: radius_(0.0), decay_(1.0)
{
}

template<typename T>
InflationFilter<T>::~InflationFilter()
{
}

template<typename T>
bool InflationFilter<T>::configure()
{
    ParameterReader param_reader(this->param_prefix_, this->params_interface_);

    if (!param_reader.get(std::string("radius"), radius_)) {
        RCLCPP_ERROR(
        this->logging_interface_->get_logger(),
        "ForceFieldExpansion filter did not find parameter `radius`.");
        return false;
    }

    if (radius_ < 0.0) {
        RCLCPP_ERROR(
        this->logging_interface_->get_logger(),
        "ForceFieldExpansion filter: Radius must be greater than zero.");
        return false;
    }

    if (!param_reader.get(std::string("decay"), decay_)) {
        RCLCPP_ERROR(
        this->logging_interface_->get_logger(),
        "ForceFieldExpansion filter did not find parameter `decay`.");
        return false;
    }

    if (decay_ <= 0.0) {
        RCLCPP_ERROR(
        this->logging_interface_->get_logger(),
        "ForceFieldExpansion filter: Decay parameter must be greater than zero.");
        return false;
    }

    if (!param_reader.get(std::string("input_layer"), inputLayer_)) {
        RCLCPP_ERROR(
        this->logging_interface_->get_logger(),
        "ForceFieldExpansion filter did not find parameter `input_layer`.");
        return false;
    }

    if (!param_reader.get(std::string("output_layer"), outputLayer_)) {
        RCLCPP_ERROR(
        this->logging_interface_->get_logger(),
        "ForceFieldExpansion filter did not find parameter `output_layer`.");
        return false;
    }
    return true;
}

template<typename T>
bool InflationFilter<T>::update(const T & mapIn, T & mapOut)
{
    // Add new layer to the map.
    mapOut = mapIn;
    mapOut.add(outputLayer_);

    double value;
    Eigen::Vector2d center;
    
    // Remove basic layers so that the whole layer could be displayed
    mapOut.setBasicLayers(std::vector<std::string>());

    // Loop through all the grids in map
    for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
        double maxValue = 0.0;
        mapOut.getPosition(*iterator, center);
        bool has_valid_neighbor = false;

        // Loop through a radius around the target grid
        for (grid_map::CircleIterator submapIterator(mapOut, center, radius_);
            !submapIterator.isPastEnd();
            ++submapIterator) 
        {   
            // Ignore invalid grid
            if (!mapOut.isValid(*submapIterator, inputLayer_)) continue;
            
            has_valid_neighbor = true; // At least one valid neighbor found

            Eigen::Vector2d neighbor;
            mapOut.getPosition(*submapIterator, neighbor);
            double normalized_dist = (center - neighbor).norm();

            value = mapOut.at(inputLayer_, *submapIterator);

            // Compute value based on exponential decay
            double currValue = (1.0 - value) * exp(-normalized_dist * decay_);

            if (currValue > maxValue) maxValue = currValue;
        }
        
        // Only update grid that have at least one valid neighbor
        if (has_valid_neighbor) {
            mapOut.at(outputLayer_, *iterator) = 1.0 - maxValue;
        }

    }

    return true;
}

}  // namespace grid_map

PLUGINLIB_EXPORT_CLASS(
  grid_map::InflationFilter<grid_map::GridMap>,
  filters::FilterBase<grid_map::GridMap>)
