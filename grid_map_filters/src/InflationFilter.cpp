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

    for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
        double forceFieldSum = 0.0;
        double weightSum = 0.0;
        mapOut.getPosition(*iterator, center);

        for (grid_map::CircleIterator submapIterator(mapOut, center, radius_);
        !submapIterator.isPastEnd();
        ++submapIterator)
        {
        if (!mapOut.isValid(*submapIterator, inputLayer_)) {
            continue;
        }
        Eigen::Vector2d neighbor;
        mapOut.getPosition(*submapIterator, neighbor);
        double distance = (center - neighbor).norm();
        double distanceWeight = 1.0 / (distance + 1.0); // Inverse distance weight
        value = mapOut.at(inputLayer_, *submapIterator);

        // Compute force field based on obstacle (lower traversability values)
        double force = (1.0 - value) * distanceWeight * decay_;

        forceFieldSum += force;
        weightSum += distanceWeight;
        }

        if (weightSum > 0) {
        mapOut.at(outputLayer_, *iterator) = 1.0 - std::min(1.0, forceFieldSum / weightSum);
        } else {
        mapOut.at(outputLayer_, *iterator) = 1.0; // Handle the case with no valid neighbors.
        }
    }

    return true;
}

}  // namespace grid_map

PLUGINLIB_EXPORT_CLASS(
  grid_map::InflationFilter<grid_map::GridMap>,
  filters::FilterBase<grid_map::GridMap>)
