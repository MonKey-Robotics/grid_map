/*
 * grid_map_pcl_loader_node.cpp
 *
 *  Created on: Aug 26, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <memory>
#include <string>
#include <utility>

#include "grid_map_core/GridMap.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "filters/filter_chain.hpp"
#include "grid_map_pcl/helpers.hpp"

namespace gm = ::grid_map::grid_map_pcl;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("grid_map_pcl_loader_node");
  gm::setVerbosityLevelToDebugIfFlagSet(node);

  rclcpp::QoS custom_qos = rclcpp::QoS(1).transient_local();
  auto gridMapPub = node->create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map_from_raw_pointcloud", custom_qos);

  grid_map::GridMapPclLoader gridMapPclLoader(node->get_logger());
  const std::string pathToCloud = gm::getPcdFilePath(node);
  gridMapPclLoader.loadParameters(node);
  gridMapPclLoader.loadCloudFromPcdFile(pathToCloud);

  gm::processPointcloud(&gridMapPclLoader, node);

  grid_map::GridMap gridMap = gridMapPclLoader.getGridMap();
  gridMap.setFrameId(gm::getMapFrame(node));

  // Apply filter chain if use_filter_chain is set to true
  if (!node->has_parameter("use_filter_chain")) {
    node->declare_parameter("use_filter_chain", true);
  }
  bool use_filter_chain = node->get_parameter("use_filter_chain").as_bool();

  if (use_filter_chain) {
    // Setup filter chain
    filters::FilterChain<grid_map::GridMap> filterChain("grid_map::GridMap");
    if (filterChain.configure(
        "filters", node->get_node_logging_interface(),
        node->get_node_parameters_interface()))
    {
        RCLCPP_INFO(node->get_logger(), "Filter chain configured.");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Could not configure the filter chain!");
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    // Apply filter chain.
    RCLCPP_INFO(node->get_logger(), "Applying filter chain...");
    if (!filterChain.update(gridMap, gridMap)) {
        RCLCPP_ERROR(node->get_logger(), "Could not update the grid map filter chain!");
        return EXIT_FAILURE;
    }
  }

  gm::saveGridMap(gridMap, node, gm::getMapRosbagTopic(node));

  // publish grid map
  auto msg = grid_map::GridMapRosConverter::toMessage(gridMap);
  gridMapPub->publish(std::move(msg));

  // run
  rclcpp::spin(node->get_node_base_interface());
  return EXIT_SUCCESS;
}
