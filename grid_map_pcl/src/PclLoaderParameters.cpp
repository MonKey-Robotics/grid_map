/*
 * PclLoaderParameters.cpp
 *
 *  Created on: Nov 7, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include "grid_map_pcl/PclLoaderParameters.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace grid_map
{

namespace grid_map_pcl
{

PclLoaderParameters::PclLoaderParameters(const rclcpp::Logger & node_logger)
: node_logger_(node_logger) {}

void PclLoaderParameters::handleYamlNode(const YAML::Node & yamlNode)
{
  const std::string prefix = "pcl_grid_map_extraction";

  parameters_.numThreads_ =
    yamlNode[prefix]["num_processing_threads"].as<int>();

  parameters_.cloudTransformation_.translation_.x() =
    yamlNode[prefix]["cloud_transform"]["translation"]["x"].as<double>();
  parameters_.cloudTransformation_.translation_.y() =
    yamlNode[prefix]["cloud_transform"]["translation"]["y"].as<double>();
  parameters_.cloudTransformation_.translation_.z() =
    yamlNode[prefix]["cloud_transform"]["translation"]["z"].as<double>();

  parameters_.cloudTransformation_.rpyIntrinsic_.x() =
    yamlNode[prefix]["cloud_transform"]["rotation"]["r"].as<double>();
  parameters_.cloudTransformation_.rpyIntrinsic_.y() =
    yamlNode[prefix]["cloud_transform"]["rotation"]["p"].as<double>();
  parameters_.cloudTransformation_.rpyIntrinsic_.z() =
    yamlNode[prefix]["cloud_transform"]["rotation"]["y"].as<double>();

  parameters_.clusterExtraction_.clusterTolerance_ =
    yamlNode[prefix]["cluster_extraction"]["cluster_tolerance"].as<
    double>();
  parameters_.clusterExtraction_.minNumPoints_ =
    yamlNode[prefix]["cluster_extraction"]["min_num_points"].as<int>();
  parameters_.clusterExtraction_.maxNumPoints_ =
    yamlNode[prefix]["cluster_extraction"]["max_num_points"].as<int>();

  parameters_.outlierRemoval_.isRemoveOutliers_ =
    yamlNode[prefix]["outlier_removal"]["is_remove_outliers"].as<bool>();
  parameters_.outlierRemoval_.meanK_ =
    yamlNode[prefix]["outlier_removal"]["mean_K"].as<int>();
  parameters_.outlierRemoval_.stddevThreshold_ =
    yamlNode[prefix]["outlier_removal"]["stddev_threshold"].as<double>();

  parameters_.gridMap_.resolution_ =
    yamlNode[prefix]["grid_map"]["resolution"].as<double>();
  parameters_.gridMap_.minCloudPointsPerCell_ =
    yamlNode[prefix]["grid_map"]["min_num_points_per_cell"].as<int>();
  parameters_.gridMap_.height_type_ =
    yamlNode[prefix]["grid_map"]["height_type"].as<int>();
  parameters_.gridMap_.height_thresh_ =
    yamlNode[prefix]["grid_map"]["height_thresh"].as<double>();

  parameters_.downsampling_.isDownsampleCloud_ =
    yamlNode[prefix]["downsampling"]["is_downsample_cloud"].as<bool>();
  parameters_.downsampling_.voxelSize_.x() =
    yamlNode[prefix]["downsampling"]["voxel_size"]["x"].as<double>();
  parameters_.downsampling_.voxelSize_.y() =
    yamlNode[prefix]["downsampling"]["voxel_size"]["y"].as<double>();
  parameters_.downsampling_.voxelSize_.z() =
    yamlNode[prefix]["downsampling"]["voxel_size"]["z"].as<double>();
}

bool PclLoaderParameters::loadParameters(rclcpp::Node::SharedPtr node)
{
  const std::string prefix = "pcl_grid_map_extraction";

  // Helper lambda to declare parameter if not already declared
  auto declare_parameter_if_not_declared = [&](const std::string & name, auto default_value) {
    if (!node->has_parameter(name)) {
      node->declare_parameter(name, default_value);
    }
  };

  // Number of Threads
  declare_parameter_if_not_declared(prefix + ".num_processing_threads", static_cast<int>(parameters_.numThreads_));
  parameters_.numThreads_ = node->get_parameter(prefix + ".num_processing_threads").as_int();

  // Cloud Transformation - Translation
  declare_parameter_if_not_declared(prefix + ".cloud_transform.translation.x", 0.0);
  declare_parameter_if_not_declared(prefix + ".cloud_transform.translation.y", 0.0);
  declare_parameter_if_not_declared(prefix + ".cloud_transform.translation.z", 0.0);

  parameters_.cloudTransformation_.translation_.x() =
    node->get_parameter(prefix + ".cloud_transform.translation.x").as_double();
  parameters_.cloudTransformation_.translation_.y() =
    node->get_parameter(prefix + ".cloud_transform.translation.y").as_double();
  parameters_.cloudTransformation_.translation_.z() =
    node->get_parameter(prefix + ".cloud_transform.translation.z").as_double();

  // Cloud Transformation - Rotation
  declare_parameter_if_not_declared(prefix + ".cloud_transform.rotation.r", 0.0);
  declare_parameter_if_not_declared(prefix + ".cloud_transform.rotation.p", 0.0);
  declare_parameter_if_not_declared(prefix + ".cloud_transform.rotation.y", 0.0);

  parameters_.cloudTransformation_.rpyIntrinsic_.x() =
    node->get_parameter(prefix + ".cloud_transform.rotation.r").as_double();
  parameters_.cloudTransformation_.rpyIntrinsic_.y() =
    node->get_parameter(prefix + ".cloud_transform.rotation.p").as_double();
  parameters_.cloudTransformation_.rpyIntrinsic_.z() =
    node->get_parameter(prefix + ".cloud_transform.rotation.y").as_double();

  // Cluster Extraction
  declare_parameter_if_not_declared(prefix + ".cluster_extraction.cluster_tolerance", 0.5);
  declare_parameter_if_not_declared(prefix + ".cluster_extraction.min_num_points", 1);
  declare_parameter_if_not_declared(prefix + ".cluster_extraction.max_num_points", 1000000);

  parameters_.clusterExtraction_.clusterTolerance_ =
    node->get_parameter(prefix + ".cluster_extraction.cluster_tolerance").as_double();
  parameters_.clusterExtraction_.minNumPoints_ =
    node->get_parameter(prefix + ".cluster_extraction.min_num_points").as_int();
  parameters_.clusterExtraction_.maxNumPoints_ =
    node->get_parameter(prefix + ".cluster_extraction.max_num_points").as_int();

  // Outlier Removal
  declare_parameter_if_not_declared(prefix + ".outlier_removal.is_remove_outliers", false);
  declare_parameter_if_not_declared(prefix + ".outlier_removal.mean_K", 30.0);
  declare_parameter_if_not_declared(prefix + ".outlier_removal.stddev_threshold", 1.0);

  parameters_.outlierRemoval_.isRemoveOutliers_ =
    node->get_parameter(prefix + ".outlier_removal.is_remove_outliers").as_bool();
  parameters_.outlierRemoval_.meanK_ =
    node->get_parameter(prefix + ".outlier_removal.mean_K").as_double();
  parameters_.outlierRemoval_.stddevThreshold_ =
    node->get_parameter(prefix + ".outlier_removal.stddev_threshold").as_double();

  // Grid Map Parameters
  declare_parameter_if_not_declared(prefix + ".grid_map.resolution", 0.1);
  declare_parameter_if_not_declared(prefix + ".grid_map.min_num_points_per_cell", 1);
  declare_parameter_if_not_declared(prefix + ".grid_map.height_type", 1);
  declare_parameter_if_not_declared(prefix + ".grid_map.height_thresh", 0.1);

  parameters_.gridMap_.resolution_ =
    node->get_parameter(prefix + ".grid_map.resolution").as_double();
  parameters_.gridMap_.minCloudPointsPerCell_ =
    node->get_parameter(prefix + ".grid_map.min_num_points_per_cell").as_int();
  parameters_.gridMap_.height_type_ =
    node->get_parameter(prefix + ".grid_map.height_type").as_int();
  parameters_.gridMap_.height_thresh_ =
    node->get_parameter(prefix + ".grid_map.height_thresh").as_double();

  // Downsampling Parameters
  declare_parameter_if_not_declared(prefix + ".downsampling.is_downsample_cloud", false);
  declare_parameter_if_not_declared(prefix + ".downsampling.voxel_size.x", 0.1);
  declare_parameter_if_not_declared(prefix + ".downsampling.voxel_size.y", 0.1);
  declare_parameter_if_not_declared(prefix + ".downsampling.voxel_size.z", 0.1);

  parameters_.downsampling_.isDownsampleCloud_ =
    node->get_parameter(prefix + ".downsampling.is_downsample_cloud").as_bool();
  parameters_.downsampling_.voxelSize_.x() =
    node->get_parameter(prefix + ".downsampling.voxel_size.x").as_double();
  parameters_.downsampling_.voxelSize_.y() =
    node->get_parameter(prefix + ".downsampling.voxel_size.y").as_double();
  parameters_.downsampling_.voxelSize_.z() =
    node->get_parameter(prefix + ".downsampling.voxel_size.z").as_double();
  
  return true;
}

const PclLoaderParameters::Parameters & PclLoaderParameters::get() const
{
  return parameters_;
}

}  // namespace grid_map_pcl

}  // namespace grid_map
