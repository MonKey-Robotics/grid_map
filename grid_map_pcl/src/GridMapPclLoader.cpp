/*
 * GridMapPclLoader.cpp
 *
 *  Created on: Aug 26, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <chrono>

#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#ifdef GRID_MAP_PCL_OPENMP_FOUND
#include <omp.h>
#endif

#include <pcl/common/io.h>

#include <rclcpp/rclcpp.hpp>

#include "grid_map_core/GridMapMath.hpp"
#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_pcl/helpers.hpp"

namespace grid_map
{

GridMapPclLoader::GridMapPclLoader(const rclcpp::Logger & node_logger)
: node_logger_(node_logger),
  pointcloudProcessor_(node_logger_)
{
  params_ = std::make_unique<grid_map_pcl::PclLoaderParameters>(node_logger_);
}

GridMapPclLoader::~GridMapPclLoader() = default;

const grid_map::GridMap & GridMapPclLoader::getGridMap() const
{
  return workingGridMap_;
}

void GridMapPclLoader::loadCloudFromPcdFile(const std::string & filename)
{
  Pointcloud::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
  inputCloud = grid_map_pcl::loadPointcloudFromPcd(filename);
  setInputCloud(inputCloud);
}

void GridMapPclLoader::setInputCloud(Pointcloud::ConstPtr inputCloud)
{
  setRawInputCloud(inputCloud);
  setWorkingCloud(inputCloud);
}

void GridMapPclLoader::setRawInputCloud(Pointcloud::ConstPtr rawInputCloud)
{
  rawInputCloud_.reset();
  Pointcloud::Ptr temp(new Pointcloud());
  pcl::copyPointCloud(*rawInputCloud, *temp);
  rawInputCloud_ = temp;
}

void GridMapPclLoader::setWorkingCloud(Pointcloud::ConstPtr workingCloud)
{
  workingCloud_.reset();
  Pointcloud::Ptr temp(new Pointcloud());
  pcl::copyPointCloud(*workingCloud, *temp);
  workingCloud_ = temp;
}

void GridMapPclLoader::preProcessInputCloud()
{
  // Preprocess: Remove outliers, downsample cloud, transform cloud
  RCLCPP_INFO_STREAM(node_logger_, "Preprocessing of the pointcloud started");

  if (params_->get().outlierRemoval_.isRemoveOutliers_) {
    auto filteredCloud = pointcloudProcessor_.removeOutliersFromInputCloud(workingCloud_);
    setWorkingCloud(filteredCloud);
  }

  if (params_->get().downsampling_.isDownsampleCloud_) {
    auto downsampledCloud = pointcloudProcessor_.downsampleInputCloud(workingCloud_);
    setWorkingCloud(downsampledCloud);
  }

  auto transformedCloud = pointcloudProcessor_.applyRigidBodyTransformation(workingCloud_);
  setWorkingCloud(transformedCloud);
  RCLCPP_INFO_STREAM(node_logger_, "Preprocessing and filtering finished");
}

void GridMapPclLoader::initializeGridMapGeometryFromInputCloud()
{
  workingGridMap_.clearAll();
  const double resolution = params_->get().gridMap_.resolution_;
  if (resolution < 1e-4) {
    throw std::runtime_error("Desired grid map resolution is zero");
  }

  // find point cloud dimensions
  // min and max coordinate in x,y and z direction
  pcl::PointXYZ minBound;
  pcl::PointXYZ maxBound;
  pcl::getMinMax3D(*workingCloud_, minBound, maxBound);

  // from min and max points we can compute the length
  grid_map::Length length = grid_map::Length(maxBound.x - minBound.x, maxBound.y - minBound.y);

  // we put the center of the grid map to be in the middle of the point cloud
  grid_map::Position position = grid_map::Position(
    (maxBound.x + minBound.x) / 2.0,
    (maxBound.y + minBound.y) / 2.0);
  workingGridMap_.setGeometry(length, resolution, position);

  RCLCPP_INFO_STREAM(
    node_logger_,
    "Grid map dimensions: " << workingGridMap_.getLength()(0) <<
      " x " << workingGridMap_.getLength()(1));
  RCLCPP_INFO_STREAM(
    node_logger_, "Grid map resolution: " << workingGridMap_.getResolution());
  RCLCPP_INFO_STREAM(
    node_logger_,
    "Grid map num cells: " << workingGridMap_.getSize()(0) << " x " <<
      workingGridMap_.getSize()(1));
  RCLCPP_INFO_STREAM(node_logger_, "Initialized map geometry");
}

void GridMapPclLoader::addLayerFromInputCloud(const std::string & layer)
{
  RCLCPP_INFO_STREAM(node_logger_, "Started adding layer: " << layer);
  // Preprocess: allocate memory in the internal data structure
  preprocessGridMapCells();
  workingGridMap_.add(layer);
  grid_map::Matrix & gridMapData = workingGridMap_.get(layer);
  unsigned int linearGridMapSize = workingGridMap_.getSize().prod();

#ifndef GRID_MAP_PCL_OPENMP_FOUND
  RCLCPP_WARN_STREAM(
    node_logger_,
    "OpemMP not found, defaulting to single threaded implementation");
#else
  omp_set_num_threads(params_->get().numThreads_);
#pragma omp parallel for schedule(dynamic, 10)
#endif
  // Iterate through grid map and calculate the corresponding height based on the point cloud
  for (unsigned int linearIndex = 0; linearIndex < linearGridMapSize; ++linearIndex) {
    processGridMapCell(linearIndex, &gridMapData);
  }
  RCLCPP_INFO_STREAM(node_logger_, "Finished adding layer: " << layer);
}

void GridMapPclLoader::processGridMapCell(
  const unsigned int linearGridMapIndex,
  grid_map::Matrix * gridMapData) const
{
  // Get grid map index from linear index and check if enough points lie within the cell
  const grid_map::Index index(
    grid_map::getIndexFromLinearIndex(linearGridMapIndex, workingGridMap_.getSize()));

  Pointcloud::Ptr pointsInsideCellBorder(new Pointcloud());
  pointsInsideCellBorder = getPointcloudInsideGridMapCellBorder(index);
  const bool isTooFewPointsInCell = pointsInsideCellBorder->size() <
    params_->get().gridMap_.minCloudPointsPerCell_;
  if (isTooFewPointsInCell) {
    rclcpp::Clock clock;
    RCLCPP_WARN_STREAM_THROTTLE(
      node_logger_, clock,
      10.0, "Less than " << params_->get().gridMap_.minCloudPointsPerCell_ << " points in a cell");
    return;
  }

  (*gridMapData)(index(0), index(1)) = calculateElevationFromPointsInsideGridMapCell(
    pointsInsideCellBorder);
}

double GridMapPclLoader::calculateElevationFromPointsInsideGridMapCell(
  Pointcloud::ConstPtr cloud) const
{
  // Extract point cloud cluster from point cloud and return if none is found.
  std::vector<Pointcloud::Ptr> clusterClouds =
    pointcloudProcessor_.extractClusterCloudsFromPointcloud(cloud);
  const bool isNoClustersFound = clusterClouds.empty();
  if (isNoClustersFound) {
    rclcpp::Clock clock;
    RCLCPP_WARN_STREAM_THROTTLE(
      node_logger_, clock, 10.0, "No clusters found in the grid map cell");
    return std::nan("1");  // this will leave the grid map cell uninitialized
  }

  // Extract mean z value of cluster vector and return smallest height value
  std::vector<double> clusterHeights(clusterClouds.size());
  std::transform(
    clusterClouds.begin(), clusterClouds.end(), clusterHeights.begin(),
    [](Pointcloud::ConstPtr cloud) -> double {
      return grid_map_pcl::calculateMeanOfPointPositions(cloud).z();
    });

  double height;
  const int height_type = params_->get().gridMap_.height_type_;
  // 0: Smallest value among the average values ​​of each cluster
  if (height_type == 0) {
    height = *(std::min_element(clusterHeights.begin(), clusterHeights.end()));
    // 1: Mean value of the cluster with the most points
  } else if (height_type == 1) {
    const float min_height = *(std::min_element(clusterHeights.begin(), clusterHeights.end()));
    std::vector<int> clusterSizes(clusterClouds.size());
    for (size_t i = 0; i < clusterClouds.size(); i++) {
      clusterSizes[i] = clusterHeights[i] - min_height < params_->get().gridMap_.height_thresh_ ?
        clusterClouds[i]->size() :
        -1;
    }
    const std::vector<int>::iterator maxIt =
      std::max_element(clusterSizes.begin(), clusterSizes.end());
    const size_t maxIndex = std::distance(clusterSizes.begin(), maxIt);
    height = clusterHeights[maxIndex];
  } else {
    throw std::invalid_argument(
            "Invalid height type: " + std::to_string(height_type) +
            "\nValid types are below" +
            "\n0: Smallest value among the average values ​​of each cluster" +
            "\n1: Mean value of the cluster with the most points");
  }

  return height;
}

GridMapPclLoader::Pointcloud::Ptr GridMapPclLoader::getPointcloudInsideGridMapCellBorder(
  const grid_map::Index & index) const
{
  return pointcloudWithinGridMapCell_[index.x()][index.y()];
}

void GridMapPclLoader::loadParameters(rclcpp::Node::SharedPtr node)
{
  params_->loadParameters(node);
  pointcloudProcessor_.loadParameters(node);
}

void GridMapPclLoader::savePointCloudAsPcdFile(const std::string & filename) const
{
  pointcloudProcessor_.savePointCloudAsPcdFile(filename, *workingCloud_);
}

void GridMapPclLoader::preprocessGridMapCells()
{
  allocateSpaceForCloudsInsideCells();
  dispatchWorkingCloudToGridMapCells();
}

void GridMapPclLoader::allocateSpaceForCloudsInsideCells()
{
  const unsigned int dimX = workingGridMap_.getSize().x() + 1;
  const unsigned int dimY = workingGridMap_.getSize().y() + 1;

  // resize vectors
  pointcloudWithinGridMapCell_.resize(dimX);

  // allocate pointClouds
  for (unsigned int i = 0; i < dimX; ++i) {
    pointcloudWithinGridMapCell_[i].resize(dimY);
    for (unsigned int j = 0; j < dimY; ++j) {
      pointcloudWithinGridMapCell_[i][j].reset(new Pointcloud());
    }
  }
}

void GridMapPclLoader::dispatchWorkingCloudToGridMapCells()
{
  // For each point in input point cloud, find which grid map
  // cell does it belong to. Then copy that point in the
  // right cell in the matrix of point clouds data structure.
  // This allows for faster access in the clustering stage.
  for (unsigned int i = 0; i < workingCloud_->points.size(); ++i) {
    const Point & point = workingCloud_->points[i];
    const double x = point.x;
    const double y = point.y;
    grid_map::Index index;
    workingGridMap_.getIndex(grid_map::Position(x, y), index);
    pointcloudWithinGridMapCell_[index.x()][index.y()]->push_back(point);
  }
}
}  // namespace grid_map
