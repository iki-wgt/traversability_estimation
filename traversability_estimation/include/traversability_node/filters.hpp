#ifndef TRAVERSABILITY_ESTIMATION__TRAVERSABILITY_NODE_FILTERS_HPP_
#define TRAVERSABILITY_ESTIMATION__TRAVERSABILITY_NODE_FILTERS_HPP_

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <limits>

#include "traversability_node/params.hpp"
#include "traversability_estimation/point_types.hpp"

void voxel_down_sampling(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, TraversabilityNodeConfig params);

void radius_outlier_removal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, TraversabilityNodeConfig params);

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr euclidian_clustering(
    typename pcl::PointCloud<PointT>::Ptr input_cloud,
    TraversabilityNodeConfig params);

#endif // TRAVERSABILITY_ESTIMATION__TRAVERSABILITY_NODE_FILTERS_HPP_
