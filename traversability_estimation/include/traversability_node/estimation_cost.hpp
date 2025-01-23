#ifndef TRAVERSABILITY_ESTIMATION__TRAVERSABILITY_NODE_ESTIMATION_COST_HPP_
#define TRAVERSABILITY_ESTIMATION__TRAVERSABILITY_NODE_ESTIMATION_COST_HPP_

#include <pcl/point_cloud.h>
#include "traversability_estimation/point_types.hpp"
#include "traversability_node/params.hpp"

// Cost computation functions with parameter access
void compute_slope_cost(pcl::PointCloud<TraversablePoint>::Ptr traversable_cloud, TraversabilityNodeConfig params);

void compute_inflation_cost(pcl::PointCloud<TraversablePoint>::Ptr traversable_cloud,
                            pcl::PointCloud<PointXYZICluster>::Ptr obstacle_cloud,
                            pcl::PointCloud<TraversablePoint>::Ptr boundary_cloud,
                            TraversabilityNodeConfig params);

void compute_costs(pcl::PointCloud<TraversablePoint>::Ptr traversable_cloud,
                   pcl::PointCloud<TraversablePoint>::Ptr non_traversable_cloud,
                   pcl::PointCloud<PointXYZICluster>::Ptr obstacle_cloud,
                   pcl::PointCloud<TraversablePoint>::Ptr boundary_cloud,
                   TraversabilityNodeConfig params);

#endif // TRAVERSABILITY_ESTIMATION__TRAVERSABILITY_NODE_ESTIMATION_COST_HPP_
