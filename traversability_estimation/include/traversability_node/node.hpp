#ifndef TRAVERSABILITY_ESTIMATION__TRAVERSABILITY_NODE_NODE_HPP_
#define TRAVERSABILITY_ESTIMATION__TRAVERSABILITY_NODE_NODE_HPP_

// Other headers
#include "traversability_estimation/point_types.hpp"
#include "traversability_node/params.hpp"
#include "traversability_node/estimation_info.hpp"
#include "traversability_node/estimation_cost.hpp"
#include "traversability_node/filters.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// PCL Headers
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rcpputils/filesystem_helper.hpp>
#include <chrono>

// OpenMP for parallel processing
#include <omp.h>
#include <pcl/features/normal_3d_omp.h>

class TraversabilityEstimationNode : public rclcpp::Node
{
public:
    TraversabilityEstimationNode();
    ~TraversabilityEstimationNode();

private:
    // Configuration parameters
    TraversabilityNodeConfig config_;

    // Callback for receiving point clouds
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // Callback for periodic publishing of traversability results
    void publish_callback();

    // ROS timers, subscriptions, and publishers
    rclcpp::TimerBase::SharedPtr timer_;                                              // Timer for periodic publishing
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;              // Subscription for input point cloud
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_traversable_;     // Publisher for traversable points
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_non_traversable_; // Publisher for non-traversable points
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacle_;        // Publisher for obstacle points
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_boundary_;        // Publisher for boundary points

    // Processed point clouds
    pcl::PointCloud<TraversablePoint>::Ptr traversable_cloud_;     // Cloud for traversable points
    pcl::PointCloud<TraversablePoint>::Ptr non_traversable_cloud_; // Cloud for non-traversable points
    pcl::PointCloud<PointXYZICluster>::Ptr obstacle_cloud_;        // Cloud for clustered obstacles
    pcl::PointCloud<TraversablePoint>::Ptr boundary_cloud_;        // Cloud for boundary points
};

#endif // TRAVERSABILITY_ESTIMATION__TRAVERSABILITY_NODE_NODE_HPP_
