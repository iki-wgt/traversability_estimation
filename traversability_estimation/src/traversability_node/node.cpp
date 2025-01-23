#include "traversability_node/node.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <rcpputils/filesystem_helper.hpp>
#include <omp.h>
#include <chrono>

/**
 * @brief Constructor for the TraversabilityEstimationNode class.
 *
 * Initializes the node, loads parameters, sets up publishers and subscribers, and starts processing.
 */
TraversabilityEstimationNode::TraversabilityEstimationNode()
    : Node("traversability_estimation")
{
    // Load parameters from configuration
    load_parameters(config_, this);

    // Set the number of threads for OpenMP
    omp_set_num_threads(config_.common.n_threads);

    // Setup timer for periodic publishing based on the publish rate
    this->timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / config_.common.publish_rate),
        std::bind(&TraversabilityEstimationNode::publish_callback, this));

    // Create publishers for traversable, non-traversable, and obstacle point clouds
    pub_traversable_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("traversable", 1);
    pub_non_traversable_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("non_traversable", 1);
    pub_obstacle_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("obstacle", 1);
    pub_boundary_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("boundary", 1);

    // Initialize point clouds
    traversable_cloud_ = std::make_shared<pcl::PointCloud<TraversablePoint>>();
    non_traversable_cloud_ = std::make_shared<pcl::PointCloud<TraversablePoint>>();
    obstacle_cloud_ = std::make_shared<pcl::PointCloud<PointXYZICluster>>();
    boundary_cloud_ = std::make_shared<pcl::PointCloud<TraversablePoint>>();

    // Determine if the input_cloud is a .pcd file or a topic
    if (rcpputils::fs::path(config_.common.input_cloud).extension().string() == ".pcd")
    {
        // Load PCD file from disk
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(config_.common.input_cloud, *cloud) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file: %s", config_.common.input_cloud.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded PCD file: %s", config_.common.input_cloud.c_str());

        // Convert PCL cloud to ROS message and trigger callback
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud, output_msg);
        output_msg.header.frame_id = config_.common.map_frame;
        callback(std::make_shared<sensor_msgs::msg::PointCloud2>(output_msg));
    }
    else
    {
        // Subscribe to topic if it's not a .pcd file
        RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", config_.common.input_cloud.c_str());
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            config_.common.input_cloud,
            1,
            std::bind(&TraversabilityEstimationNode::callback, this, std::placeholders::_1));
    }
}

TraversabilityEstimationNode::~TraversabilityEstimationNode() {}

/**
 * @brief Callback function for processing incoming point cloud messages.
 *
 * @param msg The incoming point cloud message.
 */
void TraversabilityEstimationNode::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (!traversable_cloud_->points.empty()) // Don't process if already processed
    {
        return;
    }
    using Clock = std::chrono::high_resolution_clock;

    // Start timer for the entire callback
    auto start_time = Clock::now();

    // Convert to pcl::PointXYZI
    auto start_conversion = Clock::now();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);
    auto end_conversion = Clock::now();
    RCLCPP_INFO(this->get_logger(), "Point cloud conversion took: %.3f ms",
                std::chrono::duration<double, std::milli>(end_conversion - start_conversion).count());

    // Apply Voxel Down Sampling
    auto start_voxel_down_sampling = Clock::now();
    voxel_down_sampling(cloud, config_);
    auto end_voxel_down_sampling = Clock::now();
    RCLCPP_INFO(this->get_logger(), "Voxel down sampling took: %.3f ms",
                std::chrono::duration<double, std::milli>(end_voxel_down_sampling - start_voxel_down_sampling).count());

    // Apply Radius Outlier Removal
    auto start_outlier_removal = Clock::now();
    radius_outlier_removal(cloud, config_);
    auto end_outlier_removal = Clock::now();
    RCLCPP_INFO(this->get_logger(), "Radius outlier removal took: %.3f ms",
                std::chrono::duration<double, std::milli>(end_outlier_removal - start_outlier_removal).count());

    // Clear previous clouds
    traversable_cloud_->clear();
    non_traversable_cloud_->clear();
    obstacle_cloud_->clear();
    boundary_cloud_->clear();

    // Traversability estimation
    auto start_traversable_estimation = Clock::now();
    perform_traversability_estimation(cloud, traversable_cloud_, obstacle_cloud_, config_);
    auto end_traversable_estimation = Clock::now();
    RCLCPP_INFO(this->get_logger(), "Traversability estimation took: %.3f ms",
                std::chrono::duration<double, std::milli>(end_traversable_estimation - start_traversable_estimation).count());

    // Boundary estimation
    auto start_compute_boundary = Clock::now();
    compute_boundary(traversable_cloud_, boundary_cloud_, config_);
    auto end_compute_boundary = Clock::now();
    RCLCPP_INFO(this->get_logger(), "Boundary computation took: %.3f ms",
                std::chrono::duration<double, std::milli>(end_compute_boundary - start_compute_boundary).count());

    // Cost computation
    auto start_compute_costs = Clock::now();
    compute_costs(traversable_cloud_, non_traversable_cloud_, obstacle_cloud_, boundary_cloud_, config_);
    auto end_compute_costs = Clock::now();
    RCLCPP_INFO(this->get_logger(), "Cost computation took: %.3f ms",
                std::chrono::duration<double, std::milli>(end_compute_costs - start_compute_costs).count());

    // Log total processing time for the callback
    auto end_time = Clock::now();
    RCLCPP_INFO(this->get_logger(), "Total callback processing time: %.3f ms",
                std::chrono::duration<double, std::milli>(end_time - start_time).count());
}

/**
 * @brief Periodic callback function for publishing processed point clouds.
 */
void TraversabilityEstimationNode::publish_callback()
{
    // Publish traversable cloud
    if (!traversable_cloud_->empty())
    {
        sensor_msgs::msg::PointCloud2 traversable_msg;
        pcl::toROSMsg(*traversable_cloud_, traversable_msg);
        traversable_msg.header.stamp = this->get_clock()->now();
        traversable_msg.header.frame_id = config_.common.map_frame;
        pub_traversable_->publish(traversable_msg);
    }

    // Publish non-traversable cloud
    if (!non_traversable_cloud_->empty())
    {
        sensor_msgs::msg::PointCloud2 non_traversable_msg;
        pcl::toROSMsg(*non_traversable_cloud_, non_traversable_msg);
        non_traversable_msg.header.stamp = this->get_clock()->now();
        non_traversable_msg.header.frame_id = config_.common.map_frame;
        pub_non_traversable_->publish(non_traversable_msg);
    }

    // Publish obstacle cloud
    if (!obstacle_cloud_->empty())
    {
        sensor_msgs::msg::PointCloud2 obstacle_msg;
        pcl::toROSMsg(*obstacle_cloud_, obstacle_msg);
        obstacle_msg.header.stamp = this->get_clock()->now();
        obstacle_msg.header.frame_id = config_.common.map_frame;
        pub_obstacle_->publish(obstacle_msg);
    }

    // Publish boundary cloud
    if (!boundary_cloud_->empty())
    {
        sensor_msgs::msg::PointCloud2 boundary_msg;
        pcl::toROSMsg(*boundary_cloud_, boundary_msg);
        boundary_msg.header.stamp = this->get_clock()->now();
        boundary_msg.header.frame_id = config_.common.map_frame;
        pub_boundary_->publish(boundary_msg);
    }
}
