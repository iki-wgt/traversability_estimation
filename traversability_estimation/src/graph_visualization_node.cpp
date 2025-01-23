#include "graph_visualization_node.hpp"
#include "traversability_estimation_interfaces/srv/get_graph.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

GraphVisualizationNode::GraphVisualizationNode()
    : Node("graph_visualization_node")
{
    // Declare color parameters
    this->declare_parameter("color_min", std::vector<int>{0, 85, 0});
    this->declare_parameter("color_max", std::vector<int>{255, 170, 0});
    this->declare_parameter("lethal_cost", 100.0);
    this->declare_parameter("node_size", 0.1);
    this->declare_parameter("line_width", 0.01);


    // Get color parameters and convert to int vectors
    auto color_min_long = this->get_parameter("color_min").as_integer_array();
    auto color_max_long = this->get_parameter("color_max").as_integer_array();

    color_min_.assign(color_min_long.begin(), color_min_long.end());
    color_max_.assign(color_max_long.begin(), color_max_long.end());
    lethal_cost_ = this->get_parameter("lethal_cost").as_double();
    node_size_ = this->get_parameter("node_size").as_double();
    line_width_ = this->get_parameter("line_width").as_double();

    // Create a service client to get the graph
    graph_client_ = this->create_client<traversability_estimation_interfaces::srv::GetGraph>("get_graph");

    // Create publisher for marker array
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("graph_markers", 10);

    // Set up a timer to call the service periodically
    timer_ = this->create_wall_timer(
        std::chrono::seconds(10),
        std::bind(&GraphVisualizationNode::timer_callback, this));
}

void GraphVisualizationNode::timer_callback()
{
    // Check if the service is available
    if (!graph_client_->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_WARN(this->get_logger(), "Service not available, waiting...");
        return;
    }

    // Create the service request
    auto request = std::make_shared<traversability_estimation_interfaces::srv::GetGraph::Request>();

    // Call the service asynchronously and provide a callback to handle the response
    auto result_future = graph_client_->async_send_request(request,
                                                           [this](rclcpp::Client<traversability_estimation_interfaces::srv::GetGraph>::SharedFuture future_response)
                                                           {
                                                               try
                                                               {
                                                                   auto response = future_response.get();
                                                                   visualization_msgs::msg::MarkerArray marker_array = convert_graph_to_markers(response->graph);
                                                                   marker_pub_->publish(marker_array);
                                                               }
                                                               catch (const std::exception &e)
                                                               {
                                                                   RCLCPP_ERROR(this->get_logger(), "Failed to process graph response: %s", e.what());
                                                               }
                                                           });
}

visualization_msgs::msg::MarkerArray GraphVisualizationNode::convert_graph_to_markers(
    const traversability_estimation_interfaces::msg::Graph &graph)
{
    visualization_msgs::msg::MarkerArray marker_array;
    RCLCPP_INFO(this->get_logger(), "Converting graph to markers");

    // Convert the graph's nodes from PointCloud2 to pcl::PointCloud<TraversablePoint>
    pcl::PointCloud<TraversablePoint> pcl_nodes;
    pcl::fromROSMsg(graph.nodes, pcl_nodes);

    // Create a SPHERE_LIST marker for nodes
    visualization_msgs::msg::Marker node_marker;
    node_marker.header.frame_id = graph.header.frame_id;
    node_marker.header.stamp = this->now();
    node_marker.ns = "nodes";
    node_marker.id = 0;
    node_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    node_marker.scale.x = node_size_;
    node_marker.scale.y = node_size_;
    node_marker.scale.z = node_size_;
    node_marker.color.a = 1.0;

    for (const auto &node : pcl_nodes.points)
    {
        geometry_msgs::msg::Point p;
        p.x = node.x;
        p.y = node.y;
        p.z = node.z;
        node_marker.points.push_back(p);

        // Adjust color based on node cost
        std_msgs::msg::ColorRGBA color = get_cost_color(node.final_cost);
        node_marker.colors.push_back(color);
    }

    // Create a LINE_LIST marker for edges
    visualization_msgs::msg::Marker edge_marker;
    edge_marker.header.frame_id = graph.header.frame_id;
    edge_marker.header.stamp = this->now();
    edge_marker.ns = "edges";
    edge_marker.id = 1;
    edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    edge_marker.scale.x = line_width_;

    for (size_t i = 0; i < graph.edge_start_indices.size(); ++i)
    {
        int start_idx = graph.edge_start_indices[i];
        int end_idx = graph.edge_end_indices[i];
        float cost = graph.edge_costs[i];

        // Start and end points for the edge
        geometry_msgs::msg::Point start, end;
        start.x = pcl_nodes.points[start_idx].x;
        start.y = pcl_nodes.points[start_idx].y;
        start.z = pcl_nodes.points[start_idx].z;

        end.x = pcl_nodes.points[end_idx].x;
        end.y = pcl_nodes.points[end_idx].y;
        end.z = pcl_nodes.points[end_idx].z;

        edge_marker.points.push_back(start);
        edge_marker.points.push_back(end);

        // Adjust color based on edge cost
        std_msgs::msg::ColorRGBA color = get_cost_color(cost);
        edge_marker.colors.push_back(color);
        edge_marker.colors.push_back(color); // Need two colors for each line segment
    }

    // Create a LINE_LIST marker for normals
    visualization_msgs::msg::Marker normal_marker;
    normal_marker.header.frame_id = graph.header.frame_id;
    normal_marker.header.stamp = this->now();
    normal_marker.ns = "normals";
    normal_marker.id = 2;
    normal_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    normal_marker.scale.x = line_width_;

    for (const auto &node : pcl_nodes.points)
    {
        // Origin of the normal
        geometry_msgs::msg::Point origin;
        origin.x = node.x;
        origin.y = node.y;
        origin.z = node.z;

        // Endpoint of the normal
        geometry_msgs::msg::Point endpoint;
        endpoint.x = node.x + node_size_ * node.normal_x;
        endpoint.y = node.y + node_size_ * node.normal_y;
        endpoint.z = node.z + node_size_ * node.normal_z;

        normal_marker.points.push_back(origin);
        normal_marker.points.push_back(endpoint);

        // Color the normal arrow in red
        std_msgs::msg::ColorRGBA normal_color;
        normal_color.r = 1.0;
        normal_color.a = 1.0;
        normal_marker.colors.push_back(normal_color);
        normal_marker.colors.push_back(normal_color); // Two colors for each line segment
    }

    marker_array.markers.push_back(node_marker);
    marker_array.markers.push_back(edge_marker);
    marker_array.markers.push_back(normal_marker);

    return marker_array;
}

std_msgs::msg::ColorRGBA GraphVisualizationNode::get_cost_color(float cost)
{
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0; // fully opaque

    float t = std::clamp(cost / lethal_cost_, 0.0f, 1.0f);
    color.r = (color_min_[0] + t * (color_max_[0] - color_min_[0])) / 255.0;
    color.g = (color_min_[1] + t * (color_max_[1] - color_min_[1])) / 255.0;
    color.b = (color_min_[2] + t * (color_max_[2] - color_min_[2])) / 255.0;

    return color;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GraphVisualizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
