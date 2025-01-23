#ifndef TRAVERSABILITY_ESTIMATION__GRAPH_VISUALIZATION_NODE_HPP_
#define TRAVERSABILITY_ESTIMATION__GRAPH_VISUALIZATION_NODE_HPP_

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include "traversability_estimation_interfaces/srv/get_graph.hpp"
#include "traversability_estimation/point_types.hpp"

class GraphVisualizationNode : public rclcpp::Node
{
public:
    GraphVisualizationNode();

private:
    // Parameters for color gradient
    std::vector<int> color_min_;
    std::vector<int> color_max_;
    float lethal_cost_;
    float node_size_;
    float line_width_;

    // Service client to request the graph
    rclcpp::Client<traversability_estimation_interfaces::srv::GetGraph>::SharedPtr graph_client_;

    // Publisher for visualization markers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // Timer for periodic service calls
    rclcpp::TimerBase::SharedPtr timer_;

    // Callback for timer to request the graph and publish markers
    void timer_callback();

    // Helper function to convert graph data to markers
    visualization_msgs::msg::MarkerArray convert_graph_to_markers(
        const traversability_estimation_interfaces::msg::Graph &graph);

    // Helper function to compute color based on cost
    std_msgs::msg::ColorRGBA get_cost_color(float cost);
};

#endif // TRAVERSABILITY_ESTIMATION__GRAPH_VISUALIZATION_NODE_HPP_
