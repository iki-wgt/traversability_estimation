#ifndef TRAVERSABILITY_ESTIMATION__GRAPH_HPP_
#define TRAVERSABILITY_ESTIMATION__GRAPH_HPP_

#include <unordered_map>
#include <vector>
#include <utility>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>

#include "traversability_estimation/point_types.hpp"
#include "traversability_estimation_interfaces/msg/graph.hpp"
#include "graph_planning_node/params.hpp"

class Graph
{
public:
    using AdjacencyList = std::unordered_map<int, std::vector<std::pair<int, float>>>;

    Graph();
    ~Graph();

    void build_graph(pcl::PointCloud<TraversablePoint>::Ptr traversable_cloud, const GraphPlanningNodeConfig &config);

    float compute_alignment_cost(const TraversablePoint &from_point, const TraversablePoint &to_point, const GraphPlanningNodeConfig &config) const;

    traversability_estimation_interfaces::msg::Graph to_ros_message() const;

    // Find the closest point in the graph to a given position
    int find_closest_point(const Eigen::Vector3f &point, double max_search_radius) const;
    int find_closest_node(const Eigen::Vector3f &point, double max_search_radius) const;

    // Accessors for nodes_cloud_ and adjacency_list_
    pcl::PointCloud<TraversablePoint>::Ptr get_nodes_cloud() const
    {
        return nodes_cloud_;
    }
    pcl::PointCloud<TraversablePoint>::Ptr get_traversable_cloud() const { return traversable_cloud_; }
    const AdjacencyList &get_adjacency_list() const { return adjacency_list_; }

private:
    AdjacencyList adjacency_list_;
    pcl::KdTreeFLANN<TraversablePoint> kdtree_;
    pcl::KdTreeFLANN<TraversablePoint> nodes_kdtree_;
    pcl::PointCloud<TraversablePoint>::Ptr traversable_cloud_;
    pcl::PointCloud<TraversablePoint>::Ptr nodes_cloud_;
};

#endif // TRAVERSABILITY_ESTIMATION__GRAPH_HPP_
