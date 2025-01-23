# Header for timestamp and frame information
std_msgs/Header header

# List of nodes as a PointCloud2, storing the traversable cloud
sensor_msgs/PointCloud2 traversable_cloud
sensor_msgs/PointCloud2 nodes

# Start and end indices of edges, corresponding to points in nodes
int32[] edge_start_indices
int32[] edge_end_indices

# Cost associated with each edge
float32[] edge_costs
