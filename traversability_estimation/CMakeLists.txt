cmake_minimum_required(VERSION 3.8)
project(traversability_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(traversability_estimation_interfaces REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree search segmentation)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)



# Define the traversability_estimation_node executable
add_executable(traversability_estimation_node
  src/traversability_node/main.cpp
  src/traversability_node/node.cpp
  src/traversability_node/params.cpp
  src/traversability_node/filters.cpp
  src/traversability_node/estimation_info.cpp
  src/traversability_node/estimation_cost.cpp
)

# Define the graph_planning_node executable
add_executable(graph_planning_node
  src/graph_planning_node/main.cpp
  src/graph_planning_node/node.cpp
  src/graph_planning_node/graph.cpp
  src/graph_planning_node/path.cpp
  src/graph_planning_node/params.cpp
)

# Define the graph_visualization_node executable
add_executable(graph_visualization_node
  src/graph_visualization_node.cpp
)

# Add include directories
target_include_directories(traversability_estimation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_include_directories(graph_planning_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_include_directories(graph_visualization_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

# Link against PCL libraries
target_link_libraries(traversability_estimation_node ${PCL_LIBRARIES})
target_link_libraries(graph_planning_node ${PCL_LIBRARIES})
target_link_libraries(graph_visualization_node ${PCL_LIBRARIES})


# Add PCL definitions
target_compile_definitions(traversability_estimation_node PUBLIC ${PCL_DEFINITIONS} -DPCL_NO_PRECOMPILE)
target_compile_definitions(graph_planning_node PUBLIC ${PCL_DEFINITIONS} -DPCL_NO_PRECOMPILE)
target_compile_definitions(graph_visualization_node PUBLIC ${PCL_DEFINITIONS} -DPCL_NO_PRECOMPILE)


# Add dependencies for traversability_estimation_node
ament_target_dependencies(traversability_estimation_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2_ros
  tf2_eigen
  geometry_msgs
  grid_map_core
  grid_map_ros
  grid_map_msgs
)

# Add dependencies for graph_planning_node
ament_target_dependencies(graph_planning_node
  traversability_estimation_interfaces
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2_ros
  tf2_eigen
  geometry_msgs
  nav_msgs
  grid_map_core
  grid_map_ros
  grid_map_msgs
  nav2_msgs
)

# Add dependencies for graph_visualization_node
ament_target_dependencies(graph_visualization_node
  traversability_estimation_interfaces
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2_ros
  tf2_eigen
  geometry_msgs
  nav_msgs
  grid_map_core
  grid_map_ros
  grid_map_msgs
)

# Set C++ standard
set_target_properties(traversability_estimation_node PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
set_target_properties(graph_planning_node PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
set_target_properties(graph_visualization_node PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)


# Link OpenMP to both nodes
if(OpenMP_CXX_FOUND)
  target_link_libraries(traversability_estimation_node OpenMP::OpenMP_CXX)
  target_link_libraries(graph_planning_node OpenMP::OpenMP_CXX)
  target_compile_definitions(traversability_estimation_node PUBLIC -DUSE_OPENMP)
  target_compile_definitions(graph_planning_node PUBLIC -DUSE_OPENMP)
endif()

# Install targets
install(TARGETS traversability_estimation_node graph_planning_node graph_visualization_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch and config directories
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Testing setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip specific linter if necessary
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
