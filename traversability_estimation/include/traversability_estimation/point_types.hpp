#ifndef TRAVERSABILITY_ESTIMATION__POINT_TYPES_HPP_
#define TRAVERSABILITY_ESTIMATION__POINT_TYPES_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct PointXYZICluster
{
    PCL_ADD_POINT4D;  // Adds x, y, z, data[4]
    PCL_ADD_NORMAL4D; // Adds normal_x, normal_y, normal_z, data_n[4]
    float curvature;
    float intensity;
    uint32_t cluster_id;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZICluster, //
                                  (float, x, x)     //
                                  (float, y, y)     //
                                  (float, z, z)     //
                                  // Padding from PCL_ADD_POINT4D
                                  (float, data[4], data)             //
                                  (float, normal_x, normal_x)        //
                                  (float, normal_y, normal_y)        //
                                  (float, normal_z, normal_z)        //
                                  (float, curvature, curvature)      //
                                  (float, intensity, intensity)      //
                                  (uint32_t, cluster_id, cluster_id) //
)

struct TraversablePoint
{
    PCL_ADD_POINT4D;  // Adds x, y, z, data[4]
    PCL_ADD_NORMAL4D; // Adds normal_x, normal_y, normal_z, data_n[4]
    float slope;
    float curvature;
    float intensity;
    uint32_t cluster_id;
    float inflation_cost;
    float slope_cost;
    float curvature_cost;
    float final_cost;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(TraversablePoint, //
                                  (float, x, x)     //
                                  (float, y, y)     //
                                  (float, z, z)     //
                                  // Padding from PCL_ADD_POINT4D
                                  (float, data[4], data)        //
                                  (float, normal_x, normal_x)   //
                                  (float, normal_y, normal_y)   //
                                  (float, normal_z, normal_z)   //
                                  (float, slope, slope)         //
                                  (float, curvature, curvature) //
                                  // Padding from PCL_ADD_NORMAL4D
                                  (float, data_n[4], data_n)              //
                                  (float, intensity, intensity)           //
                                  (uint32_t, cluster_id, cluster_id)      //
                                  (float, inflation_cost, inflation_cost) //
                                  (float, slope_cost, slope_cost)         //
                                  (float, curvature_cost, curvature_cost) //
                                  (float, final_cost, final_cost)         //
)

#endif // TRAVERSABILITY_ESTIMATION__POINT_TYPES_HPP_
