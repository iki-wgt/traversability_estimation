#!/usr/bin/env python3
import open3d as o3d
import os

def generate_cloud(path, voxel_size=0.5):

    if not os.path.exists(path):
        print(f"Error: {path} does not exist.")
        return
    
    output_path = path.split(".")[0]
    
    #check if file already exist
    if os.path.exists(f"{output_path}.pcd"):
        return

    mesh = o3d.io.read_triangle_mesh(path)
    cloud = mesh.sample_points_uniformly(number_of_points=100_000_000)
    cloud = cloud.voxel_down_sample(voxel_size=voxel_size)

    # Save files
    o3d.io.write_point_cloud(f"{output_path}.pcd", cloud)



if __name__ == "__main__":
    # Path to your DAE or STL file
    path = "/world/world.stl"
    generate_cloud(path, voxel_size=0.1)

