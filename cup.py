## IMPORT LIBRARIES
import numpy as np
import copy
from sklearn.decomposition import PCA
import os
import open3d as o3d
from utils import *
import matplotlib.pyplot as plt
import pandas as pd
from open3d.visualization.draw_plotly import get_plotly_fig
import plotly.graph_objects as go
print("Open3D version:", o3d.__version__)
import torch

# Check if CUDA is available
if torch.cuda.is_available():
    # Set the device to GPU
    device = torch.device("cuda")
    print("Using GPU for computation.")
    print("Device type:", torch.cuda.get_device_name(device))
    print("Number of GPUs:", torch.cuda.device_count())
else:
    # Set the device to CPU
    device = torch.device("cpu")
    print("GPU not available. Using CPU for computation.")





if __name__ == '__main__':

    # Get the current directory
    current_directory = os.getcwd()

    # Go back to the parent directory
    parent_directory = os.path.dirname(current_directory)

    # Set input directory
    point_cloud_ply = os.path.join(parent_directory, 'Data', 'temple_overlapping.ply')
    output_folder = os.path.join(parent_directory, 'Data', 'Output')

    # Get pcd file
    point_cloud = open3d.io.read_point_cloud(point_cloud_ply)
    visualization_draw_geometry(point_cloud, background='white')

    ### ----- Downsampling
    # Voxel Grid
    print("Number of points BEFORE: ", len(point_cloud.points))
    point_cloud_downsampled = point_cloud.voxel_down_sample(voxel_size=0.005)
    print("Number of points AFTER: ", len(point_cloud_downsampled.points))
    o3d.visualization.draw_geometries([point_cloud_downsampled])


    ###------------------------ 1. Normals ----------------------- ###

    point_cloud_downsampled.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=100, max_nn=30))
    o3d.visualization.draw_geometries([point_cloud.paint_uniform_color((0.8,0.8,0.8)), point_cloud_downsampled], point_show_normal=True)

    ##------------------------ 2. Descriptors ----------------------- ###

    # radius_feature = 0.1
    # fpfh = o3d.pipelines.registration.compute_fpfh_feature(point_cloud_downsampled, o3d.geometry.KDTreeSearchParamRadius(radius_feature))
    #
    # variances = np.var(fpfh.data, axis=1)
    # top_dims = np.argpartition(variances, -3)[-3:]
    # fpfh  = fpfh.data.T[top_dims]
    #
    # # Normalize to [0,1]
    # fpfh_normalized = (fpfh.data - np.min(fpfh.data)) / (np.max(fpfh.data) - np.min(fpfh.data))
    #
    # # Assign colors
    # point_cloud.colors = o3d.utility.Vector3dVector(fpfh_normalized)
    # # o3d.visualization.draw_geometries([point_cloud])

    ###------------------------ 3. Keypoints ----------------------- ###

    # # Method 1: Random Selection
    # point_cloud.paint_uniform_color((0.5, 0.5, 0.5))
    # keypoint_indices = np.random.choice(len(point_cloud.points), size=100, replace=False)
    # o3d.visualization.draw_geometries([point_cloud, point_cloud.select_by_index(keypoint_indices).paint_uniform_color((1,0,0))])

    # Method 2: Intrinsic Shape Signature
    keypoints = o3d.geometry.keypoint.compute_iss_keypoints(point_cloud_downsampled)
    point_cloud_downsampled.paint_uniform_color((0.5, 0.5, 0.5))
    # o3d.visualization.draw_geometries([point_cloud_downsampled, keypoints])

    # Method 3: Fit a line
    pca = PCA(n_components=3)
    pca.fit(np.asarray(keypoints.points))

    # Line is along largest eigenvalue
    line_vector = pca.components_[0]

    # Compute two points: mean and x-distance
    center = pca.mean_
    point1 = center + line_vector
    point2 = center - line_vector

    # Create line
    lines = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector([point1, point2]),
        lines=o3d.utility.Vector2iVector([[0,1]])
    )

    # o3d.visualization.draw_geometries([point_cloud_downsampled, keypoints, lines])

    ###------------------------ 4. Reconstruction ----------------------- ###

    # Method 1: Alpha
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(point_cloud_downsampled, alpha=0.037)
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([point_cloud_downsampled, mesh], mesh_show_back_face=True)


    # Method 2: Ball Pivoting
    # Estimate normals for the downsampled point cloud
    point_cloud_downsampled.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    # Define radii for ball pivoting
    radii = [0.08, 0.16, 0.24, 0.4]
    # Perform ball pivoting surface reconstruction
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(point_cloud_downsampled, o3d.utility.DoubleVector(radii))
    # Compute vertex normals for the reconstructed mesh
    mesh.compute_vertex_normals()
    # Visualize the downsampled point cloud and reconstructed mesh
    # o3d.visualization.draw_geometries([point_cloud_downsampled, mesh], mesh_show_back_face=True)

    # Method 3: Poisson
    mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(point_cloud_downsampled, scale=0.4, linear_fit=True)
    mesh.filter_smooth_laplacian(2)
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([point_cloud_downsampled, mesh], mesh_show_back_face=True)



