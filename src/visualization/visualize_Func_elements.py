import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def project_point_cloud(point_cloud, indices, save_path="output.png"):
    # Visualize the full point cloud
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False)
    vis.add_geometry(point_cloud)

    # Highlight specific points corresponding to indices
    pcd_points = np.asarray(point_cloud.points)  # Get all points
    highlighted_points = pcd_points[indices]  # Extract points at given indices
    highlighted_pcd = o3d.geometry.PointCloud()
    highlighted_pcd.points = o3d.utility.Vector3dVector(highlighted_points)
    highlighted_pcd.paint_uniform_color([1, 0, 0])  # Red color for highlighted points

    # Add the highlighted points to the visualization
    vis.add_geometry(highlighted_pcd)
    vis.poll_events()
    vis.update_renderer()

    # Capture the screen
    image = vis.capture_screen_float_buffer()
    vis.destroy_window()

    # Save the 2D projection as an image
    plt.imsave(save_path, np.asarray(image))
    print(f"Saved projection to {save_path}")

# Load a point cloud from a .ply file
point_cloud = o3d.io.read_point_cloud(r"C:\Users\kkavy\FunctionalElementsInteraction\data\420673\420673_laser_scan.ply")
print(point_cloud)
print("Number of points:", len(point_cloud.points))

# Visualize the point cloud
o3d.visualization.draw_geometries([point_cloud])
# Example indices
indices = [
    842076, 843747, 846339, 847444, 847963, 848665, 848667, 848674, 848675,
    848794, 849896, 851364, 944830, 962076, 992260, 993505, 993664, 1053668,
    1053671, 1084694, 1381242, 1620316, 2446230, 2894298, 2993619, 3142086,
    3363037, 3375058, 3437645, 3447358, 3475473, 3481796, 3607531, 3620617,
    3637723, 3718645, 3772113, 3821344, 3822509, 3825687, 3837854, 3904429,
    3905269, 3905291, 3905316, 3905324, 3906318, 3906374, 3906385, 3906399,
    3906406, 3906429, 3911544, 3911894, 3912274, 3913175, 3913180, 3960328,
    4001651, 4068365, 4087318, 4179136, 4203560, 4230872, 4231372, 4272691,
    4378778, 4432132, 4440790, 4445055, 5333249
]

# Project the point cloud and save the highlighted 2D projection
project_point_cloud(point_cloud, indices)
