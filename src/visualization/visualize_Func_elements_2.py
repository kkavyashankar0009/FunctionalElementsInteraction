import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import json


point_cloud_file  = r"C:\Users\kkavy\FunctionalElementsInteraction\data\420673\420673_laser_scan.ply"



pcd = o3d.io.read_point_cloud(point_cloud_file)

# Load annotations
annotation_file = r"C:\Users\kkavy\FunctionalElementsInteraction\data\420673\420673_annotations.json"  # Replace with your annotation file
with open(annotation_file, "r") as f:
    annotations = json.load(f)

# Extract mask indices and labels
functional_elements = []
for annotation in annotations["annotations"]:
    mask_indices = annotation["indices"]
    label = annotation["label"]
    functional_elements.append((mask_indices, label))

# Visualize the point cloud with highlighted elements
colors = np.asarray(pcd.colors)  # Original colors
# if len(colors) == 0:
#     colors = np.zeros((len(pcd.points), 3))  # If no color, initialize to black

fade_factor = 0.8  # Adjust this to control the fade intensity (closer to 1 for more white)
colors = colors * (1 - fade_factor) + np.ones_like(colors) * fade_factor

highlighted_colors = colors.copy()
for mask_indices, label in functional_elements:
    # Apply random color for visualization
    color = np.random.rand(3)
    highlighted_colors[mask_indices] = color

# Assign the new colors back to the point cloud
pcd.colors = o3d.utility.Vector3dVector(highlighted_colors)

# Visualize
o3d.visualization.draw_geometries([pcd], window_name="Functional Interactive Elements")