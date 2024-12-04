import open3d as o3d
import numpy as np
import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Load point cloud
point_cloud_file = r"C:\Users\kkavy\FunctionalElementsInteraction\data\420673\420673_laser_scan.ply"
pcd = o3d.io.read_point_cloud(point_cloud_file)

# Load annotations
annotation_file = r"C:\Users\kkavy\FunctionalElementsInteraction\data\420673\420673_annotations.json"
with open(annotation_file, "r") as f:
    annotations = json.load(f)

# Assign default grey color if no colors exist
colors = np.asarray(pcd.colors)
if len(colors) == 0:
    print("Point cloud has no color. Assigning default grey color.")
    pcd.colors = o3d.utility.Vector3dVector(np.full((len(pcd.points), 3), 0.7))

# Function to add bounding box to the plot
def plot_bounding_box(ax, bbox_min, bbox_max, color="red"):
    """Plots a 3D bounding box given min and max bounds."""
    # Define the 8 corners of the bounding box
    corners = np.array([
        [bbox_min[0], bbox_min[1], bbox_min[2]],
        [bbox_max[0], bbox_min[1], bbox_min[2]],
        [bbox_max[0], bbox_max[1], bbox_min[2]],
        [bbox_min[0], bbox_max[1], bbox_min[2]],
        [bbox_min[0], bbox_min[1], bbox_max[2]],
        [bbox_max[0], bbox_min[1], bbox_max[2]],
        [bbox_max[0], bbox_max[1], bbox_max[2]],
        [bbox_min[0], bbox_max[1], bbox_max[2]],
    ])

    # Define the 12 edges of the bounding box
    edges = [
        [corners[0], corners[1]], [corners[1], corners[2]], [corners[2], corners[3]], [corners[3], corners[0]],  # Bottom face
        [corners[4], corners[5]], [corners[5], corners[6]], [corners[6], corners[7]], [corners[7], corners[4]],  # Top face
        [corners[0], corners[4]], [corners[1], corners[5]], [corners[2], corners[6]], [corners[3], corners[7]]   # Vertical edges
    ]

    # Plot each edge
    for edge in edges:
        ax.plot3D(*zip(*edge), color=color, linewidth=1.5)

# Loop through annotations to isolate regions, apply colors, and render images
for i, annotation in enumerate(annotations["annotations"]):
    mask_indices = annotation["indices"]
    label = annotation["label"]
    annot_id = annotation["annot_id"]

    # Extract the annotated points
    annotated_points = np.asarray(pcd.points)[mask_indices]
    annotated_colors = np.asarray(pcd.colors)[mask_indices]

    # Assign a unique color to the annotation points
    annotation_color = np.random.rand(3)  # Generate a random color
    annotated_colors[:] = annotation_color  # Apply the same color to all annotated points

    # Create a new point cloud for the annotation and its surroundings
    annotation_pcd = o3d.geometry.PointCloud()
    annotation_pcd.points = o3d.utility.Vector3dVector(annotated_points)
    annotation_pcd.colors = o3d.utility.Vector3dVector(annotated_colors)

    # Define a bounding box for surrounding points
    bbox = annotation_pcd.get_axis_aligned_bounding_box()
    min_bound = bbox.min_bound - 0.7  # Adjust the factor as needed
    max_bound = bbox.max_bound + 0.7
    expanded_bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)

    # Crop the surrounding points within the bounding box
    surrounding_pcd = pcd.crop(expanded_bbox)
    surrounding_points = np.asarray(surrounding_pcd.points)
    surrounding_colors = np.asarray(surrounding_pcd.colors)

    # Combine annotation and surrounding points
    combined_points = np.vstack((surrounding_points, annotated_points))
    combined_colors = np.vstack((surrounding_colors, annotated_colors))

    # Plot and save as an image using matplotlib
    fig = plt.figure(figsize=(7, 10))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(
        combined_points[:, 0],
        combined_points[:, 1],
        combined_points[:, 2],
        c=combined_colors,
        s=1,
        edgecolor="none"
    )

    # Add bounding box for annotation
    plot_bounding_box(ax, bbox.min_bound, bbox.max_bound, color="red")

    ax.set_title(f"Annotation {i} - {label}")
    ax.set_axis_off()


    # Save the image
    image_filename = f"outputs_FEs_annotation/bb_annotation_{annot_id}_{label}.png"
    plt.savefig(image_filename, dpi=600, bbox_inches="tight", pad_inches=0)
    plt.close(fig)

    print(f"Saved annotation image with highlighted colors: {image_filename}")
