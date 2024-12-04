import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import json
import os

# Load point cloud
point_cloud_file = r"C:\Users\kkavy\FunctionalElementsInteraction\data\420673\420673_laser_scan.ply"
pcd = o3d.io.read_point_cloud(point_cloud_file)

# Load annotations
annotation_file = r"C:\Users\kkavy\FunctionalElementsInteraction\data\420673\420673_annotations.json"
with open(annotation_file, "r") as f:
    annotations = json.load(f)

# Extract 3D points and colors
points_3d = np.asarray(pcd.points)
colors = np.asarray(pcd.colors)

# Camera intrinsic data
camera_intrinsic = np.array([
    [1600.6, 0, 951.843],  # fx, 0, cx
    [0, 1600.6, 723.087],  # 0, fy, cy
    [0, 0, 1]              # 0,  0,  1
])

# Process each annotation
output_root = "./output_3d_to_2d_trans/"
os.makedirs(output_root, exist_ok=True)

for i, annotation in enumerate(annotations["annotations"]):
    mask_indices = annotation["indices"]
    label = annotation["label"]
    annot_id = annotation["annot_id"]

    # Extract ROI points and compute the center of the ROI
    roi_points = points_3d[mask_indices]
    roi_colors = colors[mask_indices]
    roi_center = roi_points.mean(axis=0)

    # Place the camera near the ROI (extrinsic parameters)
    # Camera looks towards the ROI from a position slightly offset along the Z-axis
    camera_position = roi_center + np.array([0, 0, -2])  # Move camera 3 units closer
    translation_vector = -camera_position  # Invert to place the camera
    rotation_matrix = np.eye(3)  # Assume no rotation for simplicity

    # Project the entire scene to 2D
    points_3d_h = np.hstack((points_3d, np.ones((points_3d.shape[0], 1))))  # Homogeneous coordinates
    extrinsic_matrix = np.hstack((rotation_matrix, translation_vector.reshape(-1, 1)))
    camera_frame_points = extrinsic_matrix @ points_3d_h.T

    # Apply camera intrinsic matrix
    projected_points = camera_intrinsic @ camera_frame_points[:3, :]  # Apply intrinsic matrix
    projected_points = (projected_points[:2, :] / projected_points[2, :]).T  # Normalize by depth

    # Save 2D visualization
    plt.figure(figsize=(12, 8))
    plt.scatter(
        projected_points[:, 0], projected_points[:, 1],
        c=colors, s=1
    )
    plt.gca().invert_yaxis()
    plt.axis('equal')
    plt.title(f"2D Projection - Annotation {annot_id} ({label})")
    plt.axis("off")

    # Save the image
    output_folder = os.path.join(output_root, f"image_{annot_id}_{label}")
    os.makedirs(output_folder, exist_ok=True)
    image_filename = os.path.join(output_folder, f"projection_{annot_id}.png")
    plt.savefig(image_filename, dpi=300, bbox_inches="tight", pad_inches=0)
    plt.close()

    print(f"Saved 2D projection for annotation {annot_id}: {image_filename}")
