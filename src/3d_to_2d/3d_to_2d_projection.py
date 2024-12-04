import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import json
import os

# Set the root directory where annotation folders are stored
input_root = "./output_3d_to_2d/"  # Change to your actual output folder path

# Camera intrinsic matrix (example)
camera_matrix = np.array([
    [1600.6, 0, 951.843],  # fx, 0, cx
    [0, 1600.6, 723.087],  # 0, fy, cy
    [0, 0, 1]              # 0,  0,  1
])

# Function to project 3D points to 2D
def project_to_2d(points_3d, camera_matrix):
    # Use only x, y, z for the projection
    projected_points = camera_matrix @ points_3d.T  # Apply camera intrinsics
    # Normalize by depth (z-coordinate)
    projected_points = (projected_points[:2, :] / projected_points[2, :]).T
    return projected_points

# Function to process each annotation folder
def process_annotation_folders(input_root):
    folders = [os.path.join(input_root, folder) for folder in os.listdir(input_root) if os.path.isdir(os.path.join(input_root, folder))]
    
    for folder in folders:
        print(f"Processing folder: {folder}")
        
        # Load the `.ply` file
        ply_files = [f for f in os.listdir(folder) if f.endswith('.ply')]
        if not ply_files:
            print(f"No `.ply` file found in {folder}")
            continue
        ply_file_path = os.path.join(folder, ply_files[0])
        point_cloud = o3d.io.read_point_cloud(ply_file_path)
        
        # Convert to numpy arrays
        points_3d = np.asarray(point_cloud.points)
        colors = np.asarray(point_cloud.colors)

        # Project to 2D
        points_2d = project_to_2d(points_3d, camera_matrix)

        # Save 2D visualization
        plt.figure(figsize=(8, 6))
        plt.scatter(
            points_2d[:, 0], points_2d[:, 1],
            c=colors, s=1
        )
        plt.gca().invert_yaxis()  # Invert y-axis for image coordinates
        plt.title(f"2D Projection - {os.path.basename(folder)}")
        plt.axis("off")
        
        image_filename = os.path.join(folder, "projected_2d_image.png")
        plt.savefig(image_filename, dpi=300, bbox_inches="tight", pad_inches=0)
        plt.close()
        
        print(f"Saved 2D projection image: {image_filename}")

# Run the processing function
process_annotation_folders(input_root)
