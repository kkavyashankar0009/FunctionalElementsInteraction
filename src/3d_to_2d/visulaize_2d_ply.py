import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import os

# Set the root directory where cropped `.ply` files are stored
input_root = "./output_3d_to_2d/"

# Function to project 3D points to 2D (x, y only)
def project_to_2d(points_3d):
    return points_3d[:, :2]  # Take only x and y components

# Process each cropped `.ply` file
def process_cropped_ply_files(input_root):
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
        points_2d = project_to_2d(points_3d)

        # Save 2D visualization
        plt.figure(figsize=(12, 8))
        plt.scatter(
            points_2d[:, 0], points_2d[:, 1],
            c=colors, s=1
        )
        plt.gca().invert_yaxis()
        plt.axis('equal')
        plt.xlim(points_2d[:, 0].min() - 50, points_2d[:, 0].max() + 50)
        plt.ylim(points_2d[:, 1].max() + 50, points_2d[:, 1].min() - 50)
        plt.title(f"2D Projection - {os.path.basename(folder)}")
        plt.axis("off")

        # Save the visualization as an image
        image_filename = os.path.join(folder, "projected_2d_image_full.png")
        plt.savefig(image_filename, dpi=300, bbox_inches="tight", pad_inches=0)
        plt.close()
        
        print(f"Saved 2D projection image: {image_filename}")

# Run the function
process_cropped_ply_files(input_root)
