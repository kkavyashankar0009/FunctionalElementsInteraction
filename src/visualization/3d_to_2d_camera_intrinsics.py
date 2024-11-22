import zipfile
import os
import numpy as np
import open3d as o3d
import pandas as pd
import matplotlib.pyplot as plt

# Path to the zip file

def extract_camera_intrinsics(pincam_file_path):
    # Extract camera intrinsics
    # Check if the file exists
    if not os.path.isfile(pincam_file_path):
        raise FileNotFoundError(f"The file {pincam_file_path} does not exist.")

    # Parse the file content
    with open(pincam_file_path, 'r') as file:
        content = file.read().strip()  # Remove extra whitespace or newline characters

    # Split the content into components
    width, height, fx, fy, cx, cy = map(float, content.split())

    # Construct the intrinsic matrix
    intrinsic_matrix = [
        [fx,  0, cx],
        [ 0, fy, cy],
        [ 0,  0,  1]
    ]

    # Print the intrinsic matrix
    print("Intrinsic Matrix:")
    for row in intrinsic_matrix:
        print(row)

    # Optionally, print the image dimensions
    print(f"Image Width: {width}")
    print(f"Image Height: {height}")


    return (width, height, fx, fy, cx, cy)

def project_point_cloud_to_2d(point_cloud, intrinsic_matrix, extrinsic_matrix=None):
    # Convert point cloud to numpy array
    points = np.asarray(point_cloud.points)

    # Apply extrinsics if provided (world-to-camera transformation)
    if extrinsic_matrix is not None:
        points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))  # Add homogeneous coordinate
        points = (extrinsic_matrix @ points_homogeneous.T).T[:, :3]

    # Apply intrinsics to project points to 2D
    fx, fy, cx, cy = intrinsic_matrix[0, 0], intrinsic_matrix[1, 1], intrinsic_matrix[0, 2], intrinsic_matrix[1, 2]
    points_2d = points[:, :2] / points[:, 2:3]  # Normalize by depth (Z)
    points_2d[:, 0] = fx * points_2d[:, 0] + cx  # Scale by focal length and add principal point (X)
    points_2d[:, 1] = fy * points_2d[:, 1] + cy  # Scale by focal length and add principal point (Y)

    return points_2d


def visualize_2d_with_open3d(points_2d):
    """
    Visualize 2D points using Open3D.
    
    :param points_2d: NumPy array of 2D points with shape (N, 2)
    """
    # Convert 2D points to 3D by adding a zero Z-axis
    points_3d = np.hstack((points_2d, np.zeros((points_2d.shape[0], 1))))

    # Create an Open3D point cloud object
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points_3d)

    # Set uniform color for the points
    point_cloud.paint_uniform_color([0, 0, 1])  # Blue color

    # Visualize using Open3D
    o3d.visualization.draw_geometries([point_cloud], window_name="2D Points Visualization")

# Example usage
if __name__ == "__main__":

    
    # Extract the zip file
    zip_path = r"C:\Users\kkavy\FunctionalElementsInteraction\data\420673\42445198\hires_wide_intrinsics.zip"
    extract_dir = r"C:\Users\kkavy\FunctionalElementsInteraction\data\420673\42445198"  # Directory where files will be extracted
    with zipfile.ZipFile(zip_path, 'r') as zip_ref:
        zip_ref.extractall(extract_dir)

    print(f"Extracted files to {extract_dir}")

    # Locate the .pincam file
    hires_intrinsics_dir = os.path.join(extract_dir, "hires_wide_intrinsics")
    pincam_file_name = "42445198_10941.631.pincam"  # Replace with the correct file name
    pincam_file_path = os.path.join(hires_intrinsics_dir, pincam_file_name)

    width, height, fx, fy, cx, cy=extract_camera_intrinsics(pincam_file_path)

    # Load the point cloud
    point_cloud = o3d.io.read_point_cloud(r"C:\Users\kkavy\FunctionalElementsInteraction\data\420673\420673_laser_scan.ply")

    # Example camera intrinsic matrix (focal lengths and principal point)
    intrinsic_matrix = np.array([
        [fx, 0.0, cx],  # fx, 0, cx
        [0.0, fy, cy],  # 0, fy, cy
        [0.0, 0.0, 1.0]       # 0,  0,  1
    ])

    print(intrinsic_matrix)
    # Project to 2D
    projected_points = project_point_cloud_to_2d(point_cloud, intrinsic_matrix)

    # Save the 2D points to a file or visualize
    print("Projected 2D points:", projected_points[:10])  # Print first 10 points

    df = pd.DataFrame(projected_points, columns=['u (X-coordinate)', 'v (Y-coordinate)'])
    df.to_csv("projected_points_42445198_10941.631.csv", index=False)
    print("2D projected points saved to 'projected_points.csv'")
    loaded_points = pd.read_csv("projected_points_42445198_10941.631.csv").to_numpy()

# Plot the 2D points
    plt.scatter(loaded_points[:, 0], loaded_points[:, 1], c='blue', label='Projected Points')
    plt.title('2D Projection of 3D Points')
    plt.xlabel('u (X-coordinate)')
    plt.ylabel('v (Y-coordinate)')
    plt.grid(True)
    plt.legend()
    plt.show()
    plt.savefig("projected_points_visualization_42445198_10941.631.png")
    print("2D visualization saved as 'projected_points_visualization.png'")

    visualize_2d_with_open3d(projected_points)
    