import open3d as o3d
import numpy as np
import cv2
import json
import os

# Load the point cloud using Open3D
point_cloud_file = r"C:\Users\kkavy\FunctionalElementsInteraction\data\420673\420673_laser_scan.ply"
pcd = o3d.io.read_point_cloud(point_cloud_file)

# Extract points and colors
points = np.asarray(pcd.points)
colors = np.asarray(pcd.colors)

if len(points) == 0:
    raise ValueError("Point cloud is empty. Check the input file.")

print(f"Loaded {len(points)} points.")

# Load annotations
annotation_file = r"C:\Users\kkavy\FunctionalElementsInteraction\data\420673\420673_annotations.json"
with open(annotation_file, "r") as f:
    annotations = json.load(f)

# Define camera intrinsics (from provided camera matrix)
camera_matrix = np.array([
    [1600.6, 0, 951.843],  # fx, 0, cx
    [0, 1600.6, 723.087],  # 0, fy, cy
    [0, 0, 1]              # 0,  0,  1
])

# Define camera extrinsics (rotation and translation)
rotation_vector = np.array([0, 0, 0], dtype=np.float32)  # No rotation
translation_vector = np.array([0, 0, -5], dtype=np.float32)  # Camera positioned at -5 along Z-axis

# Define image size
img_size = (1446, 1904)  # Approx. size based on principal point cx, cy
output_folder = "./output_images/"
os.makedirs(output_folder, exist_ok=True)

# Process each annotation
for i, annotation in enumerate(annotations["annotations"]):
    mask_indices = annotation["indices"]
    label = annotation["label"]
    annot_id = annotation["annot_id"]

    # Extract annotated points
    annotated_points = points[mask_indices]

    # Define the bounding box for surroundings
    min_bound = annotated_points.min(axis=0) - 1.0  # Expand by 1.0 units
    max_bound = annotated_points.max(axis=0) + 1.0

    # Include surrounding points within the bounding box
    mask_surroundings = np.all((points >= min_bound) & (points <= max_bound), axis=1)
    surrounding_points = points[mask_surroundings]
    surrounding_colors = colors[mask_surroundings]

    # Project surrounding points to 2D using OpenCV
    projected_points, _ = cv2.projectPoints(
        surrounding_points, rotation_vector, translation_vector, camera_matrix, None
    )

    # Create a blank image
    img = np.ones((*img_size, 3), dtype=np.uint8) * 255  # White background

    # Render points
    for pt, color in zip(projected_points, surrounding_colors):
        x, y = int(pt[0][0]), int(pt[0][1])
        if 0 <= x < img_size[1] and 0 <= y < img_size[0]:  # Ensure within bounds
            color_bgr = (int(color[2] * 255), int(color[1] * 255), int(color[0] * 255))  # RGB to BGR
            cv2.circle(img, (x, y), 200, color_bgr, -1)  # Render point as a small circle

    # Save the image
    image_filename = f"{output_folder}annotation_{annot_id}_{label}.png"
    cv2.imwrite(image_filename, img)
    print(f"Saved annotation image: {image_filename}")
