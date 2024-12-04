import open3d as o3d
import numpy as np
import json
import matplotlib.pyplot as plt
import os

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
    
# Create main output directory
output_root = "./output_3d_to_2d_normalized/"
os.makedirs(output_root, exist_ok=True)

# Function to normalize points
def normalize_points(points, resolution=(1920, 1080)):
    min_bound = points.min(axis=0)
    max_bound = points.max(axis=0)
    size = max_bound - min_bound

    # Scale points to fit within the resolution
    normalized_points = (points - min_bound) / size  # Scale to [0, 1]
    normalized_points[:, 0] *= resolution[0]
    normalized_points[:, 1] *= resolution[1]

    return normalized_points

# Process each annotation
for i, annotation in enumerate(annotations["annotations"]):
    mask_indices = annotation["indices"]
    label = annotation["label"]
    annot_id = annotation["annot_id"]

    # Extract the annotated points
    annotated_points = np.asarray(pcd.points)[mask_indices]
    annotated_colors = np.asarray(pcd.colors)[mask_indices]

    # Create a new point cloud for the annotation
    annotation_pcd = o3d.geometry.PointCloud()
    annotation_pcd.points = o3d.utility.Vector3dVector(annotated_points)
    annotation_pcd.colors = o3d.utility.Vector3dVector(annotated_colors)

    # Define and expand bounding box
    bbox = annotation_pcd.get_axis_aligned_bounding_box()
    min_bound = bbox.min_bound - 0.7  # Adjust the factor as needed
    max_bound = bbox.max_bound + 0.7
    expanded_bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)

    # Crop the surrounding points within the bounding box
    surrounding_pcd = pcd.crop(expanded_bbox)
    cropped_points = np.asarray(surrounding_pcd.points)
    cropped_colors = np.asarray(surrounding_pcd.colors)

    # Normalize the points for 2D projection
    normalized_points = normalize_points(cropped_points)

    # Save the cropped point cloud as `.ply`
    cropped_ply_filename = f"{output_root}image_{annot_id}_{label}/cropped_pointcloud_{annot_id}.ply"
    os.makedirs(os.path.dirname(cropped_ply_filename), exist_ok=True)
    o3d.io.write_point_cloud(cropped_ply_filename, surrounding_pcd)

    # Prepare annotation metadata
    annotation_data = {
        "image_id": f"{annot_id}_{label}",
        "annotations": [
            {
                "annot_id": annot_id,
                "indices": mask_indices,
                "label": label
            }
        ],
        "cropped_pointcloud_file": cropped_ply_filename
    }

    # Save 2D image using matplotlib
    plt.figure(figsize=(7, 10))
    plt.scatter(
        normalized_points[:, 0],
        normalized_points[:, 1],
        c=cropped_colors,
        s=1,
        edgecolor="none"
    )
    plt.gca().invert_yaxis()
    plt.axis("equal")
    plt.xlim(0, 1920)  # Set limits to match resolution
    plt.ylim(1080, 0)
    plt.title(f"Annotation {i} - {label}")
    plt.axis("off")

    image_filename = os.path.join(output_root, f"image_{annot_id}_{label}/annotation_{annot_id}_{label}.png")
    plt.savefig(image_filename, dpi=600, bbox_inches="tight", pad_inches=0)
    plt.close()

    # Save annotation metadata as JSON
    annotation_json_filename = os.path.join(output_root, f"image_{annot_id}_{label}/annotation.json")
    with open(annotation_json_filename, "w") as annot_json_file:
        json.dump(annotation_data, annot_json_file, indent=4)

    print(f"Saved folder for annotation {annot_id}:")
    print(f"  - Image: {image_filename}")
    print(f"  - Cropped Point Cloud: {cropped_ply_filename}")
    print(f"  - Annotation JSON: {annotation_json_filename}")
