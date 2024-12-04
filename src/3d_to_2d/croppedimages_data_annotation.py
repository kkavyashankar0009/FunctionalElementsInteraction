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
output_root = "./output_3d_to_2d/"
os.makedirs(output_root, exist_ok=True)

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
    fig = plt.figure(figsize=(7, 10))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(
        cropped_points[:, 0],
        cropped_points[:, 1],
        cropped_points[:, 2],
        c=cropped_colors,
        s=1,
        edgecolor="none"
    )
    ax.set_title(f"Annotation {i} - {label}")
    ax.set_axis_off()

    image_filename = os.path.join(output_root, f"image_{annot_id}_{label}/annotation_{annot_id}_{label}.png")
    plt.savefig(image_filename, dpi=600, bbox_inches="tight", pad_inches=0)
    plt.close(fig)

    # Save annotation metadata as JSON
    annotation_json_filename = os.path.join(output_root, f"image_{annot_id}_{label}/annotation.json")
    with open(annotation_json_filename, "w") as annot_json_file:
        json.dump(annotation_data, annot_json_file, indent=4)

    print(f"Saved folder for annotation {annot_id}:")
    print(f"  - Image: {image_filename}")
    print(f"  - Cropped Point Cloud: {cropped_ply_filename}")
    print(f"  - Annotation JSON: {annotation_json_filename}")
