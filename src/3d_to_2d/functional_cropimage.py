import open3d as o3d
import numpy as np
import json
import matplotlib.pyplot as plt

# Load point cloud
point_cloud_file  = r"C:\Users\kkavy\FunctionalElementsInteraction\data\420673\420673_laser_scan.ply"
pcd = o3d.io.read_point_cloud(point_cloud_file)

# Load annotations
annotation_file = r"C:\Users\kkavy\FunctionalElementsInteraction\data\420673\420673_annotations.json"  # Replace with your annotation file
with open(annotation_file, "r") as f:
    annotations = json.load(f)

colors = np.asarray(pcd.colors)
if len(colors) == 0:
    print("Point cloud has no color. Assigning default grey color.")
    pcd.colors = o3d.utility.Vector3dVector(np.full((len(pcd.points), 3), 0.7))

# Loop through annotations to isolate regions and render images
for i, annotation in enumerate(annotations["annotations"]):
    mask_indices = annotation["indices"]
    label = annotation["label"]
    id=annotation["annot_id"]

    # Extract the annotated points
    annotated_points = np.asarray(pcd.points)[mask_indices]
    annotated_colors = np.asarray(pcd.colors)[mask_indices]

    # Create a new point cloud for the annotation and its surroundings
    annotation_pcd = o3d.geometry.PointCloud()
    annotation_pcd.points = o3d.utility.Vector3dVector(annotated_points)
    annotation_pcd.colors = o3d.utility.Vector3dVector(annotated_colors)

    # Optional: Crop surrounding points by defining a bounding box
    bbox = annotation_pcd.get_axis_aligned_bounding_box()

    # Manually expand the bounding box
    min_bound = bbox.min_bound - 0.7  # Adjust the factor as needed
    max_bound = bbox.max_bound + 0.7
    expanded_bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)

    # Crop the point cloud using the expanded bounding box
    surrounding_pcd = pcd.crop(expanded_bbox)

    # Convert cropped point cloud to numpy arrays
    cropped_points = np.asarray(surrounding_pcd.points)
    cropped_colors = np.asarray(surrounding_pcd.colors)

    # Plot and save as an image using matplotlib
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

    # Save the image
    image_filename = f"annotation_{id}_{label}.png"
    plt.savefig(image_filename, dpi=600, bbox_inches="tight", pad_inches=0)
    plt.close(fig)


    print(f"Saved annotation image: {image_filename}")
