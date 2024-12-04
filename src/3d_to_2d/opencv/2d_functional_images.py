import cv2
import numpy as np
import json

# Load point cloud (PLY file)
point_cloud_file = r"C:\Users\kkavy\FunctionalElementsInteraction\data\420673\420673_laser_scan.ply"

# Load the point cloud data
def load_point_cloud(file):
    points = []
    colors = []
    with open(file, "r",encoding="utf-8", errors="replace") as f:
        for line in f.readlines():
            if line.startswith("end_header"):
                break
        for line in f.readlines():
            x, y, z, r, g, b = map(float, line.split())
            points.append([x, y, z])
            colors.append([r / 255.0, g / 255.0, b / 255.0])  # Normalize colors
    return np.array(points), np.array(colors)

points, colors = load_point_cloud(point_cloud_file)
print(f"Number of points loaded: {len(points)}")
print(f"Number of colors loaded: {len(colors)}")

# Load annotations
annotation_file = r"C:\Users\kkavy\FunctionalElementsInteraction\data\420673\420673_annotations.json"
with open(annotation_file, "r") as f:
    annotations = json.load(f)

# Loop through annotations
for i, annotation in enumerate(annotations["annotations"]):
    mask_indices = annotation["indices"]
    label = annotation["label"]
    annot_id = annotation["annot_id"]

    # Extract the annotated points
    annotated_points = points[mask_indices]
    annotated_colors = colors[mask_indices]

    # Create a bounding box for the annotated region
    min_bound = annotated_points.min(axis=0) - 0.7
    max_bound = annotated_points.max(axis=0) + 0.7

    # Find all points within the bounding box
    mask = np.all((points >= min_bound) & (points <= max_bound), axis=1)
    cropped_points = points[mask]
    cropped_colors = colors[mask]

    # Project points to 2D (orthographic projection)
    img_size = (1000, 1000)  # Resolution of the output image
    img = np.ones((*img_size, 3), dtype=np.uint8) * 255  # White background

    # Normalize points to fit the image
    x = cropped_points[:, 0]
    y = cropped_points[:, 1]
    z = cropped_points[:, 2]  # Depth information
    x_norm = ((x - x.min()) / (x.max() - x.min()) * (img_size[1] - 1)).astype(int)
    y_norm = ((y - y.min()) / (y.max() - y.min()) * (img_size[0] - 1)).astype(int)

    # Render points to the image
    for px, py, color in zip(x_norm, y_norm, cropped_colors):
        color_bgr = (color[2] * 255, color[1] * 255, color[0] * 255)  # Convert RGB to BGR
        cv2.circle(img, (px, py), 1, color_bgr, -1)

    # Save the image
    image_filename = f"opencv/annotation_{annot_id}_{label}.png"
    cv2.imwrite(image_filename, img)

    print(f"Saved annotation image: {image_filename}")
