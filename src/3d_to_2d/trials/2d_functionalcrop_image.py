import open3d as o3d
import numpy as np
import json

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
    #annotation_pcd.colors = o3d.utility.Vector3dVector(annotated_colors)

    # Optional: Crop surrounding points by defining a bounding box
    bbox = annotation_pcd.get_axis_aligned_bounding_box()

    # Manually expand the bounding box
    min_bound = bbox.min_bound - 0.5  # Adjust the factor as needed
    max_bound = bbox.max_bound + 0.5
    expanded_bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)

    # Crop the point cloud using the expanded bounding box
    surrounding_pcd = pcd.crop(expanded_bbox)
    surrounding_pcd = pcd.crop(bbox)
    print(f"Number of points in cropped point cloud: {len(surrounding_pcd.points)}")
    if len(surrounding_pcd.points) == 0:
        print("No points in the cropped region! Check the bounding box dimensions.")
    #o3d.visualization.draw_geometries([surrounding_pcd], window_name="Debug Cropped Region")

    # Visualize the cropped point cloud and save it as an image
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=True)  # Invisible window for off-screen rendering
    vis.add_geometry(surrounding_pcd)
    
    # Set background color to white
    opt = vis.get_render_option()
    opt.background_color = np.array([1, 1, 1])  # White background
    opt.point_size = 5.0  # Increase point size for better visibility
    opt.show_coordinate_frame = True  # Hide the coordinate frame

    # Camera settings
    ctr = vis.get_view_control()
    ctr.set_zoom(0.8)  # Adjust zoom level
    ctr.rotate(0.0, 0.0)  # Reset camera rotation
    # Save the image
    image_filename = f"annotation_{id}_{label}.png"
    vis.poll_events()  # Necessary for off-screen rendering
    vis.update_renderer()
    vis.capture_screen_image(image_filename)
    vis.destroy_window()

    print(f"Saved annotation image: {image_filename}")
