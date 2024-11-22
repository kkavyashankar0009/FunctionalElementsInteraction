import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import os

def project_from_multiple_angles(point_cloud_path, angles, output_dir):
    # Load the point cloud
    point_cloud = o3d.io.read_point_cloud(point_cloud_path)
    
    # Ensure the output directory exists
    os.makedirs(output_dir, exist_ok=True)

    for i, angle in enumerate(angles):
        vis = o3d.visualization.Visualizer()
        vis.create_window(visible=False)  # Suppress the window
        vis.add_geometry(point_cloud)

        # Rotate the view
        ctr = vis.get_view_control()
        param = ctr.convert_to_pinhole_camera_parameters()
        extrinsic = param.extrinsic.copy()  # Create a modifiable copy
        
        # Generate rotation matrix
        rotation_matrix = o3d.geometry.get_rotation_matrix_from_xyz(angle)
        print(f"Angle {i}: {angle} (in radians)")
        print(f"Rotation Matrix:\n{rotation_matrix}")
        
        # Apply rotation to the extrinsic matrix
        extrinsic[:3, :3] = rotation_matrix
        param.extrinsic = extrinsic
        ctr.convert_from_pinhole_camera_parameters(param)

        vis.poll_events()
        vis.update_renderer()
        image = vis.capture_screen_float_buffer(do_render=True)
        vis.destroy_window()

        # Save the image
        output_path = os.path.join(output_dir, f"projection_angle_{i}.png")
        plt.imsave(output_path, np.asarray(image))
        print(f"Saved projection at angle {angle} to {output_path}")

# Example usage
project_from_multiple_angles(
    point_cloud_path=r"C:\Users\kkavy\FunctionalElementsInteraction\data\420673\420673_laser_scan.ply",
    angles=[(0, 0, 0), (np.pi / 4, 0, 0), (0, np.pi / 4, 0), (0, 0, np.pi / 4)],  # Angles in radians
    output_dir=r"C:\Users\kkavy\FunctionalElementsInteraction\src\projections"
)
