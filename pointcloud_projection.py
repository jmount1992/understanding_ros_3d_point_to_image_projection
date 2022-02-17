import open3d as o3d
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
import yaml
import pickle

import numpy as np
from skimage import draw
import matplotlib.pyplot as plt
import spatialmath as sm

import os
import copy
import time


def init_camera_model(camera_model_path : str) -> PinholeCameraModel:
    
    with open(camera_model_path, 'rb') as f:
        camera_info = yaml.load(f, Loader=yaml.FullLoader)

        # Construct camera model from YAML
        ci = CameraInfo()
        ci.header.stamp = 0
        ci.header.frame_id = camera_info["header"]['frame_id']
        ci.width = camera_info['width']
        ci.height = camera_info['height']
        ci.distortion_model = camera_info['distortion_model']
        ci.D = camera_info['D']
        ci.K = camera_info['K']
        ci.P = camera_info['P']
        ci.R = camera_info['R']
        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(ci)

    return camera_model


def project_to_image(pcd, radius: int = 20):
    image = np.ones((cam_model.height, cam_model.width, 3), dtype='uint8') * 255
    for idx, (point, colour) in enumerate(zip(pcd.points, pcd.colors)):

        x, y, z = point
        if z < 0:
            continue # ignore points behind image plane

        (u,v) = cam_model.project3dToPixel((x, y, z))

        if point[0] == 0 and point[1] == 0 and point[2] == 2:
            print("u,v:",u,v, colour)

        # print("IDX: %02d, (x,y): %f, %f, (u,v): %0.2f, %0.2f -- "%(idx, x, y, u, v), end='')

        # Paint pixel
        if (u >= 0 and v >= 0) and (u < cam_model.width and v < cam_model.height):
            # print("Valid -- index: %02d, (x,y): %f, %f, (u,v): %0.2f, %0.2f, colour: %f, %f, %f"%(idx, x, y, u, v, *colour))
            rr, cc = draw.ellipse(v, u, r_radius=radius, c_radius=radius, shape=image.shape) #TODO this should be more complicated shape
            image[rr,cc,:] = colour*255
        # else:
            # print("Invalid")
    
    return image


def create_cube(cube_size, num_pts):
    # Create points vector
    xx, yy, zz = np.meshgrid(np.linspace(-cube_size, cube_size, num_pts), np.linspace(-cube_size, cube_size, num_pts), np.linspace(-cube_size, cube_size, num_pts))
    xx = xx.reshape(-1)
    yy = yy.reshape(-1)
    zz = zz.reshape(-1)

    points = np.zeros((xx.shape[0], 3))
    points[:,0] = xx
    points[:,1] = yy
    points[:,2] = zz

    idx = np.where((points[:,0] == -cube_size) | (points[:,0] == cube_size) | (points[:,1] == -cube_size) | (points[:,1] == cube_size) | (points[:,2] == -cube_size) | (points[:,2] == cube_size))[0]
    points = points[idx, :]
    points = np.unique(points, axis=0)

    # Colour vector
    colours = np.zeros_like(points)
    colours[:,0] = (points[:,0] - points[:,0].min()) / (points[:,0].max() - points[:,0].min())
    colours[:,1] = (points[:,1] - points[:,1].min()) / (points[:,1].max() - points[:,1].min())
    colours[:,2] = (points[:,2] - points[:,2].min()) / (points[:,2].max() - points[:,2].min())

    # Return points and colours
    return points, colours


# def create_surface(side_size, num_pts, z_plane):
#     # Create points vector
#     xx, yy, zz = np.meshgrid(np.linspace(-side_size, side_size, num_pts), np.linspace(-side_size, side_size, num_pts), np.linspace(-side_size, side_size, num_pts))
#     xx = xx.reshape(-1)
#     yy = yy.reshape(-1)
#     zz = zz.reshape(-1)

#     points = np.zeros((xx.shape[0], 3))
#     points[:,0] = xx
#     points[:,1] = yy
#     points[:,2] = z_plane

#     points = np.unique(points, axis=0)
#     print(points.shape)
#     points = np.vstack([points, [0,0,z_plane]])
#     print(points.shape)

#     # Colour vector
#     colours = np.zeros_like(points)
#     colours[:,0] = (points[:,0] - points[:,0].min()) / (points[:,0].max() - points[:,0].min())
#     colours[:,1] = (points[:,1] - points[:,1].min()) / (points[:,1].max() - points[:,1].min())
#     colours[:,2] = (points[:,2] - points[:,2].min()) / (points[:,2].max() - points[:,2].min())
#     colours[-1, :] = [0,0,1] # blue origin

#     # Return points and colours
#     return points, colours


def visualise_coordinate_frames(camera_optical_frame):
    base_link_coord = o3d.geometry.TriangleMesh.create_coordinate_frame()
    base_link_coord.scale(1, center=(0, 0, 0))

    camera_optical_coord = o3d.geometry.TriangleMesh.create_coordinate_frame()
    camera_optical_coord.scale(0.5, center=(0, 0, 0))
    camera_optical_coord.transform(camera_optical_frame.A) 

    o3d.visualization.draw_geometries([base_link_coord, camera_optical_coord])


def visualise_cube_and_coordinate_frames(pcd, camera_optical_frame):
    base_link_coord = o3d.geometry.TriangleMesh.create_coordinate_frame()
    base_link_coord.scale(1, center=(0, 0, 0))

    camera_optical_coord = o3d.geometry.TriangleMesh.create_coordinate_frame()
    camera_optical_coord.scale(0.5, center=(0, 0, 0))
    camera_optical_coord.transform(camera_optical_frame.A) 

    o3d.visualization.draw_geometries([pcd, base_link_coord, camera_optical_coord])


def visualise_cube_results(pcd_original, pcd, image_original, image_trans, optical_frame):
    f, (ax1, ax2) = plt.subplots(1,2)
    ax1.imshow(image_original)
    ax1.set_title("Original")
    ax2.imshow(image_trans)
    ax2.set_title("Transformed")

    plt.show(block=False)
    plt.pause(1)

    # Open3d
    o3d.visualization.gui.Application.instance.initialize()
    mat = o3d.visualization.rendering.Material()
    mat.base_color = (1.0, 1.0, 1.0, 1.0)
    # mat.shader = "defaultLit"

    w = o3d.visualization.gui.Application.instance.create_window()

    # World Frame
    world_coord = o3d.geometry.TriangleMesh.create_coordinate_frame()
    world_coord.scale(1, center=(0, 0, 0))

    optical_coord = o3d.geometry.TriangleMesh.create_coordinate_frame()
    optical_coord.scale(0.5, center=(0, 0, 0))
    optical_coord.transform(optical_frame.A)

    world_scene = o3d.visualization.gui.SceneWidget()
    world_scene.scene = o3d.visualization.rendering.Open3DScene(w.renderer)
    world_scene.scene.add_geometry("World", world_coord, mat)
    world_scene.scene.add_geometry("Optical", optical_coord, mat)
    world_scene.scene.add_geometry("Pointcloud", pcd_original, mat)
    world_scene.setup_camera(60, world_scene.scene.bounding_box, (0, 0, 0))

    # Optical Frame
    optical_coord = o3d.geometry.TriangleMesh.create_coordinate_frame()
    optical_coord.scale(0.5, center=(0, 0, 0))

    world_coord = o3d.geometry.TriangleMesh.create_coordinate_frame()
    world_coord.scale(1, center=(0, 0, 0))
    world_coord.transform(optical_frame.inv().A)

    optical_scene = o3d.visualization.gui.SceneWidget()
    optical_scene.scene = o3d.visualization.rendering.Open3DScene(w.renderer)
    optical_scene.scene.add_geometry("World", world_coord, mat)
    optical_scene.scene.add_geometry("Optical", optical_coord, mat)
    optical_scene.scene.add_geometry("Pointcloud", pcd, mat)
    optical_scene.setup_camera(60, optical_scene.scene.bounding_box, (0, 0, 0))

    # Add Child
    w.add_child(world_scene)
    w.add_child(optical_scene)

    def on_layout(theme):
        r = w.content_rect
        world_scene.frame = o3d.visualization.gui.Rect(r.x, r.y, r.width / 2, r.height)
        optical_scene.frame = o3d.visualization.gui.Rect(r.x + r.width / 2 + 1, r.y, r.width / 2, r.height)

    w.set_on_layout(on_layout)

    o3d.visualization.gui.Application.instance.run()


# def visualise_surface_results(pcd_original, image_original, block=False):
#     # Image Vis
#     plt.imshow(image_original)
#     plt.show(block=block)
#     plt.pause(1)

#     # Open3d Vis
#     world_coord = o3d.geometry.TriangleMesh.create_coordinate_frame()
#     world_coord.scale(1, center=(0, 0, 0))

#     o3d.visualization.draw_geometries([pcd_original, world_coord])



if __name__ == "__main__":

    # Initialise camera model
    file_dir = os.path.abspath(os.path.dirname(__file__))
    cam_model = init_camera_model(os.path.join(file_dir, 'camera_info.yaml'))
    
    # Create a camera optical frame at [0.2, 0, 0.2]. The camera_optical_frame
    # is also the transform to go from the base_link frame to the camera_optical_frame.
    # We don't need to create a base_link frame in this example as it is at [0,0,0]
    camera_optical_frame = sm.SE3(0.2, 0, 0.2) * sm.SE3.Ry(90, unit='deg') * sm.SE3.Rz(-90, unit='deg')

    # Draw the two frames
    print("\nVisualising Coordinate Frames... Larger frame is base_link, smaller frame is the camera_optical frame")
    # visualise_coordinate_frames(camera_optical_frame)
    
    # Create cube point cloud
    points, colours = create_cube(2, 200)
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    pcd.colors = o3d.utility.Vector3dVector(colours)

    # Visualise point cloud and coordinate frames
    print("\nVisualise the coloured cube point cloud")
    # visualise_cube_and_coordinate_frames(pcd, camera_optical_frame)
    
    # Copy original and transform point cloud to optical frame
    pcd_original = copy.deepcopy(pcd)
    pcd.transform(camera_optical_frame.A)
    print(camera_optical_frame.inv())
    print(camera_optical_frame.inv().rpy())
    exit(0)

    # Project to image
    image_original = project_to_image(pcd_original)
    image_trans = project_to_image(pcd)

    # Visualisation
    # if test == 'surface':
    #     # print(image_trans[int(cam_model.height/2), int(cam_model.width/2), :])
    #     visualise_surface_results(pcd_original, image_original, plot_blocks)
    # else:
    visualise_cube_results(pcd_original, pcd, image_original, image_trans, camera_optical_frame)


