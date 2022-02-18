import open3d as o3d
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
import yaml

import numpy as np
from skimage import draw
import matplotlib.pyplot as plt
import spatialmath as sm

import os
import copy
import time
import cv2


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


def project_to_image(pcd, z_filter_on : bool = True, flip : bool = False, radius: int = 20):
    image = np.ones((cam_model.height, cam_model.width, 3), dtype='uint8') * 255

    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    if flip:
        # only doing this because by chance the ordering of the point cloud
        # has the yellow-magenta side later than the blue-green side. So this
        # is used to demonstrate importance of the z filter. All it does is reverse
        # the order of the points in the array, it doesn't change the data itself
        points = np.flipud(points)
        colors = np.flipud(colors)

    # iterate through points
    for (point, colour) in zip(points, colors):

        x, y, z = point        
        if z_filter_on and z < 0:
            continue # ignore points behind image plane
        (u,v) = cam_model.project3dToPixel((x, y, z))

        # Paint pixel
        if (u >= 0 and v >= 0) and (u < cam_model.width and v < cam_model.height):
            rr, cc = draw.ellipse(v, u, r_radius=radius, c_radius=radius, shape=image.shape) #TODO this should be more complicated shape
            image[rr,cc,:] = colour*255
    
    # return projected image
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

    # Create open3d point cloud object
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    pcd.colors = o3d.utility.Vector3dVector(colours)

    # Return cube
    return pcd


def visualise_coordinate_frames(camera_optical_frame):
    base_link_coord = o3d.geometry.TriangleMesh.create_coordinate_frame()
    base_link_coord.scale(1, center=(0, 0, 0))

    camera_optical_coord = o3d.geometry.TriangleMesh.create_coordinate_frame()
    camera_optical_coord.scale(0.5, center=(0, 0, 0))
    camera_optical_coord.transform(camera_optical_frame.A) 

    o3d.visualization.draw_geometries([base_link_coord, camera_optical_coord])


def visualise_cube_and_coordinate_frames(pcd, camera_optical_frame):
    origin_pcd = o3d.geometry.TriangleMesh.create_sphere(0.1)
    origin_pcd.paint_uniform_color([0,0,0])

    base_link_coord = o3d.geometry.TriangleMesh.create_coordinate_frame()
    base_link_coord.scale(1, center=(0, 0, 0))

    camera_optical_coord = o3d.geometry.TriangleMesh.create_coordinate_frame()
    camera_optical_coord.scale(0.5, center=(0, 0, 0))
    camera_optical_coord.transform(camera_optical_frame.A) 

    o3d.visualization.draw_geometries([pcd, base_link_coord, camera_optical_coord, origin_pcd])


def visualise_transformed_point_clouds(pcd_transform_correct, pcd_transform_incorrect, camera_optical_frame):

    # Create point cloud origin and transform by same amount as we did the actual point cloud
    origin_pcd_transform_correct = o3d.geometry.TriangleMesh.create_sphere(0.1)
    origin_pcd_transform_correct.transform(camera_optical_frame.inv().A)
    origin_pcd_transform_correct.paint_uniform_color([0,0,0])

    origin_pcd_transform_incorrect = o3d.geometry.TriangleMesh.create_sphere(0.1)
    origin_pcd_transform_incorrect.transform(camera_optical_frame.A)
    origin_pcd_transform_incorrect.paint_uniform_color([0,0,0])


    # Create coordinate frames, where camera optical frame is now at the origin
    base_link_coord = o3d.geometry.TriangleMesh.create_coordinate_frame()
    base_link_coord.scale(1, center=(0, 0, 0))
    base_link_coord.transform(camera_optical_frame.inv().A)
    # camera frame is origin, so need to move base link frame by inverse of base link to optical frame

    camera_optical_coord = o3d.geometry.TriangleMesh.create_coordinate_frame()
    camera_optical_coord.scale(0.5, center=(0, 0, 0))

    camera_optical_orig_coord = o3d.geometry.TriangleMesh.create_coordinate_frame()
    camera_optical_orig_coord.scale(0.25, center=(0, 0, 0))
    camera_optical_orig_coord.transform(camera_optical_frame.A)
    # camera_optical_orig_coord.paint_uniform_color([0.2, 0.2, 0.2])

    # Open3d
    o3d.visualization.gui.Application.instance.initialize()
    mat = o3d.visualization.rendering.Material()
    mat.base_color = (1.0, 1.0, 1.0, 1.0)
    # mat.shader = "defaultLit"

    w = o3d.visualization.gui.Application.instance.create_window()

    # Correct Transform Scene
    correct_scene = o3d.visualization.gui.SceneWidget()
    correct_scene.scene = o3d.visualization.rendering.Open3DScene(w.renderer)
    correct_scene.scene.add_geometry("Base", base_link_coord, mat)
    correct_scene.scene.add_geometry("Optical", camera_optical_coord, mat)
    correct_scene.scene.add_geometry("Pointcloud", pcd_transform_correct, mat)
    correct_scene.scene.add_geometry("Origin", origin_pcd_transform_correct, mat)
    # correct_scene.scene.add_geometry("OriginalCamera", camera_optical_orig_coord, mat)
    correct_scene.setup_camera(60, correct_scene.scene.bounding_box, (0, 0, 0))

    # Incorrect Transform Scene
    incorrect_scene = o3d.visualization.gui.SceneWidget()
    incorrect_scene.scene = o3d.visualization.rendering.Open3DScene(w.renderer)
    incorrect_scene.scene.add_geometry("Base", base_link_coord, mat)
    incorrect_scene.scene.add_geometry("Optical", camera_optical_coord, mat)
    incorrect_scene.scene.add_geometry("Pointcloud", pcd_transform_incorrect, mat)
    incorrect_scene.scene.add_geometry("Origin", origin_pcd_transform_incorrect, mat)
    # incorrect_scene.scene.add_geometry("OriginalCamera", camera_optical_orig_coord, mat)
    incorrect_scene.setup_camera(60, incorrect_scene.scene.bounding_box, (0, 0, 0))

    # Add Child
    w.add_child(correct_scene)
    w.add_child(incorrect_scene)

    def on_layout(theme):
        r = w.content_rect
        correct_scene.frame = o3d.visualization.gui.Rect(r.x, r.y, r.width / 2, r.height)
        incorrect_scene.frame = o3d.visualization.gui.Rect(r.x + r.width / 2 + 1, r.y, r.width / 2, r.height)

    w.set_on_layout(on_layout)

    o3d.visualization.gui.Application.instance.run()


def visualise_image(image):
    f, (ax1) = plt.subplots(1,1)
    ax1.imshow(image)
    ax1.set_title("Image")

    plt.show(block=True)




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
    visualise_coordinate_frames(camera_optical_frame)
    
    # Create cube point cloud
    cube_pcd = create_cube(2, 200)

    # Visualise point cloud and coordinate frames
    print("\nVisualise the coloured cube")
    visualise_cube_and_coordinate_frames(cube_pcd, camera_optical_frame)
    
    # Copy original and transform point cloud to optical frame
    # Let's transform using current and inverse of camera optical frame transform
    print("\nTransforming the point cloud and visualising... left hand side is the correct transform")
    pcd_transform_correct = copy.deepcopy(cube_pcd)
    pcd_transform_correct.transform(camera_optical_frame.inv().A)

    pcd_transform_incorrect = copy.deepcopy(cube_pcd)
    pcd_transform_incorrect.transform(camera_optical_frame.A)

    # Visualise the two transformed point clouds
    visualise_transformed_point_clouds(pcd_transform_correct, pcd_transform_incorrect, camera_optical_frame)


    # Project to image and visualise image
    print("\nProjecting Image... without z-filter")
    projected_image = project_to_image(pcd_transform_correct, False, True)
    visualise_image(projected_image)

    print("\nProjecting Image... with z-filter")
    projected_image = project_to_image(pcd_transform_correct)
    visualise_image(projected_image)


