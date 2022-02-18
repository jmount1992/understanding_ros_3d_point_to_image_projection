# Understanding How To Project 3D Points Onto An Image Plane In ROS

This is a short demonstration and tutorial on how you can project 3D points into an image using the [Image Geometry](http://docs.ros.org/en/kinetic/api/image_geometry/html/python/index.html) python module. Unfortunately, the documentation of the model is limited and doesn't define if the x, y, z coordinates for a point need to be in the optical or body frame of the camera (see [Suffix Frames in REP103](https://www.ros.org/reps/rep-0103.html#suffix-frames) for information on body vs optical frames for cameras). Additionally, the `project3dToPixel` image geometry function only can handle a single point at a time, this is computational inefficient and we can rewrite this function in Python to utilise Numpy to allow the projection of any number of points with a single call. A working version of all the code can be found in [pointcloud_projection](pointcloud_projection.py)

## Getting Coordinate Frames Correct
The first thing we will investigate is what coordinate frame a point must be in, before it is passed to the `project3dToPixel` function. To do this we will first create and visualise two coordinate frames and a coloured cube using the [SpatialMath](https://github.com/petercorke/spatialmath-python) and [Open3D](http://www.open3d.org/) libraries. The larger coordinate frame in the image below represents our base link frame, while the smaller frame is the optical frame of our camera and is located at [0.2, 0, 0.2] relative to our base link coordinate frame. Remember that for a optical frame the z-axis points forward, the x-axis to the right, and the y-axis down, as specified by [REP103](https://www.ros.org/reps/rep-0103.html). The cube is nicely coloured and the origin of the cube is at [0, 0, 0] and hence the origin is the same location as the base_link coordinate frame.

![Coordinate Frames](/figures/coordinate_frames.png)

![Rotating Cube](/figures/cube_rotation.gif)

If we now take at a look at position our coordinates frames inside the cube we can see that our base link and camera optical frame are pointing at the edge where yellow is in the bottom left and magenta is in the top right. The black sphere represents the center of the point cloud and is located at [0, 0, 0]. Given this image, we can see that when we project the points onto an image we should have yellow in the bottom left and magenta in the top right. Let's see how we can make that happen.

![Inside The Cube](/figures/inside_cube.png)

First we need the transform from base link to the camera optical_frame. Since the position of our camera is at [0.2, 0, 0.2] and we know the optical frame must have the z-axis pointing forward, x-axis point to the right, and the y-axis point down we also know the camera optical frame is a 90 degree rotation about the y-axis, followed by a -90 degree rotation in the z-axis. The code to do this is:

```python
import spatialmath as sm
camera_optical_frame = sm.SE3(0.2, 0, 0.2) * sm.SE3.Ry(90, unit='deg') * sm.SE3.Rz(-90, unit='deg')
```

We now have the base link to camera optical frame transform, and we need to shift our points into the camera frame. To do this we are going to use the following piece of pseudo-code:

```python
<open3d.geometry.PointCloud>.transform(<spatialmath.SE3>.A)
```

where `<open3d.geometry.PointCloud>` is an Open3D point cloud containing the points we need to tranforms and `<spatialmath.SE3>` is a Spatial Math SE3 object that represents the appropriate transformation. You might intuitively think that the appropriate transformation is the base link to camera optical frame transform (the one we setup in the code snippet above). However, we actually want the inverse of this transform. Why? Well we want the camera optical frame to become the "origin". If that seems a little odd, think of this way. We want to move the camera optical frame "back" to the base link frame, which is the inverse of the base link to camera optical frame transform. By doing this we will move the location of the camera optical frame back to where the base link frame is now, and hence make the x, y, z points of our cube relative to the camera optical frame. So, to transform our point cloud cube we would do the following:

```python
pcd.transform(camera_optical_frame.inv().A)
```

The image below shows the transform when we do the inverse and when we don't do the inverse. The larger coordinate frame represents the base link frame, and the smaller coordinate frame represents the camera optical frame. The center of the point cloud is represented by the black sphere. Remember, we want the camera optical frame to be at [0, 0, 0] and so everything is visualised relative to it. So as we can see, in the left hand side of the image, the black sphere remains at the same location as the base link frame. However, in the right hand side of the image, where we have done `pcd.transform(camera_optical_frame.A)` (i.e., have done the base link to camera optical frame transform, rather than the inverse), the black sphere no longer aligns with the base link coordinate frame. In fact, if we considered the smaller coordinate frame, the one representing the camera optical frame, to be the base link frame, the position of the black sphere is in the location of where the camera optical frame would be (i.e., at [0.2, 0, 0.2]). We can also see that in the left hand image, where the inverse transform was used, our camera z-axis is still pointing at yellow in the bottom left and magenta in the bottom right. This is not the case for the right hand side of the image where the incorrect transform was used. Hopefully, this has demonstrated why we need to do the inverse of the base link to camera optical frame transform when shifting the point cloud. Now onto projecting the points onto the image plane.

![Transform Comparison](/figures/transform_comparison.png)

To project 3D points onto a image plane we need the projection matrix `P`. In a ROS environment we can obtain the projection matrix of a camera through the `camera_info` message. In fact, we can use a camera info topic and the Image Geometry module to create a pin hole camera model object, and utilise the class function `project3dToPixel` to project our 3D points. This class function utilises the projection matrix under the hood. Here is the code snippet to read in a camera info message stored as a YAML file and create a PinholeCameraModel object.

```python
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
```

Unfortunately, the documentation of the `project3dToPixel` doesn't specify the coordinate frame a point needs to be when projecting. If you know you're projection maths, you can look at the source code and determine which frame the points need to be in. However, let's assume we don't know the projection maths and so, we shall try the two obvious versions. The two obvious versions are body and optical frames of the camera. Remember, the optical frame is a rotation of 90 degree rotation about the y-axis, followed by a -90 degree rotation in the z-axis of the camera body frame. In other words, we have the following mapping:

* `x in the body frame = z in the optical frame`
* `y in the body frame = -x in the optical frame`
* `z in the body frame = -y in the optical frame`

Rather than extending out this demonstration and going through the incorrect first one first, I am just going to tell you the `project3dToPixel` functions expects the points in the camera optical frame. You can use the mapping above if you have transformed your points into the body frame. The code to project our points would be as follows:

```python
for point in points:
    x,y,z = point # point is in optical frame
    (u,v) = cam_model.project3dToPixel((x, y, z)) # use the mapping above to map body frame to optical frame if required
```

If we do that we get the following projected image.

![Projected Image 1](/figures/projected_image_1.png)

Woah, what has happened here? The function has projected the points behind camera onto the image plane. We can easily fix this by filtering out points behind the camera. The code to do so is simply:

```python
for point in points:
    x,y,z = point # point is in optical frame
    if z < 0: # ignore points behind the camera
        continue

    (u,v) = cam_model.project3dToPixel((x, y, z)) 
```

Okay that looks better. We are getting what we expected, yellow in the bottom-left and magenta in the top-right. 

![Projected Image 2](/figures/projected_image_2.png)
