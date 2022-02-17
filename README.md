# Understanding How To Project 3D Points Onto An Image Plane In ROS

This is a short demonstration and tutorial on how you can project 3D points into an image using the [Image Geometry](http://docs.ros.org/en/kinetic/api/image_geometry/html/python/index.html) python module. Unfortunately, the documentation of the model is limited and doesn't define if the x, y, z coordinates for a point need to be in the optical or body frame of the camera (see [Suffix Frames in REP103](https://www.ros.org/reps/rep-0103.html#suffix-frames) for information on body vs optical frames for cameras). Additionally, the `project3dToPixel` image geometry function only can handle a single point at a time, this is computational inefficient and we can rewrite this function in Python to utilise Numpy to allow the projection of any number of points with a single call.

## Getting Coordinate Frames Correct
The first thing we will investigate is what coordinate frame a point must be in, before it is passed to the `project3dToPixel` function. To do this we will first create and visualise two coordinate frames and a coloured cube using the [SpatialMath](https://github.com/petercorke/spatialmath-python) and [Open3D](http://www.open3d.org/) libraries. The larger coordinate frame in the image below represents our base link frame, while the smaller frame is the optical frame of our camera and is located at [0.2, 0, 0.2] relative to our base link coordinate frame. Remember that for a optical frame the z-axis points forward, the x-axis to the right, and the y-axis down, as specified by [REP103](https://www.ros.org/reps/rep-0103.html). The cube is nicely coloured and the origin of the cube is at [0, 0, 0] and hence the origin is the same location as the base_link coordinate frame.

![Coordinate Frames](/figures/coordinate_frames.png)

![Rotating Cube](/figures/cube_rotation.gif)

If we now take at a look at position our coordinates frames inside the cube we can see that our base link and camera optical frame are pointing at the edge where yellow is in the bottom left and magenta is in the top right. Therefore, when we project the points onto an image we should have yellow in the bottom left and magenta in the top right. Let's see how we can make that happen.

![Inside The Cube](/figures/inside_cube.png)

First we need the transform from base link to the camera optical_frame. Since the position of our camera is at [0.2, 0, 0.2] and we know the optical frame must have the z-axis pointing forward, x-axis point to the right, and the y-axis point down we also know the camera optical frame is a 90 degree rotation about the y-axis, followed by a -90 degree rotation in the z-axis. The code to do this is:

```python
import spatialmath as sm
camera_optical_frame = sm.SE3(0.2, 0, 0.2) * sm.SE3.Ry(90, unit='deg') * sm.SE3.Rz(-90, unit='deg')
```

We now have the base link to camera optical frame transform, and we need to shift our points into the camera frame. To do this we are going to use the following piece of sudo code:

```python
<open3d.geometry.PointCloud>.transform(<spatialmath.SE3>.A)
```

where `<open3d.geometry.PointCloud>` is an Open3D point cloud containing the points we need to tranforms and `<spatialmath.SE3>` is a Spatial Math SE3 object that represents the appropriate transformation. You might intuitively think that the appropriate transformation is the base link to camera optical frame transform (the one we setup in the code snippet above). However, we actually want the inverse of this transform. Why? Well we want the camera optical frame to become the "origin". If that seems a little odd, think of this way. We want to move the camera optical frame "back" to the base link frame, which is the inverse of the base link to camera optical frame. By doing this we will move the location of the camera optical frame back to where the base link frame is now, and hence make the x, y, z points of our cube relative to the camera optical frame. So, to transform our point cloud cube we would do the following:

```python
pcd.transform(camera_optical_frame.inv().A)
```

The images below show the the transform when we don't do the inverse and when we do the inverse. The origin of the point cloud is represented by the large black point.
