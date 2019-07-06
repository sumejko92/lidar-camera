# Lidar-camera

This repo contains the lidar_camera package that takes inputs from a 3D lidar and separate camera, combines the inputs in such a way as to produce two separate outputs:
1. A PointCloud2, that includes all the LIDAR points that overlap the camera image with added R, G and B data channels that contain the color of the point in the environment.
2. A standard uint16 depth image that is of the same size and FOV as the camera image and has metric depth data for each of the pixels.

More info on the setup and how to run can be found inside the package.


