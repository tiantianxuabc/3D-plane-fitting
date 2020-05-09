# 3D-plane-fitting
Fit 3d plane based on C++

![Point](point_cloud_640_576_kinect.png) | ![depth](.depth_color_image.jpg)

Input a 3d point cloud or depth image of depth camera and calibration information and fitting a 3D plane

Point cloud format ： X Y Z

depth is the distance between the point and camera.


Conversion of depth to point cloud through camera calibration information

camera calibration:

fx 0 cx

0 fy cy

0 0 1

formualtion 

X = (x - cx) / fx * depth

Y = (y - cy) / fy * depth 

Z = depth
			
