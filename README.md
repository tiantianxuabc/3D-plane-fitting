# 3D-plane-fitting
Fit 3d plane based on C++


Input a 3d point cloud or depth image of depth camera and calibration information and fitting a 3D plane

Point cloud format ： x y z

depth is the distance between the point and camera.


Conversion of depth to point cloud through camera calibration information

camera calibration:

fx 0 cx

0 fy cy

0 0 1

formualtion 

X = (x - cx) / f * depth

Y = (y - cy) / f * depth 

Z = depth
			
