# TangoViewer for Tango Tablet
## Description
Project Tango Tablet Point Cloud Viewer with Camera Sync.

Original Codes from https://github.com/googlesamples/tango-examples-c/tree/master/point-cloud-jni-example namanujan edition.

I add some codes for RGB Camera sync.

## Dependency
You need Android NDK and OpenCV to compile this project.

# TangoViewer for PC ver 2
## Description
This program shows PointCloud files created by TangoViewer for Tango Tablet.

Select Folder that contains *.bin files from tablet.

## Dependency
 - OpenGL (Included in Windows SDK)
 - GLM Library (Included in repository)
 - Point Cloud Library (http://unanancyowen.com/?p=1255&lang=en, Visual Studion 2015 Version)

## History
 - Version 0.0 : Initial Commit
 - Version 0.1 : Add PCL
 - Version 0.3 : Plane Segmentation
 - Version 0.4 : Object Oriented Bounding Box
 - Version 0.5 : Odometry Correction
 - Version 0.7 : Add some functions
 - Version 1.0 : Add Server using Boost::asio
 - Version 2.0 : Rewritten codes
