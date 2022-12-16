# Kinect3D
A collection of filtering methods and an interactive ICP.
This GitHub repository is a side product of my bachelor's thesis [here](https://urn.fi/URN:NBN:fi:tuni-202205195106). The goal was to get the point clouds from Kinect V2 and align the point clouds with acquiring a complete object point cloud.

The code found here is a collection of mostly PCL's example codes but slightly modified.
In each sub-section, there are the required hardware/libraries that are needed to be able to run the programs.

## Depth image acquisition
The camera view combined with the depth data is shown. Pictures can be taken, and those are converted to undistorted 3D-point clouds. These are saved in .pcd files.
C++ and Python implementations are available.

## PointCloud filtering
This section includes many filtering methods that might be needed to isolate the object point cloud from the background.

## Interactive ICP
In this section, PCL's interactive ICP program is modified.
This allows us to see the current position of the point cloud regarding the target point cloud.
If the alignment fails, the user can help the program by controlling the movable point cloud with 6-degrees-of-freedom.

## Point cloud data
The entire point clouds are taken from a Steam controller from different views. There is also a larger point clouds taken from larger object, sofa.

## Sidenote
Feel free to use, modify or improve the code by suggesting improvements or bugs in the issues section. Creating a pull request is also an option :)
