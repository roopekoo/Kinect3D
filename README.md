# Kinect3D
A collection of filtering methods and an interactive ICP
This GitHub repository is a side product of my bachelor's thesis [INSERT LINK  HERE] where the was to get the point clouds from the Kinect V2 and align the point clouds so that a complete object point cloud is acquired.

The code found here is a collection of mostly PCL's example codes but slightly modified.
In each sub-section, there are the required hardware/libraries that are needed to be able to run the programs.

## Depth image acquisition
The camera view combined with the depth data is shown. Pictures can be taken and those are converted to undistorted 3D-point clouds. These are saved in .pcd files.
C++ and Python implementations are available.

## PointCloud filtering
This section includes many filtering methods that might be needed to isolate the object point cloud from the background.

## Interactive ICP
In this section is a modified PCL's interactive ICP program.
This allows us to see the current position of the point cloud regarding the target point cloud.
If the alignment fails, the user can help the program by controlling the movable point cloud with 6-degrees-of-freedom.

## Misc
Here is some miscellaneous code that was tried on but not used. 

## Sidenote
Feel free to use, modify or improve the code by suggesting improvements or bugs in the issues section. Creating a pull request is also an option :)
