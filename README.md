# Kinect3D
A collection of filtering methods and an interactive ICP
This github repository is a sideproduct of my batchelor's thesis [INSERT LINK  HERE] where the was to get the pointclouds from the Kinect V2 and align the pointclouds that a coplete object point cloud is acquired.

The code found here is a collection of mostly PCL's example codes but slightly modified.
In each sub-section, there are the required hardware/libraries that are needed to be able to run the programs.

## Depth image acquisition
The camera viev combined with the depth data is shown. Pictures can be taken and those are converted to undistroted 3D-point clouds. These are saved in .pcd files.
C++ and Python implemetations are available.

## PointCloud filtering
This section includes many filtering methods that might be needed to isolate the object point cloud from the bacground.

## Interactive ICP
In this section is a modified PCL's interactive ICP program.
This allows to see the current position of the point cloud regarding to the target point cloud.
If the alignment fails, the user can help the program by controlling the movable point cloud with 6-degrees-of-freedom.

## Misc
Here are some miscelianous code that were tried on but not used. 

## Sidenote
Feel free to use, modify/improve the code by suggesting improvements or bugs in the issues-section. Creating a pull request is also an option :)
