# Depth Data Acquisition from Kinect v2
Python and c++ implementations are available.
With this program, you can see the live image from the device. The image merges the RGB and the depth images together.

Point the Kinect camera to a desired position and press ***ENTER*** to take a picture of the scene. A new point *.pcd* file will be created.
You can take as many pictures as you want. The naming goes as follows: *XYZ_0.pcd*, *XYZ_1.pcd* etc.
The outputted files are undistorted 3D point clouds without color information.
You can quit the program by pressing the ***q*** key.


## Requirements

### Hardware
Kinect v2

### Software

#### C++
+ libfreenect2
+ opencv2
+ pcl

#### Python
+ numpy
+ cv2
+ open3d
+ pylibfreenect2

# Build and run
## C++
1. create a build directory in /C++ `mkdir build`
2. go to the build directory `cd build`
3. `cmake ..`
4. `make`
5. run with `./KinectV2Depth`

## python
`python KinectV2Depth.py`
