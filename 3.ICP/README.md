# Interactive iterative closest point
This is a modified version of PCL's version of interactive ICP. The user can see the stationary(white) and movable(red) point clouds.

By pressing ***SPACE***, you perform 1 ICP iteration.

You can select a moving mode with keys:
- ***1***: move the point cloud along the x-axis
- ***2***: move the point cloud along the y-axis
- ***3***: move the point cloud along the z-axis
- ***4***: rotate the point cloud around the x-axis
- ***5***: rotate the point cloud around the y-axis
- ***6***: rotate the point cloud around the z-axis

You can move/rotate the point cloud with the given selected direction using the ***UP*** and ***DOWN*** arrow keys.

Quit the program with the ***q*** key.

The output gives a merged point cloud called *merged.pcd*

## Requirements
*PCL*

# Build and run
`mkdir build`
`cd build`
`cmake ..`
`make`
`./interactive_icp <stationaryCloud> <movableCloud> [iterations]`

## Examples
`./interactive_icp file1.pcd file2.pcd`
- file1 is the stationary cloud and file2 to be aligned. Initial default iteration amount is 1.


`./interactive_icp merged.pcd file3.pcd 10`
- merged is the stationary cloud and file3 to be aligned. The initial iteration amount is 10.
