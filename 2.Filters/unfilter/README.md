# unfilter
This filter acquires the nearest points from the original point cloud from the given cluster.
Outputs restored.pcd file.

`<required>`

`[optional]`

*filename* is required and defines the point cloud file to be read.

*inputCloud* Smaller cluster. This is the source where the nearest points are searched from the background cloud.

*backgroundCloud* The original point cloud where the nearest points are searched.

*pointDistance* The search radius for the nearest points.

NOTE: it does not matter how you put the clouds in the command. The code assumes the point cloud with a larger amount of points is the background cloud.
Also, make sure that the input cloud is from the background cloud; otherwise, weird results may occur.

## Examples
./unfilter cloud_cluster_0.pcd XYZ_0.pcd
- Restores the points from XYZ_0.pcd with a default search radius of 0.02 m

./unfilter cloud_cluster_0.pcd XYZ_0.pcd 0.03
- Restores the points from XYZ_0.pcd with a search radius of 0.03 m

# Build and run
1. `mkdir build`
2. `cd build`
3. `cmake ..`
4. `make`
5. `./unfilter <inputCloud> <backgroundCloud> [pointDistance]`
