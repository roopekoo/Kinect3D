# Downsampler
This filter averages the points in the point cloud in a 3D voxel grid.
Returns _Downsampled.pcd_ file

`<required>`

`[optional]`

*filename* is required and defines the point cloud file to be read. 

*voxelSize* is the side length of the voxel cube. Larger value=more filtered points and less quality.

## Examples
./DownSample XYZ_0.pcd
  - downsamples the point cloud with 0.01 m voxel cube (1x1x1 cm^3)

./DownSample XYZ_0.pcd 0.007
  - downsamples the point cloud with 0.007 m voxel cube (7x7x7 mm^3)

# Build and run
1. `mkdir build`
2. `cd build`
3. `cmake ..`
4. `make`
5. `./DownSample <filename> [voxelSize]`
