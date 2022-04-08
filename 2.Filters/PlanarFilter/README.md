# Planar filter
This filter removes a planar surface from the point cloud.
Outputs PlanarFiltered.pcd

`<required>`

`[optional]`

*filename* is required and defines the point cloud file to be read.

*distanceToPlane* How far can the point be before not being included in the planar and being removed.

## Examples
./PlanarFilter Downsampled_0.pcd
- Locates the planar and removes any point around the default value 0.01 m away from it

./PlanarFilter Downsampled_0.pcd 0.014
- Locates the planar and removes any point around 0.01 m away from it

# Build and run
1. `mkdir build`
2. `cd build`
3. `cmake ..`
4. `make`
5. `./PlanarFilter <filename> [distanceToPlane]`
