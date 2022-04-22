# Depth filter
This simple passthrough filter lets you filter out the points that do not belong to the given filter band.
This filters the z values of the points.

`<required>`

`[optional]`

*filename* is required and defines the point cloud file to be read.

*depthFar* is the farthest distance value a point can have without being filtered.
  
*depthNear* is the smallest value the point can have without being filtered out.

## Examples
./distance_filter XYZ_0.pcd
- Uses the default values to passthrough point between 0.5 to 2 meters

./distance_filter XYZ_0.pcd 1
- Uses the default values to passthrough point between 0.5 to 1 meter

./distance_filter XYZ_0.pcd 1.2 0.8
- Uses the default values to passthrough point between 0.8 to 1.2 meter

# Build and run
1. `mkdir build`
2. `cd build`
3. `cmake ..`
4. `make`
5. `./distance_filter <filename> [depthFar] [depthNear]`
