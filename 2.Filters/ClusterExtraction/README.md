# Cluster Extraction
This filter locates all the clusters that are between min and max point size and all the points in the cluster are at most certain distance away from the nearest neighbouyr.
Returns cloud_cluster_n.pcd

`<required>`

`[optional]`

*filename* is required and defines the point cloud file to be read.

*pointDistance* is the farthest distance allowed to the nearest pointr neigor.
  
*minClusterSize* Is the minimum size of the cluster.

*maxClusterSize* is the maximum size of the cluster.

## Examples
./ClusterExtraction PlanarFiltered.pcd
- Finds clusters with point distance of 0.01, min cluster size of 100 points and max cluster size of 25000

./ClusterExtraction PlanarFiltered.pcd 0.02
- Finds clusters with point distance of 0.02 and default cluster size limits.

./ClusterExtraction PlanarFiltered.pcd 0.03 300
- Finds clusters with point distance of 0.03, min 300 points in a cluster and a default max cluster size

./ClusterExtraction PlanarFiltered.pcd 0.04 350 19000
- Finds clusters with point distance of 0.04, min 350 points in a cluster and max of 19000 points in one cluster.

# Build and run
1. `mkdir build`
2. `cd build`
3. `cmake ..`
4. `make`
5. `./ClusterExtraction <filename> [pointDistance] [minClusterSize] [maxClusterSize]`
