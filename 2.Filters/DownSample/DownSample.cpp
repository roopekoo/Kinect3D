#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

bool is_num(std::string numstr)
{
  int i = 0;
  int str_len = numstr.size();
  bool isNum = true;
  bool isFirstDot = true;
  while (i < str_len && isNum)
  {
    if (!std::isdigit(numstr[i]))
      {
        // Allow first decimal point but not a second one
        if (numstr[i] == '.' && isFirstDot)
        {
          isFirstDot = false;
        }
        else
        {
          isNum = false;
        }
      }
    i++;
  }
  return isNum;
}

int main(int argc, char *argv[])
{
     float LEAF_SIZE = 0.01;
     if(argc <2)
     {
          std::cout << "Missing an input file argument!" << std::endl;
          std::cout << "Usage: <filename> [voxelSize]" << std::endl;
          return -1;
     }
     if(argc ==3)
     {
          if(!is_num(argv[2]))
          {
               std::cout << "voxelSize is not a decimal number!" << std::endl;
               return -1;
          }
          LEAF_SIZE = atof(argv[2]);
     }
     pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
     pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2());

     // Fill in the cloud data
     pcl::PCDReader reader;
     reader.read (argv[1], *cloud);

     std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
          << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

     // Create the filtering object
     pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
     sor.setInputCloud (cloud);
     sor.setLeafSize (LEAF_SIZE,LEAF_SIZE,LEAF_SIZE);
     sor.filter (*cloud_filtered);

     std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
          << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

     pcl::PCDWriter writer;
     writer.write ("Downsampled.pcd", *cloud_filtered, 
          Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

     return (0);
}