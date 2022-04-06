#include <iostream>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

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

void write_file(pcl::PointCloud<pcl::PointXYZ> cloud, std::string filename)
{
    std::stringstream fullFileName;
    fullFileName << filename << ".pcd";
    pcl::io::savePCDFileASCII(fullFileName.str(), cloud);
    std::cout << fullFileName.str() << " pointcloud saved!" << std::endl;
}

int main(int argc, char *argv[])
{
  float DISTANCE = 0.01;
  if (argc < 2)
  {
    std::cout << "Missing an input file argument!" << std::endl;
    std::cout << "Usage: <filename> [distanceToPlane]" << std::endl;
    return -1;
  }
  if (argc == 3)
  {
    if (!is_num(argv[2]))
    {
      std::cout << "distanceToPlane is not a decimal number!" << std::endl;
      return -1;
    }
    DISTANCE = atof(argv[2]);
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read(argv[1], *cloud);

     std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
               << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

    // Objects for storing the point clouds.
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);

  // Get the plane model, if present.
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  
  seg.setInputCloud(cloud);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(DISTANCE);
  seg.setMaxIterations(100);
  seg.setOptimizeCoefficients(true);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0){
    std::cout << "Could not find a plane in the scene." << std::endl;
    return(-1);
  }
  // Copy the points of the plane to a new cloud.
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative (true);
  extract.filter(*cloud_filtered);
  std::cout << "Amount of points after removing planar: " << cloud_filtered->size() << std::endl;

  // Write the filtered pointcloud to a file
  write_file(*cloud_filtered,"PlanarFiltered");
  return (0);
}