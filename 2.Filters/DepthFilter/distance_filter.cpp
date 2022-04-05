#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;

void write_file(cloudPtr cloud, std::string filename)
{
  std::stringstream fullFileName;
  fullFileName << filename << ".pcd";
  pcl::io::savePCDFileASCII(fullFileName.str(), *cloud);
  std::cout << fullFileName.str() << " pointcloud saved!" << std::endl;
}

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
  float DEPTH_FAR = 2;
  float DEPTH_NEAR = 0.5;
  if (argc < 2)
  {
    std::cout << "Missing an input file argument!" << std::endl;
    std::cout << "Usage: <filename> [depthFar] [depthNear]" << std::endl;
    return -1;
  }
  for (int i = 2; i < argc && i < 4; i++)
  {
    if (i == 2)
    {
      if (!is_num(argv[i]))
      {
        std::cout << "depthFar is not a decimal number!" << std::endl;
        return -1;
      }
      DEPTH_FAR = atof(argv[i]);
    }
    else if (i == 3)
    {
      if (!is_num(argv[i]))
      {
        std::cout << "depthNear is not a decimal number!" << std::endl;
        return -1;
      }
      DEPTH_NEAR = atof(argv[i]);
    }
  }
  // Read pcd file
  pcl::PCDReader reader;
  cloudPtr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
  reader.read(argv[1], *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->size() << " data points." << std::endl; //*

  // Filter points based on distance
  pcl::PassThrough<pcl::PointXYZ> pass;
  cloudPtr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(DEPTH_NEAR, DEPTH_FAR); // passthrough distances
  pass.filter(*cloudOut);
  std::cout << "Amount of points after passthrough filtering: " << cloudOut->size() << std::endl;

  write_file(cloudOut, "DepthFiltered");

  return 0;
}
