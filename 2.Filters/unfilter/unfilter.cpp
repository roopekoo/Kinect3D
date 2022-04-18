#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;

cloudPtr read_file(cloudPtr cloud, std::string filename)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) //* load the file
    {
        std::cout << "Couldn't read file " << filename << std::endl;
        return NULL;
    }
    return cloud;
}

void write_file(cloudPtr cloud, std::string filename)
{
    std::string fullFileName = filename + ".pcd";
    pcl::io::savePCDFileASCII(fullFileName, *cloud);
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
    float RADIUS = 0.02; // points in the background cloud, radius of cloudIn points are included in the output cloud
    if (argc < 3)
    {
        std::cout << "Missing input or background pointclouds." << std::endl;
        std::cout << "Usage: <inputCloud> <backgroundCloud> [pointDistance]" << std::endl;
        return -1;
    }
    if (argc == 4)
    {
        if (!is_num(argv[3]))
        {
            std::cout << "pointDistance is not a decimal number!" << std::endl;
            return -1;
        }
        RADIUS = atof(argv[3]);
    }

    cloudPtr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
    cloudPtr cloudBack(new pcl::PointCloud<pcl::PointXYZ>);
    cloudPtr cloudTemp(new pcl::PointCloud<pcl::PointXYZ>);

    cloudIn = read_file(cloudIn, argv[1]);
    cloudTemp = read_file(cloudTemp, argv[2]);

    //Either pointcloud file is invalid
    if (cloudIn == NULL || cloudTemp == NULL)
    {
        return -1;
    }

    // Switch clouds to correct one by comparing the amount of points
    if (cloudIn->size() < cloudTemp->size())
    {
        cloudBack = cloudTemp;
    }
    else
    {
        cloudBack = cloudIn;
        cloudIn = cloudTemp;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloudBack);

    std::vector<int> pointIndexes;
    // Search nearest points in cloudBack from cloudIn within RADIUS of each point
    for (const auto &point : *cloudIn)
    {
        std::vector<int> pointIdxRadiusSearch;         // to store index of surrounding points
        std::vector<float> pointRadiusSquaredDistance; // to store distance to surrounding points

        if (kdtree.radiusSearch(point, RADIUS, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
            pointIndexes.insert(pointIndexes.end(), pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());
        }
    }

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    inliers->indices = pointIndexes;
    // Filter out the points that are not found in previus step.
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloudBack);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloudBack);

    write_file(cloudBack, "restored");

    return 0;
}