#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

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
    float MIN_POINTS = 100;
    float MAX_POINTS = 25000;
    if (argc < 2)
    {
        std::cout << "Missing an input file argument!" << std::endl;
        std::cout << "Usage: <filename> [pointDistance] [minClusterSize] [maxClusterSize]" << std::endl;
        return -1;
    }
    for (int i = 2; i < argc && i < 5; i++)
    {
        if (i == 2)
        {
            if (!is_num(argv[i]))
            {
                std::cout << "pointDistance is not a decimal number!" << std::endl;
                return -1;
            }
            DISTANCE = atof(argv[i]);
        }
        else if (i == 3)
        {
            if (!is_num(argv[i]))
            {
                std::cout << "minClusterSize is not a decimal number!" << std::endl;
                return -1;
            }
            MIN_POINTS = atof(argv[i]);
        }
        else if (i == 4)
        {
            if (!is_num(argv[i]))
            {
                std::cout << "depthNear is not a decimal number!" << std::endl;
                return -1;
            }
            MAX_POINTS = atof(argv[i]);
        }
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Fill in the cloud data
    pcl::PCDReader reader;
    reader.read(argv[1], *cloud);
    std::cout << "PointCloud before filtering has: " << cloud->size() << " data points." << std::endl;

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(DISTANCE);
    ec.setMinClusterSize(MIN_POINTS);
    ec.setMaxClusterSize(MAX_POINTS);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &idx : it->indices)
            cloud_cluster->push_back((*cloud)[idx]); //*
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j;
        write_file(*cloud_cluster, ss.str());
        j++;
    }
    return (0);
}