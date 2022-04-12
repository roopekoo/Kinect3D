#include <iostream>
#include <string>
#include <math.h>

#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h> // TicToc

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// The point clouds we will be using
PointCloudT::Ptr cloud_in(new PointCloudT);  // Original point cloud
PointCloudT::Ptr cloud_tr(new PointCloudT);  // Transformed point cloud
PointCloudT::Ptr cloud_icp(new PointCloudT); // ICP output point cloud

Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

bool next_iteration = false;
bool update_pc = false;
int command = 0;
const float STEP = 0.001;
const float ANGLE = M_PI/180*1; // 1 degree in radians

void print4x4Matrix(const Eigen::Matrix4d &matrix)
{
  printf("Rotation matrix :\n");
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
  printf("Translation vector :\n");
  printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

bool is_num(std::string str)
{
  for(char &c: str)
  {
    if(!isdigit(c))
    {
      return false;
    }
  }
  return true;
}

void move(bool forward)
{
  transformation_matrix = Eigen::Matrix4d::Identity();
  float step = STEP;
  if(!forward)
  {
    step = -step;
  }
  //Take step along x axis
  if(command == 1)
  {
    transformation_matrix(0,3) = step;
  }
  //Take step along y axis
  else if(command == 2)
  {
    transformation_matrix(1,3) = step;
  }
  //Take step along z axis
  else if(command == 3)
  {
    transformation_matrix(2,3) = step;
  }
  pcl::transformPointCloud(*cloud_icp,*cloud_icp,transformation_matrix);
  update_pc = true;
}

void rotate(bool forward)
{
  transformation_matrix = Eigen::Matrix4d::Identity();
  float angle = ANGLE;
  if(!forward)
  {
    angle = -angle;
  }
  float cos_th = cos(angle);
  float sin_th = sin(angle);
  float sin_neg_th = -sin(angle);

  //Find centroid point and rotate pointcloud around that
  pcl::PointXYZ centroid;
  pcl::computeCentroid(*cloud_icp, centroid);


  transformation_matrix(0,3) = -centroid.x;
  transformation_matrix(1,3) = -centroid.y;
  transformation_matrix(2,3) = -centroid.z;
  pcl::transformPointCloud(*cloud_icp,*cloud_icp,transformation_matrix);

  transformation_matrix = Eigen::Matrix4d::Identity();
  //Rotate around x axis
  if(command == 4)
  {
    transformation_matrix(1,1) = cos_th;
    transformation_matrix(1,2) = sin_neg_th;
    transformation_matrix(2,1) = sin_th;
    transformation_matrix(2,2) = cos_th;
  }
  //Rotate around y axis
  else if(command == 5)
  {
    transformation_matrix(0,0) = cos_th;
    transformation_matrix(0,2) = sin_th;
    transformation_matrix(2,0) = sin_neg_th;
    transformation_matrix(2,2) = cos_th;
  }
  //Rotate around z axis
  else if(command == 6)
  {
    transformation_matrix(0,0) = cos_th;
    transformation_matrix(0,1) = sin_neg_th;
    transformation_matrix(1,0) = sin_th;
    transformation_matrix(1,1) = cos_th;
  }
  pcl::transformPointCloud(*cloud_icp,*cloud_icp,transformation_matrix);
  transformation_matrix = Eigen::Matrix4d::Identity();
  //Move pointcloud cack to centroid
  transformation_matrix(0,3) = centroid.x;
  transformation_matrix(1,3) = centroid.y;
  transformation_matrix(2,3) = centroid.z;
  pcl::transformPointCloud(*cloud_icp,*cloud_icp,transformation_matrix);

  update_pc = true;
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *nothing)
{
  std::string sym = event.getKeySym();
  if(event.keyDown())
  {
    if(sym == "space")
    {
      next_iteration = true;
    }
    else if(sym == "Up")
    {
      if(command>0)
      {
        if(command < 4)
        {
          move(true);
        }
        else if(command < 7)
        {
          rotate(true);
        }
      }
    }
    else if(sym == "Down")
    {
      if(command>0)
      {
        if(command < 4)
        {
          move(false);
        }
        else if(command < 7)
        {
          rotate(false);
        }
      }
    }
    
  }
  else if(event.keyUp())
  {
    if(is_num(sym))
    {
      command = std::stoi(sym);
    }
  }
}

void write_file(PointCloudT::Ptr cloud, std::string filename)
{
  std::string fullFileName = filename + ".pcd";
  pcl::io::savePCDFileASCII(fullFileName, *cloud);
}

int main(int argc, char *argv[])
{
  // Checking program arguments
  if (argc < 3)
  {
    printf("Usage :\n");
    printf("\t\t%s file1.pcd file1.pcd number_of_ICP_iterations\n", argv[0]);
    PCL_ERROR("Provide two pcd files.\n");
    return (-1);
  }

  int iterations = 1; // Default number of ICP iterations
  if (argc > 3)
  {
    // If the user passed the number of iteration as an argument
    iterations = atoi(argv[3]);
    if (iterations < 1)
    {
      PCL_ERROR("Number of initial iterations must be >= 1\n");
      return (-1);
    }
  }

  pcl::console::TicToc time;
  time.tic();
  if (pcl::io::loadPCDFile(argv[1], *cloud_in) < 0)
  {
    PCL_ERROR("Error loading cloud %s.\n", argv[1]);
    return (-1);
  }
  std::cout << "\nLoaded file " << argv[1] << " (" << cloud_in->size() << " points) in " << time.toc() << " ms\n"
            << std::endl;
  if (pcl::io::loadPCDFile(argv[2], *cloud_icp) < 0)
  {
    PCL_ERROR("Error loading cloud %s.\n", argv[1]);
    return (-1);
  }
  std::cout << "\nLoaded file " << argv[1] << " (" << cloud_in->size() << " points) in " << time.toc() << " ms\n"
            << std::endl;

  // Display in terminal the transformation matrix
  std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
  print4x4Matrix(transformation_matrix);

  // Executing the transformation
  *cloud_tr = *cloud_icp; // We backup cloud_icp into cloud_tr for later use

  // The Iterative Closest Point algorithm
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setInputSource(cloud_icp);
  icp.setInputTarget(cloud_in);


  // Visualization
  pcl::visualization::PCLVisualizer viewer("ICP demo");
  // Create two vertically separated viewports

  // The color we will be using
  float bckgr_gray_level = 0.0; // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  // Original point cloud is white
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
  viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in");

  // ICP aligned point cloud is red
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_icp, 180, 20, 20);
  viewer.addPointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp");

  // Adding text descriptions in each viewport
  viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl);
  viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl);

  std::stringstream ss;
  ss << iterations;
  std::string iterations_cnt = "ICP iterations = " + ss.str();
  viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl);

  // Set background color
  viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);

  // Register keyboard callback :
  viewer.registerKeyboardCallback(&keyboardEventOccurred, (void *)NULL);

  // Display the visualiser
  while (!viewer.wasStopped())
  {
    viewer.spinOnce();

    // The user pressed "space" :
    if(next_iteration)
    {
      // The Iterative Closest Point algorithm
      time.tic();
      icp.align(*cloud_icp);
      std::cout << "Applied 1 ICP iteration in " << time.toc() << " ms" << std::endl;

      if (icp.hasConverged())
      {
        printf("\033[11A"); // Go up 11 lines in terminal output.
        printf("\nICP has converged, score is %+.5e\n", icp.getFitnessScore());
        std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix *= icp.getFinalTransformation().cast<double>(); // WARNING /!\ This is not accurate! For "educational" purpose only!
        print4x4Matrix(transformation_matrix);                                // Print the transformation between original pose and current pose

        ss.str("");
        ss << iterations;
        std::string iterations_cnt = "ICP iterations = " + ss.str();
        viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
        viewer.updatePointCloud(cloud_icp, cloud_icp_color_h,"cloud_icp");
      }
      else
      {
        PCL_ERROR("\nICP has not converged.\n");
        return (-1);
      }
      next_iteration = false;
    }
    if(update_pc)
    {
      viewer.updatePointCloud(cloud_icp, cloud_icp_color_h,"cloud_icp");
      update_pc = false;
    }
  }
  *cloud_icp = *cloud_icp + *cloud_in;
  write_file(cloud_icp, "merged");
  return (0);
}
