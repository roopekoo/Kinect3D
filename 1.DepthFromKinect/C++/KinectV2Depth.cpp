#include <iostream>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/highgui.hpp>
#include <opencv2/core/mat.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

libfreenect2::Freenect2 freenect2;
libfreenect2::Freenect2Device *dev = 0;
libfreenect2::PacketPipeline *pipeline = 0;

void write_file(pcl::PointCloud<pcl::PointXYZ> cloud, std::string filename)
{
    std::stringstream fullFileName;
    fullFileName << filename << ".pcd";
    pcl::io::savePCDFileASCII(fullFileName.str(), cloud);
    std::cout << fullFileName.str() << " pointcloud saved!" << std::endl;
}

void depth2XYZ(libfreenect2::Registration *registration, libfreenect2::Frame *undistorted, int pic_counter)
{
    pcl::PointCloud<pcl::PointXYZ> points;
    int height = undistorted->height;
    int width = undistorted->width;
    for (int v = 0; v < height; v++)
    {
        for (int u = 0; u < width; u++)
        {
            float X, Y, Z = 0;
            registration->getPointXYZ(undistorted, v, u, X, Y, Z);
            // allow only valid points, (disallow nan, inf and 0)
            if (X + Y + Z > 0)
            {
                points.push_back(pcl::PointXYZ(X, Y, Z));
            }
        }
    }
    std::cout << pic_counter << std::endl;
    std::stringstream filename;
    filename << "XYZ_" << pic_counter;
    write_file(points, filename.str());
}

int main()
{
    // Test if Kinect is connected
    if (freenect2.enumerateDevices() == 0)
    {
        std::cout << "No device connected!" << std::endl;
        return -1;
    }
    // Get device serial
    std::string serial = freenect2.getDefaultDeviceSerialNumber();

    // Use OpenGL pipeline
    // OpenCLPacketPipeline and CpuPacketPipeline can be used
    pipeline = new libfreenect2::OpenGLPacketPipeline();

    // Open device
    dev = freenect2.openDevice(serial, pipeline);

    // Setup listeners
    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    libfreenect2::FrameMap frames;

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);

    // Start the device
    dev->start();

    // NOTE: must be called after device.start()
    libfreenect2::Registration *registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());

    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

    int pic_counter = 0;
    while (true)
    {
        listener.waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        registration->apply(rgb, depth, &undistorted, &registered);

        cv::Mat colorDepth = cv::Mat(
            registered.height, registered.width,
            CV_8UC4, registered.data);

        // Show current camera view depth+color view
        cv::imshow("RGB+Depth", colorDepth);
        int key = cv::waitKey(1);
        // ENTER pressed->take a picture
        if (key == 13)
        {
            depth2XYZ(registration, &undistorted, pic_counter);
            pic_counter++;
        }
        // pressed q-> quit the program
        else if (key == 113)
        {
            break;
        }

        listener.release(frames);
    }

    dev->stop();
    dev->close();
    return 0;
}