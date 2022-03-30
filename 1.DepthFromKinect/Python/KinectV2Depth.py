# coding: utf-8

# import numpy as np
import numpy as np
import cv2
import sys
import open3d as o3d
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from simpleicp import PointCloud, SimpleICP

pic_counter = 0

try:
    from pylibfreenect2 import OpenGLPacketPipeline

    pipeline = OpenGLPacketPipeline()
except:
    try:
        from pylibfreenect2 import OpenCLPacketPipeline

        pipeline = OpenCLPacketPipeline()
    except:
        from pylibfreenect2 import CpuPacketPipeline

        pipeline = CpuPacketPipeline()
print("Packet pipeline:", type(pipeline).__name__)

fn2 = Freenect2()
# Test if Kinect is connected
if fn2.enumerateDevices() == 0:
    print("No device connected!")
    sys.exit(1)

# Setup device
serial = fn2.getDeviceSerialNumber(0)
device = fn2.openDevice(serial, pipeline=pipeline)

# Setup listener
listener = SyncMultiFrameListener(
    FrameType.Color | FrameType.Ir | FrameType.Depth)

# Register listeners
device.setColorFrameListener(listener)
device.setIrAndDepthFrameListener(listener)

# Start the device
device.start()

# NOTE: must be called after device.start()
registration = Registration(device.getIrCameraParams(),
                            device.getColorCameraParams())

undistorted = Frame(512, 424, 4)
registered = Frame(512, 424, 4)


def depth2xyz():
    global pic_counter
    points = []
    for v in range(undistorted.height):
        for u in range(undistorted.width):
            X, Y, Z = registration.getPointXYZ(undistorted, v, u)
            # Discard all nan values
            if X + Y + Z > 0:
                points.append([X, Y, Z])
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud('XYZ_{}.pcd'.format(pic_counter), pcd)
    pic_counter += 1


while True:
    frames = listener.waitForNewFrame()

    color = frames["color"]
    ir = frames["ir"]
    depth = frames["depth"]

    registration.apply(color, depth, undistorted, registered)

    depth_rect = cv2.rectangle(registered.asarray(dtype=np.uint8), (206, 162), (306, 262), (254, 0, 0), 1)
    cv2.imshow("color+depth", depth_rect)

    listener.release(frames)

    key = cv2.waitKey(delay=1)
    if key == ord('\r'):
        depth2xyz()
    elif key == ord('q'):
        break

device.stop()
device.close()

sys.exit(0)
