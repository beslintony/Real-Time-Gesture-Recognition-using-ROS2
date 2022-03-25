# Import the necessary libraries
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
# from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import numpy as np
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

from openni import openni2
from openni import _openni2 as c_api

import open3d as o3d

# optional: when the device's OpenNI path not included in the system
dist = "/home/fdai6135/AstraOpenNI2Drivers/OpenNI-Linux-x64-2.3.0.66/Redist"


class DepthPublisher(Node):
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('depth_pub')

    # Create the publisher. This publisher will publish an Image
    # to the depth_image topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(
        sensor_msgs.Image, 'depth_image', 10)

    self.pcd_publisher = self.create_publisher(
            sensor_msgs.PointCloud2, 'pcd', 10)

    # We will publish a message every 0.1 seconds
    timer_period = 1/30.0  # seconds

    # Open and register the initialized Device
    self.device = openni2.Device.open_any()
    # Register Message
    if self.device:
      print("Device has been registered")
    else:
      print("Error! Device Registration failed!")

    # Create the depth stream
    self.depth_stream = self.device.create_depth_stream()
    # Display mode and resolution config from the OpenNI
    self.depth_stream.set_video_mode(c_api.OniVideoMode(pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_1_MM, resolutionX=640, resolutionY=480,
                          fps=30))
    # Start the output stream
    self.depth_stream.start()
    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)
    # Convertion between ROS and OpenCV images
    self.br = CvBridge()

  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.1 seconds.
    """
    # Display depth stream from the Function get_depth_image
    self.cap = self.get_depth_image()
 
    # Publish the image.
    # The 'cv2_to_imgmsg' method converts an OpenCV
    # image to a ROS 2 image message
    self.publisher_.publish(self.br.cv2_to_imgmsg(self.cap))

    self.pcd = point_cloud(self.get_points(), 'map')
    self.pcd_publisher.publish(self.pcd)

    # Display the message on the console
    self.get_logger().info('Publishing depth image')


  def get_depth_image(self):
    distanceMap = np.frombuffer(self.depth_stream.read_frame(
    ).get_buffer_as_uint16(), dtype=np.uint16).reshape(480, 640)
    # Correct the range, Depth images are 12bits
    depthImage = np.uint8(distanceMap.astype(float) * 255 / 2 ** 12 - 1)
    # Fill with blacks
    depthImage = 255 - depthImage
    return depthImage

  def get_points(self):
      depth_o3d = o3d.geometry.Image((self.get_depth_image()).astype(np.uint16))

      pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_o3d,
                                                              o3d.camera.PinholeCameraIntrinsic(
                                                                  o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault),
                                                              np.identity(4), depth_scale=100.0, depth_trunc=1000.0)
      return np.asarray(pcd.points)

def point_cloud(points, parent_frame):
    """ Creates a point cloud message.
    Args:
        points: Nx3 array of xyz positions.
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message

    Code source:
        https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0

    References:
        https://github.com/ros2/examples/blob/master/rclpy/topics/pointcloud_publisher/examples_rclpy_pointcloud_publisher/pointcloud_publisher.py

    """
    # In a PointCloud2 message, the point cloud is stored as an byte
    # array. In order to unpack it, we also include some parameters
    # which desribes the size of each individual point.

    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize  # A 32-bit float takes 4 bytes.

    data = points.astype(dtype).tobytes()

    # The fields specify what the bytes represents. The first 4 bytes
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]

    # The PointCloud2 message also has a header which specifies which
    # coordinate frame it is represented in.
    header = std_msgs.Header(frame_id=parent_frame)

    return sensor_msgs.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3),  # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]),
        data=data
    )


def main(args=None):
  # Initialize OpenNI library
  openni2.initialize(dist)
  if openni2.is_initialized():
    print("openni2 driver has been initialized!")
  else:
    print("Error! openni2 driver has not been initialized!")

  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  depth_pub = DepthPublisher()
  
  # Spin the node so the callback function is called.
  rclpy.spin(depth_pub)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  depth_pub.destroy_node()
  openni2.unload()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()
if __name__ == '__main__':
  main()