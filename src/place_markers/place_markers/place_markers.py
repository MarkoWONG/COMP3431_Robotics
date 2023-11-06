# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Colour segementation and connected components example added by Claude sSammut
  
# Import the necessary libraries

import rclpy # Python library for ROS 2
import rclpy.qos
from pyquaternion import Quaternion
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type

import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import numpy as np
import math

from tf2_ros.buffer import Buffer
from visualization_msgs.msg import Marker, MarkerArray
 
class ImageSubscriber(Node):
  PINK = 0
  BLUE = 1
  GREEN = 2
  YELLOW = 3
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')

    self.marker_list = MarkerArray();
    self.marker_list.marker = [];
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
 #    'video_frames', 
      '/camera/image_raw/uncompressed', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    #For storing objects detected by the robot's camera
    self.detected_objects = [];
    self.image_size = None;

    # position listener
    self.qos = rclpy.qos.QoSProfile(
        reliability=rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        depth=1
    )

    # transform lisnter
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
    self.plot_publisher = self.create_publisher(MarkerArray, "visualization_marker_array", 10)
  
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)

    # The following code is a simple example of colour segmentation
    # and connected components analysis
    
    # Convert BGR image to HSV
    hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
    self.image_size = hsv_frame.shape;

    self.calcDistanceAndPublish()
    
    # #PINK
    # light_pink = (158, 101, 168)
    # dark_pink = (179, 255, 255)
    # pink_mask = cv2.inRange(hsv_frame, light_pink, dark_pink)

    #We only need to worry about blue, green and yellow because all the markers are half pink
    self.find_blue_objects(hsv_frame, current_frame)
    self.find_green_objects(hsv_frame, current_frame)
    self.find_yellow_objects(hsv_frame, current_frame)
         
    # Display camera image
    cv2.namedWindow("camera")
    cv2.imshow("camera", current_frame)
    # Display masked image
    # cv2.imshow("mask", mask)
    
    cv2.waitKey(1)

  def calcDistanceAndPublish(self):
    if len(self.detected_objects) == 0: 
      return 

    robot_currX = 0;
    robot_currY = 0;
    robot_currZ = 0;
    
    object = self.detected_objects[0];
    object_height = object.h;
    object_width = object.w;
    object_fromLeft = object.x;
    object_fromTop = object.y;
    object_area = object.area;
    object_centroid = object.centroid;
    object_color = object.color;

    dist_objectToMarker = self.distMarkerToCamera(object_height);
    object_realHeight = 200;
    object_pixelHeight = object_height * 0.2646;

    real_distance = object_realHeight / object_pixelHeight * dist_objectToMarker;
    rel_xToCenter = object_fromLeft - self.image_size[0] / 2.0;
    real_xToCenter = object_realHeight / object_pixelHeight * rel_xToCenter
    
    cylinder_absX = robot_currX + real_xToCenter;
    cylinder_absY = math.sqrt(math.pow(real_distance, 2) - math.pow(cylinder_absX, 2))      
    cylinder_absZ = 0;

    self.add_new_point([cylinder_absX, cylinder_absY, cylinder_absZ], object_color)
  
  def generate_marker(self, coordinate, color):
    # down cylinder
    marker = Marker()
    # marker.header.frame_id = "map"
    marker.header.frame_id = "/map"
    marker.id = len(self.marker_list.markers) + 1
    marker.type = marker.CYLINDER
    marker.action = marker.ADD
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = float(coordinate[0])
    marker.pose.position.y = float(coordinate[1])
    marker.pose.position.z = float(coordinate[2]) + 0.1
    marker.scale.x = 0.14
    marker.scale.y = 0.14
    marker.scale.z = 0.2
    marker.color.a = 1.0
    if special_color_up != True:
      rgb = (255,192,203)
    elif color == self.YELLOW:
      rgb = (255,234,0)
    elif color == self.BLUE:
      rgb = (0,191,255)
    else:
      rgb = (0,100,0)
    marker.color.r = rgb[0] / 255.0
    marker.color.g = rgb[1] / 255.0
    marker.color.b = rgb[2] / 255.0
    self.marker_list.markers.append(marker)

  def add_new_point(self, coordinate, color):
    coordinate[0] = float(coordinate[0])
    coordinate[1] = float(coordinate[1])
    coordinate[2] = float(coordinate[2])
    ## print("add_new_point2")
    self.generate_marker(coordinate, color)
    print(f"-----------Current length{len(self.marker_list.markers)}----------")
    self.plot_publisher.publish(self.marker_list)

  def find_blue_objects(self, hsv_frame, current_frame):
    # Filter out everything that is not blue
    light_blue = (100, 50, 30)
    dark_blue = (105, 255, 255)
    
    blue_mask = cv2.inRange(hsv_frame, light_blue, dark_blue)
    result = cv2.bitwise_and(current_frame, current_frame, mask=blue_mask)

    cv2.namedWindow("blue_mask")
    cv2.imshow("blue_mask", blue_mask)

    # Run 4-way connected components, with statistics
    output = cv2.connectedComponentsWithStats(blue_mask, 4, cv2.CV_32S)
    (numLabels, labels, stats, centroids) = output
 
    # Print statistics for each blob (connected component)
    # use these statistics to find the bounding box of each blob
    # and to line up laser scan with centroid of the blob
    for i in range(1, numLabels):
      x = stats[i, cv2.CC_STAT_LEFT]
      y = stats[i, cv2.CC_STAT_TOP]
      w = stats[i, cv2.CC_STAT_WIDTH]
      h = stats[i, cv2.CC_STAT_HEIGHT]
      area = stats[i, cv2.CC_STAT_AREA]
        
      #If the area of the blob is more than 250 pixels:
      if area > 250:
        print("Found a blue object")
        (cX, cY) = centroids[i]
        print(x, y, w, h, area, f"{cX}, {cY}")
        
        self.detected_objects.append({"x": x, "y": y, "w": w, "h": h, "area": area, "centroid": centroids[i], "color": self.BLUE})
  
  def find_green_objects(self, hsv_frame, current_frame):
    # Filter out everything that is not green
    light_green = (40, 65, 30)
    dark_green = (90, 255, 255)
    green_mask = cv2.inRange(hsv_frame, light_green, dark_green)
    result = cv2.bitwise_and(current_frame, current_frame, mask=green_mask)
    
    cv2.namedWindow("green_mask")
    cv2.imshow("green_mask", green_mask)

    # Run 4-way connected components, with statistics
    output = cv2.connectedComponentsWithStats(green_mask, 4, cv2.CV_32S)
    (numLabels, labels, stats, centroids) = output
 
    # Print statistics for each blob (connected component)
    # use these statistics to find the bounding box of each blob
    # and to line up laser scan with centroid of the blob
    for i in range(1, numLabels):
      x = stats[i, cv2.CC_STAT_LEFT]
      y = stats[i, cv2.CC_STAT_TOP]
      w = stats[i, cv2.CC_STAT_WIDTH]
      h = stats[i, cv2.CC_STAT_HEIGHT]
      area = stats[i, cv2.CC_STAT_AREA]
        
      #If the area of the blob is more than 250 pixels:
      if area > 250:
        print("Found a green object")
        print(x, y, w, h, area)
        self.detected_objects.append({"x": x, "y": y, "w": w, "h": h, "area": area, "centroid": centroids[i], "color": self.GREEN})

  def find_yellow_objects(self, hsv_frame, current_frame):
    # Filter out everything that is not yellow
    light_yellow = (20, 110, 30)
    dark_yellow = (35, 255, 255)
    yellow_mask = cv2.inRange(hsv_frame, light_yellow, dark_yellow)
    result = cv2.bitwise_and(current_frame, current_frame, mask=yellow_mask)

    cv2.namedWindow("yellow_mask")
    cv2.imshow("yellow_mask", yellow_mask)

    # Run 4-way connected components, with statistics
    output = cv2.connectedComponentsWithStats(yellow_mask, 4, cv2.CV_32S)
    (numLabels, labels, stats, centroids) = output
 
    # Print statistics for each blob (connected component)
    # use these statistics to find the bounding box of each blob
    # and to line up laser scan with centroid of the blob
    for i in range(1, numLabels):
      x = stats[i, cv2.CC_STAT_LEFT]
      y = stats[i, cv2.CC_STAT_TOP]
      w = stats[i, cv2.CC_STAT_WIDTH]
      h = stats[i, cv2.CC_STAT_HEIGHT]
      area = stats[i, cv2.CC_STAT_AREA]
        
      #If the area of the blob is more than 250 pixels:
      if area > 250:
        print("Found a yellow object")
        print(x, y, w, h, area)
        self.detected_objects.append({"x": x, "y": y, "w": w, "h": h, "area": area, "centroid": centroids[i], "color": self.YELLOW})

def showDistance(self, marker_height):
    if (marker_height >= 15 and marker_height <= 65):
      print(f'Distance to marker is {self.distMarkerToCamera(marker_height)}')
    else:
      print(f"Too close or far to marker, Distance won't be accurate. Distance is less than 200mm or greater than 500mm")
      
  def distMarkerToCamera(self, marker_height):
    # Quadratic regression equation distance = a*x^2 + b*x + c
    a = 0.192229
    b = -23.8783
    c = 941.638
    distance = (a*(marker_height**2)) - (b*marker_height) + c 
    showDistance(marker_height)
    return distance

def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
