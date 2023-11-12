# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# color segementation and connected components example added by Claude sSammut
  
# Import the necessary libraries


import rclpy # Python library for ROS 2
import rclpy.qos
from pyquaternion import Quaternion
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from nav_msgs.msg import Odometry

import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import numpy as np
import math

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray
 
class ImageSubscriber(Node):
  PINK = 0
  BLUE = 1
  GREEN = 2
  YELLOW = 3
  MAX_AREA_DETECTION_THRESHOLD = 1100
  MIN_AREA_DETECTION_THRESHOLD = 200

  BLUE_PINK = False
  PINK_BLUE = False
  GREEN_PINK = False
  YELLOW_PINK = False
  
  marker_counter = 0

  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
  
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
 #    'video_frames', 
      '/camera/image_raw/uncompressed', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning

    # Subscribe to odometry
    # self.subscription = self.create_subscription(
    #   Odometry,
    #   '/odom',
    #   self.listener_callback,
    #   10)
    
    self.position = None

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    #For storing objects detected by the robot's camera
    self.detected_objects = []

    # transform lisnter
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    self.marker_list = MarkerArray()
    self.marker_list.markers = []
    self.marker_pub = self.create_publisher(MarkerArray, "visualization_marker_array", 10)

  # def odom_callback(self, data):
  #   pos = data.pose.pose.position
  #   self.position = pos
  #   (posx, posy, posz) = (pos.x, pos.y, pos.z)

  def listener_callback(self, data):
    """
    Callback function.
    """
    self.detected_objects = []
    
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)

    # Convert BGR image to HSV
    hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
    self.image_size = hsv_frame.shape
    
    #Detect objects runs a connected component analysis and adds the blobs to a list "detected_objects"
    self.detect_objects(hsv_frame, current_frame)
    self.calcDistanceAndPublish()

    # Display camera image
    cv2.namedWindow("camera")
    cv2.imshow("camera", current_frame)
    
    cv2.waitKey(1)

  def detect_objects(self, hsv_frame, current_frame):
    
    light_pink = (158, 101, 168)
    dark_pink = (179, 255, 255)
    pink_mask = cv2.inRange(hsv_frame, light_pink, dark_pink)
    result = cv2.bitwise_and(current_frame, current_frame, mask=pink_mask)

    light_blue = (100, 50, 30)
    dark_blue = (105, 255, 255)
    blue_mask = cv2.inRange(hsv_frame, light_blue, dark_blue)
    result = cv2.bitwise_and(current_frame, current_frame, mask=blue_mask)

    light_green = (40, 65, 30)
    dark_green = (90, 255, 255)
    green_mask = cv2.inRange(hsv_frame, light_green, dark_green)
    result = cv2.bitwise_and(current_frame, current_frame, mask=green_mask)

    light_yellow = (20, 110, 30)
    dark_yellow = (35, 255, 255)
    yellow_mask = cv2.inRange(hsv_frame, light_yellow, dark_yellow)
    result = cv2.bitwise_and(current_frame, current_frame, mask=yellow_mask)

    #First detect pink
    # Run 4-way connected components, with statistics for blue+pink objects
    output = cv2.connectedComponentsWithStats(pink_mask, 4, cv2.CV_32S)
    (numLabels, labels, stats, centroids) = output
    
    blue_objects = cv2.connectedComponentsWithStats(blue_mask, 4, cv2.CV_32S)
    (numLabelsB, labelsB, statsB, centroidsB) = blue_objects

    yellow_objects = cv2.connectedComponentsWithStats(yellow_mask, 4, cv2.CV_32S)
    (numLabelsY, labelsY, statsY, centroidsY) = yellow_objects
        
    green_objects = cv2.connectedComponentsWithStats(green_mask, 4, cv2.CV_32S)
    (numLabelsG, labelsG, statsG, centroidsG) = green_objects

    for i in range(1, numLabels):
      x = stats[i, cv2.CC_STAT_LEFT]
      y = stats[i, cv2.CC_STAT_TOP]
      w = stats[i, cv2.CC_STAT_WIDTH]
      h = stats[i, cv2.CC_STAT_HEIGHT]
      area = stats[i, cv2.CC_STAT_AREA]
        
      if area >= self.MIN_AREA_DETECTION_THRESHOLD and area <= self.MAX_AREA_DETECTION_THRESHOLD:
        (centroid_x1, centroid_y1) = centroids[i]
        pink_on_top = True

        #Check for blue objects
        for j in range(1, numLabelsB):
          (centroid_x2, centroid_y2) = centroidsB[j]

          if (not (area <= 1.2 * statsB[j, cv2.CC_STAT_AREA] and area >= 0.8 * statsB[j, cv2.CC_STAT_AREA])):
            continue

          #check that x coordinate of the centroid of the blue blob is within +/- 10% of the x coordinate of the pink blob
          if (centroid_x2 <= 1.1 * centroid_x1 and centroid_x1 >= 0.9 * centroid_x1):
            #check that the the blue blob is on top of the pink blob
            if (centroid_y2 <= centroid_y1): 
              pink_on_top = False 
          print(f"color: blue, pink on top: {pink_on_top}, width: {w}, height: {h}, area: {area}, centroid of entire marker: {centroid_x1}, {centroid_x2}")
          self.add_objects(pink_mask, blue_mask, self.BLUE, pink_on_top)

        #Check for yellow objects
        for j in range(1, numLabelsY):
          (centroid_x2, centroid_y2) = centroidsY[j]

          if (not (area <= 1.2 * statsY[j, cv2.CC_STAT_AREA] and area >= 0.8 * statsY[j, cv2.CC_STAT_AREA])):
            continue

          #check that x coordinate of the centroid of the yellow blob is within +/- 10% of the x coordinate of the pink blob
          if (centroid_x2 <= 1.1 * centroid_x1 and centroid_x1 >= 0.9 * centroid_x1):
            #check that the the yellow blob is on top of the pink blob
            if (centroid_y2 <= centroid_y1): 
              pink_on_top = False
          print(f"color: yellow, pink on top: {pink_on_top}, width: {w}, height: {h}, area: {area}, centroid of entire marker: {centroid_x1}, {centroid_x2}")
          self.add_objects(pink_mask, yellow_mask, self.YELLOW, pink_on_top)
  
        #Check for green objects
        for j in range(1, numLabelsG):
          (centroid_x2, centroid_y2) = centroidsG[j]

          if (not (area <= 1.2 * statsG[j, cv2.CC_STAT_AREA] and area >= 0.8 * statsG[j, cv2.CC_STAT_AREA])):
            continue

          #check that x coordinate of the centroid of the green blob is within +/- 10% of the x coordinate of the pink blob
          if (centroid_x2 <= 1.1 * centroid_x1 and centroid_x1 >= 0.9 * centroid_x1):
            #check that the the green blob is on top of the pink blob
            if (centroid_y2 <= centroid_y1): 
              pink_on_top = False
          print(f"color: green, pink on top: {pink_on_top}, width: {w}, height: {h}, area: {area}, centroid of entire marker: {centroid_x1}, {centroid_x2}")
          self.add_objects(pink_mask, green_mask, self.GREEN, pink_on_top)
  
  def add_objects(self, mask1, mask2, color, pink_on_top):
    combined_mask = cv2.bitwise_or(mask1, mask2)

    output = cv2.connectedComponentsWithStats(combined_mask, 4, cv2.CV_32S)
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
      if area >= self.MIN_AREA_DETECTION_THRESHOLD and area <= self.MAX_AREA_DETECTION_THRESHOLD:
        self.detected_objects.append({"color": color, "pink_on_top": pink_on_top, "x": x, "y": y, "w": w, "h": h, "area": area, "centroid": centroids[i]})
        print(f"color: {color}, pink on top: {pink_on_top}, width: {w}, height: {h}, area: {area}")
  
  def calcDistanceAndPublish(self):
    if len(self.detected_objects) == 0: 
      return 
    
    # # Odometry Position
    # odom_pos = self.position
    # (posx, posy, posz) = (odom_pos.x, odom_pos.y, odom_pos.z)

    # # Transform odometry position to map
    # (translation, rotation) = self.tf_listener('/odom', '/map', rclpy.time.Time())
    
    #Transform robot's current position (base_link) to map coordinates
    
    translation = [0,0,0]
    quaternion = [1,0,0,0]


    for object in self.detected_objects:
      color = object["color"]
      pink_on_top = object["pink_on_top"]

      if (color == self.BLUE and pink_on_top == False and self.BLUE_PINK == False):
        self.BLUE_PINK = True
        self.generate_marker()
      elif (color == self.BLUE and pink_on_top == True and self.PINK_BLUE == False):
        self.PINK_BLUE = True
        self.add_new_point()
      elif (color == self.GREEN and self.GREEN_PINK == False):
        self.GREEN_PINK = True
        self.add_new_point()
      if (color == self.BLUE and pink_on_top == False and self.BLUE_PINK == False):
        self.BLUE_PINK = True
        self.add_new_point()


    # # real_distance = object_realHeight / object_pixelHeight * dist_objectToMarker
    # real_distance = dist_objectToMarker # * similarTriangleRatio
    # rel_xToCenter = object_fromLeft - self.image_size[0] / 2.0
    # # real_xToCenter = object_realHeight / object_pixelHeight * rel_xToCenter
    
    # # cylinder_absX = robot_currX + real_xToCenter
    # cylinder_absX = rel_xToCenter * similarTriangleRatio
    # cylinder_absY = math.sqrt(math.pow(real_distance, 2) - math.pow(cylinder_absX, 2))      
    # cylinder_absZ = 0
    
    # obj_in_cam = [cylinder_absY, cylinder_absX, 0]
    # # obj_in_cam = [0, 0, 0]
    # translation = [0,0,0]
    # quaternion = [1,0,0,0]
    # translation, quaternion = self.transform_frame("map", "camera_link", translation, quaternion)
    # if (len(translation) != 3):
    #   return

    # my_quater = Quaternion(quaternion[0], quaternion[1], quaternion[2],quaternion[3])
    # rotation = my_quater.rotate(obj_in_cam)
    # final_coordinate = [0,0,0]
    # final_coordinate[0] = rotation[0] + translation[0]
    # final_coordinate[1] = rotation[1] + translation[1]
    # final_coordinate[2] = rotation[2] + translation[2]

    # self.add_new_point(final_coordinate, object_color, pink_on_top)
  
  def generate_marker(self, coordinate, color, pink_on_top):
    pink = (255.0, 0.0, 230.0)
    yellow = (255.0, 239.0, 0.0)
    green = (0.0, 255.0, 34.0)
    blue = (43.0, 0.0, 255.0)

    top_rgb = pink
    if (pink_on_top == False and color == self.YELLOW):
      top_rgb = yellow
    elif (pink_on_top == False and color == self.GREEN):
      top_rgb = green
    elif (pink_on_top == False and color == self.BLUE):
      top_rgb = blue

    bot_rgb = pink
    if (pink_on_top == True and color == self.YELLOW):
      bot_rgb = yellow
    elif (pink_on_top == True and color == self.GREEN):
      bot_rgb = green
    elif (pink_on_top == True and color == self.BLUE):
      bot_rgb = blue

    #Generate top half of the marker
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.header.stamp = rclpy.time.Time()
    marker.id = self.marker_counter + 1
    self.marker_counter = self.marker_counter + 1
    marker.type = marker.CYLINDER
    marker.action = marker.ADD
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = float(coordinate[0])
    marker.pose.position.y = float(coordinate[1])
    marker.pose.position.z = float(coordinate[2]) + 0.32
    marker.scale.x = 0.17 # Change to 14 if needed
    marker.scale.y = 0.17 # Change to 14 if needed
    marker.scale.z = 0.23 # Change to 20 if needed
    marker.color.a = 1.0
    marker.color.r = top_rgb[0] / 255.0
    marker.color.g = top_rgb[1] / 255.0
    marker.color.b = top_rgb[2] / 255.0
    self.marker_list.markers.append(marker)

    # Generate bottom half of the marker
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.header.stamp = rclpy.time.Time()
    marker.id = self.marker_counter + 1
    self.marker_counter = self.marker_counter + 1
    marker.type = marker.CYLINDER
    marker.action = marker.ADD
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = float(coordinate[0])
    marker.pose.position.y = float(coordinate[1])
    marker.pose.position.z = float(coordinate[2]) + 0.3
    marker.scale.x = 0.17 # Change to 14 if needed
    marker.scale.y = 0.17 # Change to 14 if needed
    marker.scale.z = 0.23 # Change to 20 if needed
    marker.color.r = top_rgb[0] / 255.0
    marker.color.g = top_rgb[1] / 255.0
    marker.color.b = top_rgb[2] / 255.0
    self.marker_list.markers.append(marker)

    self.marker_pub.publish(self.marker_list)

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
