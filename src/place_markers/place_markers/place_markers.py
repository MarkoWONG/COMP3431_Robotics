# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# color segementation and connected components example added by Claude sSammut
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
import rclpy.qos
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type

import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import numpy as np
import math

from nav_msgs.msg import Odometry
from pyquaternion import Quaternion

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray
 
class ImageSubscriber(Node):
  # Colour dectection
  PINK = 0
  BLUE = 1
  GREEN = 2
  YELLOW = 3
  MAX_AREA_DETECTION_THRESHOLD = 800
  MIN_AREA_DETECTION_THRESHOLD = 375

  BLUE_PINK = False
  PINK_BLUE = False
  GREEN_PINK = False
  PINK_GREEN = False
  YELLOW_PINK = False
  PINK_YELLOW = False

  # Robot Position
  robot_pos_x = 0
  robot_pos_y = 0
  robot_pos_z = 0
  robot_ori_x = 0
  robot_ori_y = 0
  robot_ori_z = 0
  robot_ori_w = 0
  robot_yaw = 0

  # Create an ImageSubscriber class, which is a subclass of the Node class.
  def __init__(self):
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # ------------------ Subscribers and Publishers ---------------------------
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
 #    'video_frames', 
      '/camera/image_raw/uncompressed', 
      self.camera_callback, 
      10)
    self.subscription # prevent unused variable warning
      
    # Subscribe to odometery, to get robot position
    self.subscription2 = self.create_subscription(
      Odometry,
      '/odom',
      self.odom_callback,
      rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
    )
    # position listener
    self.qos = rclpy.qos.QoSProfile(
        reliability=rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        depth=1
    )
    # transform listener
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    # Create publisher
    self.marker_publisher = self.create_publisher(MarkerArray, "visualization_marker_array", 10)

    # --------------------- Class Members ---------------------------------------
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    #For storing objects detected by the robot's camera
    self.detected_objects = []
    self.image_size = None
    
    # For storing all markers
    self.marker_list = MarkerArray()
    self.marker_list.markers = [] 

# ------------------------------- CallBack Functions -----------------------------
  # What to do when recieving a image from camera
  def camera_callback(self, data):
    
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)

    # Convert BGR image to HSV
    hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
    self.image_size = hsv_frame.shape
    
    self.detected_objects = []
    # find colour objects and place it in detected_objects list
    self.detect_objects(hsv_frame, current_frame)

    # Add detected_objects --> marker_list
    self.add_detected_objects()

    # publish marker_list
    print(f"-----------Current length{len(self.marker_list.markers)}----------")
    self.marker_publisher.publish(self.marker_list)

    # Display camera image
    cv2.namedWindow("camera")
    cv2.imshow("camera", current_frame)
    # Display masked image
    # cv2.imshow("mask", mask)
    
    cv2.waitKey(1)


  #  What to do when recieving position from robot
  def odom_callback(self, msg):
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    self.robot_pos_x, self.robot_pos_y, self.robot_pos_z = (position.x, position.y, position.z)
    self.robot_ori_x, self.robot_ori_y, self.robot_ori_z, self.robot_ori_w = (orientation.x, orientation.y, orientation.z, orientation.w)
    
    # Extract the yaw angle (orientation) from the quaternion
    roll, pitch, self.robot_yaw = self.euler_from_quaternion(orientation)

# ------------------------------- Helper Functions -----------------------------
  def euler_from_quaternion(self, quaternion):
    # Convert a quaternion to roll, pitch, and yaw (Euler angles)
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw
  
  # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  # Find colour blobs from camera feed and place it in dectected list
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


    # First detect pink
    # Run 4-way connected components, with statistics for blue+pink objects
    output = cv2.connectedComponentsWithStats(pink_mask, 4, cv2.CV_32S)
    (numLabels, labels, stats, centroids) = output
    
    blue_objects = cv2.connectedComponentsWithStats(blue_mask, 4, cv2.CV_32S)
    (numLabelsB, labelsB, statsB, centroidsB) = blue_objects

    yellow_objects = cv2.connectedComponentsWithStats(yellow_mask, 4, cv2.CV_32S)
    (numLabelsY, labelsY, statsY, centroidsY) = yellow_objects
        
    green_objects = cv2.connectedComponentsWithStats(green_mask, 4, cv2.CV_32S)
    (numLabelsG, labelsG, statsG, centroidsG) = green_objects

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
        (centroid_x1, centroid_y1) = centroids[i]

        #Check for blue objects
        for j in range(1, numLabelsB):
          pink_on_top = True
          (centroid_x2, centroid_y2) = centroidsB[j]

          if (not (area <= 1.2 * statsB[j, cv2.CC_STAT_AREA] and area >= 0.8 * statsB[j, cv2.CC_STAT_AREA])):
            continue

          #check that x coordinate of the centroid of the blue blob is within +/- 10% of the x coordinate of the pink blob
          if (centroid_x2 <= 1.1 * centroid_x1 and centroid_x1 >= 0.9 * centroid_x1):
            #check that the the blue blob is on top of the pink blob
            if (centroid_y2 <= centroid_y1): 
              pink_on_top = False 

          print(f"colour: blue, pink on top: {pink_on_top}, width: {w}, height: {h}, area: {area}, centroid of entire marker: {centroid_x1}, {centroid_x2}")
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

          print(f"colour: yellow, pink on top: {pink_on_top}, width: {w}, height: {h}, area: {area}, centroid of entire marker: {centroid_x1}, {centroid_x2}")
          self.add_objects(pink_mask, yellow_mask, self.YELLOW, pink_on_top)
  
        #Check for green objects
        for j in range(1, numLabelsG):
          (centroid_x2, centroid_y2) = centroidsG[j]

          if (not (area <= 1.2 * statsG[j, cv2.CC_STAT_AREA] and area >= 0.8 * statsG[j, cv2.CC_STAT_AREA])):
            continue

          #check that x coordinate of the centroid of the yellow blob is within +/- 10% of the x coordinate of the pink blob
          if (centroid_x2 <= 1.1 * centroid_x1 and centroid_x1 >= 0.9 * centroid_x1):
            #check that the the green blob is on top of the pink blob
            if (centroid_y2 <= centroid_y1): 
              pink_on_top = False
          print(f"colour: yellow, pink on top: {pink_on_top}, width: {w}, height: {h}, area: {area}, centroid of entire marker: {centroid_x1}, {centroid_x2}")
          self.add_objects(pink_mask, green_mask, self.GREEN, pink_on_top)
  
  # Add marker to detected_objects list
  def add_objects(self, mask1, mask2, colour, pink_on_top):
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
        
      # If the height of blob is > 25 and < 55 then add it to dectected
      # otherwise distance calculation will not be accurate
      if h >= 25 and h <= 55 and self.new_marker(colour, pink_on_top):
        self.detected_objects.append({"colour": colour, "pink_on_top": pink_on_top, "x": x, "y": y, "w": w, "h": h, "area": area, "centroid": centroids[i]})
      
      #If the area of the blob is more than 250 pixels:
      # if area >= self.MIN_AREA_DETECTION_THRESHOLD and area <= self.MAX_AREA_DETECTION_THRESHOLD:
      #   self.detected_objects.append({"colour": colour, "pink_on_top": pink_on_top, "x": x, "y": y, "w": w, "h": h, "area": area, "centroid": centroids[i]})
 
  # check if dectected object is new 
  def new_marker(self, dectected_colour, pink_on_top):
    if pink_on_top == True:
        if self.PINK_BLUE == False and dectected_colour == self.BLUE:
          self.PINK_BLUE = True
          return True
        if self.PINK_GREEN == False and dectected_colour == self.GREEN:
          self.PINK_GREEN = True
          return True
        if self.PINK_YELLOW == False and dectected_colour == self.YELLOW:
          self.PINK_YELLOW = True
          return True
    else:
      if self.BLUE_PINK == False and dectected_colour == self.BLUE:
        self.BLUE_PINK = True
        return True
      if self.GREEN_PINK == False and dectected_colour == self.GREEN:
        self.GREEN_PINK = True
        return True
      if self.YELLOW_PINK == False and dectected_colour == self.YELLOW:
        self.YELLOW_PINK = True
        return True
    return False
  
  # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  # Add detected_objects --> marker_list
  def add_detected_objects(self):
    # Add all detected objects to marker_list
    for object in self.detected_objects:

      coordinates = self.calculateCoordinates(object) 
      print(f"coordinates for {object['colour']} marker is x:{coordinates[0]} y:{coordinates[1]} z:{coordinates[2]}")
      
      # Creates marker object and adds to marker list
      self.CreateMarker(coordinates, object['colour'], object['pink_on_top'])

  def calculateCoordinates(self, object):
    distance = self.distMarkerToCamera(object['h'])

    # Calculate the markers coordinates based on distance and yaw
    object_x_coord = self.robot_pos_x + distance * math.cos(self.robot_yaw)
    object_y_coord = self.robot_pos_y + distance * math.sin(self.robot_yaw)
    object_z_coord = 0
    obj_in_cam = [object_y_coord, object_x_coord, object_z_coord]
    # return obj_in_cam
  
    # Victor's angle adjustment
    # object_realHeight = 0.2 
    # similarTriangleRatio = object_realHeight / object['h']
    # rel_xToCenter = object['x'] - self.image_size[0] / 2.0
    # cylinder_absX = rel_xToCenter * similarTriangleRatio
    # cylinder_absY = math.sqrt(math.pow(distance, 2) - math.pow(cylinder_absX, 2))      
    # cylinder_absZ = 0
    # obj_in_cam = [cylinder_absY, cylinder_absX, cylinder_absZ]

    # Tranform coordinate to coordinate on cartographer map
    translation = [0,0,0]
    quaternion = [1,0,0,0]
    translation, quaternion = self.transformFrame("map", "camera_link", translation, quaternion)
    if (len(translation) != 3):
      return
    my_quater = Quaternion(quaternion[0], quaternion[1], quaternion[2],quaternion[3])
    rotation = my_quater.rotate(obj_in_cam)
    final_coordinate = [0,0,0]
    final_coordinate[0] = rotation[0] + translation[0]
    final_coordinate[1] = rotation[1] + translation[1]
    final_coordinate[2] = rotation[2] + translation[2]
    return final_coordinate
  
  def distMarkerToCamera(self, marker_height):
    # Quadratic regression equation distance = a*x^2 + b*x + c
    a = 0.000561
    b = -0.0622
    c = 2.22
    # distance = (a*(marker_height**2)) - (b*marker_height) + c 
    distance = 1
    print(f"Distance From Camera to Marker was {distance}m")
    return distance

  def transformFrame(self, target, source , translation, quaternion):
      try:
        transform = self.tf_buffer.lookup_transform(target_frame=target, source_frame=source, time=rclpy.time.Time()).transform
        translation[0] = translation[0] + transform.translation.x
        translation[1] = translation[1] + transform.translation.y
        translation[2] = translation[2] + transform.translation.z
        quaternion[0] = transform.rotation.w
        quaternion[1] = quaternion[1] + transform.rotation.x
        quaternion[2] = quaternion[2] + transform.rotation.y
        quaternion[3] = quaternion[3] + transform.rotation.z
        return (translation, quaternion)
      except Exception:
        return ([0],[0])
  
  def CreateMarker(self, coordinate, color, pink_on_top):
    # Create top half of marker
    self.CreateHalfMarker(coordinate, color, pink_on_top, True)

    # Create bottom half of marker
    self.CreateHalfMarker(coordinate, color, pink_on_top, False)

  def CreateHalfMarker(self, coordinate, color, pink_on_top, top):
    marker = Marker()
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
    if top == True:
      marker.pose.position.z = float(coordinate[2]) + 0.3
    else:
      marker.pose.position.z = float(coordinate[2]) + 0.1
    marker.scale.x = 0.14
    marker.scale.y = 0.14
    marker.scale.z = 0.2
    marker.color.a = 1.0
    if pink_on_top == True:
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

    # add to marker list
    self.marker_list.markers.append(marker)

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
