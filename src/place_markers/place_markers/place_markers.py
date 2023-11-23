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

  #Pixel detection thresholds
  MAX_AREA_DETECTION_THRESHOLD = 2000
  MIN_AREA_DETECTION_THRESHOLD = 400

  BLUE_PINK = False
  PINK_GREEN = False
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

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    #For storing objects detected by the robot's camera
    self.detected_objects = []

    # transform lisnter
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    # Set up marker array and publisher
    self.marker_list = MarkerArray()
    self.marker_list.markers = []
    self.marker_pub = self.create_publisher(MarkerArray, "/visualization_marker_array", 10) #added a slash to visualization marker array
    
  def listener_callback(self, data):
    """
    Callback function.
    """
    #Clear list of detected objects
    self.detected_objects = []
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)

    # Convert BGR image to HSV
    hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
    self.image_size = hsv_frame.shape
    
    #Runs connected component analysis and adds the detected markers to a list "detected_objects"
    self.detect_objects(hsv_frame, current_frame)
    
    #Calcualtes coordinates and publishes the markers to cartographer.
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
    
    cv2.namedWindow("camera1")
    cv2.imshow("camera1", pink_mask)
    cv2.namedWindow("camera2")
    cv2.imshow("camera2", green_mask)
    cv2.namedWindow("camera2")
    cv2.imshow("camera2", blue_mask)
    cv2.namedWindow("camera3")
    cv2.imshow("camera3", yellow_mask)

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
          # print(f"color: blue, pink on top: {pink_on_top}, width: {w}, height: {h}, area: {area}, centroid of entire marker: {centroid_x1}, {centroid_x2}")
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
          # print(f"color: yellow, pink on top: {pink_on_top}, width: {w}, height: {h}, area: {area}, centroid of entire marker: {centroid_x1}, {centroid_x2}")
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
          # print(f"color: green, pink on top: {pink_on_top}, width: {w}, height: {h}, area: {area}, centroid of entire marker: {centroid_x1}, {centroid_x2}")
          self.add_objects(pink_mask, green_mask, self.GREEN, pink_on_top)
  
  def add_objects(self, mask1, mask2, color, pink_on_top):
    combined_mask = cv2.bitwise_or(mask1, mask2)

    output = cv2.connectedComponentsWithStats(combined_mask, 4, cv2.CV_32S)
    (numLabels, labels, stats, centroids) = output
 
    for i in range(1, numLabels):
      x = stats[i, cv2.CC_STAT_LEFT]
      y = stats[i, cv2.CC_STAT_TOP]
      w = stats[i, cv2.CC_STAT_WIDTH]
      h = stats[i, cv2.CC_STAT_HEIGHT]
      area = stats[i, cv2.CC_STAT_AREA]
        
      if area >= self.MIN_AREA_DETECTION_THRESHOLD and area <= self.MAX_AREA_DETECTION_THRESHOLD:
        self.detected_objects.append({"color": color, "pink_on_top": pink_on_top, "x": x, "y": y, "w": w, "h": h, "area": area, "centroid": centroids[i]})
        # print(f"color: {color}, pink on top: {pink_on_top}, width: {w}, height: {h}, area: {area}")
  
  def calcDistanceAndPublish(self):
    if len(self.detected_objects) == 0: 
      return 
    
    #Transform robot's current position (base_link) to map coordinates
    translation = [0, 0, 0]
    quaternion = [1, 0, 0, 0]
    origin = [0, 0, 0]

    transformation = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
    
    #Rotation
    quaternion[0] = transformation.transform.rotation.w
    quaternion[1] = quaternion[1] + transformation.transform.rotation.x
    quaternion[2] = quaternion[2] + transformation.transform.rotation.y
    quaternion[3] = quaternion[3] + transformation.transform.rotation.z
    
    rotationQuaternion = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
    transformed_coordinate = rotationQuaternion.rotate(origin)
    
    #Translation
    translation[0] = transformation.transform.translation.x + translation[0]
    translation[1] = transformation.transform.translation.y + translation[1]
    translation[2] = transformation.transform.translation.z + translation[2]

    transformed_coordinate[0] = transformed_coordinate[0] + translation[0]
    transformed_coordinate[1] = transformed_coordinate[1] + translation[1]
    transformed_coordinate[2] = transformed_coordinate[2] + translation[2]

    for object in self.detected_objects:
      color = object["color"]
      pink_on_top = object["pink_on_top"]
      
      #Check if a marker of the same colour has already been added.
      if (color == self.BLUE and self.BLUE_PINK == False):
        self.BLUE_PINK = True
        self.generate_marker(transformed_coordinate, color, pink_on_top)
      elif (color == self.GREEN and pink_on_top == True and self.PINK_GREEN == False):
        self.PINK_GREEN = True
        self.generate_marker(transformed_coordinate, color, pink_on_top)
      elif (color == self.GREEN and pink_on_top == False and self.GREEN_PINK == False):
        self.GREEN_PINK = True
        self.generate_marker(transformed_coordinate, color, pink_on_top)
      elif (color == self.YELLOW and self.YELLOW_PINK == False):
        self.YELLOW_PINK = True
        self.generate_marker(transformed_coordinate, color, pink_on_top)
  
  # Adds a new marker on cartographer.
  def generate_marker(self, coordinate, color, pink_on_top):

    color_txt = "BLUE"
    if (color == self.GREEN):
      color_txt = "GREEN"
    elif (color == self.YELLOW):
      color_txt = "YELLOW"
    
    print(f"PUBlISHING {color_txt} MARKER. PINK ON TOP: {pink_on_top}. COORDINATES: {coordinate}")

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

    # Generate the top half of the marker
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.id = self.marker_counter + 1
    self.marker_counter = self.marker_counter + 1
    marker.type = marker.CYLINDER
    marker.action = marker.ADD
    marker.pose.position.x = float(coordinate[0])
    marker.pose.position.y = float(coordinate[1])
    marker.pose.position.z = float(coordinate[2]) + 0.3
    marker.scale.x = 0.17 
    marker.scale.y = 0.17 
    marker.scale.z = 0.23 
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.color.a = 1.0
    marker.color.r = top_rgb[0] / 255.0
    marker.color.g = top_rgb[1] / 255.0
    marker.color.b = top_rgb[2] / 255.0
    self.marker_list.markers.append(marker)

  # Generate the bottom half of the marker
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.id = self.marker_counter + 1
    self.marker_counter = self.marker_counter + 1
    marker.type = marker.CYLINDER
    marker.action = marker.ADD
    marker.pose.position.x = float(coordinate[0])
    marker.pose.position.y = float(coordinate[1])
    marker.pose.position.z = float(coordinate[2]) + 0.1
    marker.scale.x = 0.17 
    marker.scale.y = 0.17 
    marker.scale.z = 0.23 
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.color.a = 1.0
    marker.color.r = bot_rgb[0] / 255.0
    marker.color.g = bot_rgb[1] / 255.0
    marker.color.b = bot_rgb[2] / 255.0
    self.marker_list.markers.append(marker)

    self.marker_pub.publish(self.marker_list)   

  #######################################################################################################################
  # NOTE: WE DID NOT USE THE FUNCTIONS BELOW IN OUR DEMONSTRATION. THESE FUNCTIONS ARE FOR MARKER LOCALISATION #
  #######################################################################################################################
  
  # def showDistance(self, marker_height):
  #   if (marker_height >= 15 and marker_height <= 65):
  #     print(f'Distance to marker is {self.distMarkerToCamera(marker_height)}')
  #   else:
  #     print(f"Too close or far to marker, Distance won't be accurate. Distance is less than 200mm or greater than 500mm")
      
  # def distMarkerToCamera(self, marker_height):
  #   # Quadratic regression equation distance = a*x^2 + b*x + c
  #   a = 0.000561
  #   b = -0.0622
  #   c = 2.22
  #   distance = (a*(marker_height**2)) - (b*marker_height) + c 
  #   # self.showDistance(marker_height)
  #   return distance # return 1 for debugging

  # def calcDistanceAndPublish(self):
  #   if len(self.detected_objects) == 0: 
  #     return 

  #   robot_currX = 0;
  #   robot_currY = 0;
  #   robot_currZ = 0;
    
  #   object = self.detected_objects[0]
  #   object_height = object["h"]
  #   object_width = object["w"]
  #   object_fromLeft = object["x"]
  #   object_fromTop = object["y"]
  #   object_area = object["area"]
  #   object_centroid = object["centroid"]
  #   object_color = object["color"]
  #   pink_on_top = object["pink_on_top"]

  #   dist_objectToMarker = self.distMarkerToCamera(object_height)
  #   object_realHeight = 0.2 # change to 0.2 if needed
  #   # object_pixelHeight = object_height * 0.2646
  #   similarTriangleRatio = object_realHeight / object_height

  #   # real_distance = object_realHeight / object_pixelHeight * dist_objectToMarker
  #   real_distance = dist_objectToMarker # * similarTriangleRatio
  #   rel_xToCenter = object_fromLeft - self.image_size[0] / 2.0
  #   # real_xToCenter = object_realHeight / object_pixelHeight * rel_xToCenter
    
  #   # cylinder_absX = robot_currX + real_xToCenter
  #   cylinder_absX = rel_xToCenter * similarTriangleRatio
  #   cylinder_absY = math.sqrt(math.pow(real_distance, 2) - math.pow(cylinder_absX, 2))      
  #   cylinder_absZ = 0
    
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
