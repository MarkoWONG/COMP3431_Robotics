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
  MAX_AREA_DETECTION_THRESHOLD = 1100
  MIN_AREA_DETECTION_THRESHOLD = 200
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
    self.marker_list.markers = [];
      
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
    self.detected_objects = []
    
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)

    # Convert BGR image to HSV
    hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
    self.image_size = hsv_frame.shape
    
    #We only need to worry about blue, green and yellow because all the markers are half pink
    self.detect_objects(hsv_frame, current_frame)
    self.calcDistanceAndPublish()

    # Display camera image
    cv2.namedWindow("camera")
    cv2.imshow("camera", current_frame)
    # Display masked image
    # cv2.imshow("mask", mask)
    
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
  
  def check_existing_markers(self, point, new_color):
    for marker in self.marker_list.markers:
      if math.sqrt((point[0] - marker.pose.position.x)**2 + \
        (point[1] - marker.pose.position.y)**2) <= 0.5:
        r = marker.color.r * 255
        g = marker.color.g * 255
        b = marker.color.b * 255
        
        if new_color == self.BLUE and abs(r) <= 0.1 and abs(g - 191) <= 0.1 and abs(b - 255) < 0.1:
          return True
        elif new_color == self.YELLOW and abs(r - 255) <= 0.1 and abs(g - 234) <= 0.1 and abs(b) < 0.1:
          return True
        elif new_color == self.GREEN and abs(r) <= 0.1 and abs(g - 100) <= 0.1 and abs(b) < 0.1:
          return True
    return False

  def transform_frame(self, target, source , translation, quaternion):
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

  def calcDistanceAndPublish(self):
    if len(self.detected_objects) == 0: 
      return 

    robot_currX = 0;
    robot_currY = 0;
    robot_currZ = 0;
    
    object = self.detected_objects[0]
    object_height = object["h"]
    object_width = object["w"]
    object_fromLeft = object["x"]
    object_fromTop = object["y"]
    object_area = object["area"]
    object_centroid = object["centroid"]
    object_color = object["color"]
    pink_on_top = object["pink_on_top"]

    dist_objectToMarker = self.distMarkerToCamera(object_height)
    object_realHeight = 200 # change to 0.2 if needed
    # object_pixelHeight = object_height * 0.2646
    similarTriangleRatio = object_realHeight / object_height

    # real_distance = object_realHeight / object_pixelHeight * dist_objectToMarker
    real_distance = dist_objectToMarker # * similarTriangleRatio
    rel_xToCenter = object_fromLeft - self.image_size[0] / 2.0
    # real_xToCenter = object_realHeight / object_pixelHeight * rel_xToCenter
    
    # cylinder_absX = robot_currX + real_xToCenter
    cylinder_absX = rel_xToCenter * similarTriangleRatio
    cylinder_absY = math.sqrt(math.pow(real_distance, 2) - math.pow(cylinder_absX, 2))      
    cylinder_absZ = 0
    
    obj_in_cam = [cylinder_absY, cylinder_absX, 0]
    # obj_in_cam = [0, 0, 0]
    translation = [0,0,0]
    quaternion = [1,0,0,0]
    translation, quaternion = self.transform_frame("map", "camera_link", translation, quaternion)
    if (len(translation) != 3):
      return

    my_quater = Quaternion(quaternion[0], quaternion[1], quaternion[2],quaternion[3])
    rotation = my_quater.rotate(obj_in_cam)
    final_coordinate = [0,0,0]
    final_coordinate[0] = rotation[0] + translation[0]
    final_coordinate[1] = rotation[1] + translation[1]
    final_coordinate[2] = rotation[2] + translation[2]

    print(f"final coordinate: {final_coordinate}")
    self.add_new_point(final_coordinate, object_color, pink_on_top)
  
  def generate_marker(self, coordinate, color, pink_on_top):
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
    marker.scale.x = 0.14 # Change to 14 if needed
    marker.scale.y = 0.14 # Change to 14 if needed
    marker.scale.z = 0.2 # Change to 20 if needed
    marker.color.a = 1.0
    if pink_on_top is False:
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

    # up cylinder
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
    marker.pose.position.z = float(coordinate[2]) + 0.3
    marker.scale.x = 0.14 # Change to 14 if needed
    marker.scale.y = 0.14 # Change to 14 if needed
    marker.scale.z = 0.2 # Change to 20 if needed
    if pink_on_top is True:
      rgb = (255,192,203)
    elif color == self.YELLOW:
      rgb = (255,234,0)
    elif color == self.BLUE:
      rgb = (0,191,255)
    else:
      rgb = (0,100,0)
    marker.color.a = 1.0
    marker.color.r = rgb[0] / 255.0
    marker.color.g = rgb[1] / 255.0
    marker.color.b = rgb[2] / 255.0
    self.marker_list.markers.append(marker)

  def add_new_point(self, coordinate, color, pink_on_top):
    coordinate[0] = float(coordinate[0])
    coordinate[1] = float(coordinate[1])
    coordinate[2] = float(coordinate[2])
    if self.check_existing_markers(coordinate, color):
      return
    ## print("add_new_point2")
    self.generate_marker(coordinate, color, pink_on_top)
    print(f"-----------Current length{len(self.marker_list.markers)}----------")
    self.plot_publisher.publish(self.marker_list)

  def showDistance(self, marker_height):
    if (marker_height >= 15 and marker_height <= 65):
      print(f'Distance to marker is {self.distMarkerToCamera(marker_height)}')
    else:
      print(f"Too close or far to marker, Distance won't be accurate. Distance is less than 200mm or greater than 500mm")
      
  def distMarkerToCamera(self, marker_height):
    # Quadratic regression equation distance = a*x^2 + b*x + c
    a = 0.000561
    b = -0.0622
    c = 2.22
    distance = (a*(marker_height**2)) - (b*marker_height) + c 
    # self.showDistance(marker_height)
    return distance # return 1 for debugging


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
