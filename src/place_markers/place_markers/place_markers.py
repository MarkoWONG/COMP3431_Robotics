# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Colour segementation and connected components example added by Claude sSammut
  
# Import the necessary libraries

import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

 
class ImageSubscriber(Node):
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
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)

    # The following code is a simple example of colour segmentation
    # and connected components analysis
    
    # Convert BGR image to HSV
    hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
    
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


  def find_blue_objects(self, hsv_frame, current_frame):
    # Filter out everything that is not blue
    light_blue = (87, 62, 30)
    dark_blue = (140, 255, 255)
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
        
      #If the area of the blob is more than 150 pixels:
      if area < 150:
        print("Found a blue object")
        print(x, y, w, h, area)
        self.detected_objects.append({"x": x, "y": y, "w": w, "h": h, "area": area, "centroid": centroids[i]})
  
  def find_green_objects(self, hsv_frame, current_frame):
    # Filter out everything that is not green
    light_green = (43, 44, 30)
    dark_green = (87, 255, 255)
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
        
      #If the area of the blob is more than 150 pixels:
      if area < 150:
        print("Found a green object")
        print(x, y, w, h, area)
        self.detected_objects.append({"x": x, "y": y, "w": w, "h": h, "area": area, "centroid": centroids[i]})

  def find_yellow_objects(self, hsv_frame, current_frame):
    # Filter out everything that is not yellow
    light_yellow = (20, 84, 30)
    dark_yellow = (41, 255, 255)
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
        
      #If the area of the blob is more than 150 pixels:
      if area < 150:
        print("Found a yellow object")
        print(x, y, w, h, area)
        self.detected_objects.append({"x": x, "y": y, "w": w, "h": h, "area": area, "centroid": centroids[i]})

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
