
# Publishes a coordinate transformation between an ArUco marker and a camera
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
   
# Import the necessary ROS 2 libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from geometry_msgs.msg import TransformStamped # Handles TransformStamped message
from sensor_msgs.msg import Image # Image is the message type
from tf2_ros import TransformBroadcaster

 
# Import Python libraries
import cv2 # OpenCV library
import numpy as np # Import Numpy library
from scipy.spatial.transform import Rotation as R
import sys
import args
import cv2.aruco
import json
import time 

 
# The different ArUco dictionaries built into the OpenCV library. 
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

pose_estimation_list = []
 
class ArucoNode(Node):
  """
  Create an ArucoNode class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('aruco_node')
 
    # Declare parameters
    self.declare_parameter("aruco_dictionary_name", "DICT_ARUCO_ORIGINAL")
    self.declare_parameter("aruco_marker_side_length", 0.1)
    self.declare_parameter("camera_calibration_parameters_filename", "/home/kimhotvedt/kand_ws/src/opencv_tools/opencv_tools/calibration_chessboard.yaml")
    self.declare_parameter("image_topic", "/video_frames")
    self.declare_parameter("aruco_marker_name", "aruco_marker")
     
    # Read parameters
    aruco_dictionary_name = self.get_parameter("aruco_dictionary_name").get_parameter_value().string_value
    self.aruco_marker_side_length = self.get_parameter("aruco_marker_side_length").get_parameter_value().double_value
    self.camera_calibration_parameters_filename = self.get_parameter(
      "camera_calibration_parameters_filename").get_parameter_value().string_value
    image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
    self.aruco_marker_name = self.get_parameter("aruco_marker_name").get_parameter_value().string_value
    marker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)

  # Check that we have a valid ArUco marker
    if marker_dict is None:
      self.get_logger().info("[INFO] ArUCo tag of '{}' is not supported".format(
        args["type"]))
         
    # Load the camera parameters from the saved file
    cv_file = cv2.FileStorage(
      self.camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ) 
    self.mtx = cv_file.getNode('K').mat()
    self.dst = cv_file.getNode('D').mat()
    cv_file.release()
     
    # Load the ArUco dictionary
    self.get_logger().info("[INFO] detecting '{}' markers...".format(
      aruco_dictionary_name))
    self.this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
    self.this_aruco_parameters = cv2.aruco.DetectorParameters_create()
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      image_topic, 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
       
    # Initialize the transform broadcaster
    self.tfbroadcaster = TransformBroadcaster(self)
    

    # Create a publisher for TransformStamped message to Gazebo
    #self.gazebo_transform_publisher = self.create_publisher(TransformStamped, '/gazebo_transform_topic', 10)

       
    # Used to convert between ROS and OpenCV images
    self.bridge = CvBridge()

    self.start_time = time.time()

    # A list to store information about the pose estimation, later to be used in analysis of pose stability.

  # def publish_aruco_tf(self, translation_vector, rotation_quaternion):
  #   """
  #   Publishes a TF transform between the camera_depth_frame and the Aruco frame.
  #   """
  #   # Create a TransformStamped message
  #   s = TransformStamped()
  #   s.header.stamp = self.get_clock().now().to_msg()
  #   s.header.frame_id = 'camera_depth_frame'
  #   s.child_frame_id = 'Aruco'  # Specify the frame ID for the Aruco link

  #   # Store the translation (i.e. position) information
  #   s.transform.translation.x = translation_vector[0]
  #   s.transform.translation.y = translation_vector[1]
  #   s.transform.translation.z = translation_vector[2]

  #   # Store the rotation information
  #   s.transform.rotation.x = rotation_quaternion[0]
  #   s.transform.rotation.y = rotation_quaternion[1]
  #   s.transform.rotation.z = rotation_quaternion[2]
  #   s.transform.rotation.w = rotation_quaternion[3]

  #   # Send the transform
  #   self.tfbroadcaster.sendTransform(s)
    
    
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
  
    # Convert ROS Image message to OpenCV image
    current_frame = self.bridge.imgmsg_to_cv2(data)
     
    # Detect ArUco markers in the video frame
    (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(
      current_frame, self.this_aruco_dictionary, parameters=self.this_aruco_parameters,
      cameraMatrix=self.mtx, distCoeff=self.dst)
 
    # Check that at least one ArUco marker was detected
    if marker_ids is not None:
     
      # Draw a square around detected markers in the video frame
      cv2.aruco.drawDetectedMarkers(current_frame, corners, marker_ids)
 
      for i, marker_id in enumerate(marker_ids): 
        # Get the rotation and translation vectors
        rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
          corners,
          self.aruco_marker_side_length,
          self.mtx,
          self.dst)
        

         
      # The pose of the marker is with respect to the camera lens frame.
      # Imagine you are looking through the camera viewfinder, 
      # the camera lens frame's:
      # x-axis points to the right
      # y-axis points straight down towards your toes
      # z-axis points straight ahead away from your eye, out of the camera
      #for i, marker_id in enumerate(marker_ids):  

                  
        # Extract translation (tvec) and rotation (rvec) vectors
        translation = tvecs.flatten().tolist()  # Convert to list [x,y,z]
        rotation = rvecs.flatten().tolist()     # Convert to list [r1,r2,r3]
        markerID_list = marker_id.tolist()

        elapsed_time = time.time() - self.start_time
        #print(elapsed_time)
        # Get timestamp
        timestamp_sec = data.header.stamp.sec
        timestamp_nanosec = data.header.stamp.nanosec
        timestamp = timestamp_sec + timestamp_nanosec * 1e-9

        # #timestamp2 = self.get_clock().now().to_msg()
        # seconds = data.header.stamp.sec 
        # nanoseconds = data.header.stamp.nanosec
        # microseconds = seconds + (nanoseconds / 1e9)
        # epoch_time = datetime.utcfromtimestamp(0)
        # current_date_time = epoch_time + timedelta(seconds=microseconds)
        # print("Hopefully correct time: ",current_date_time)

        
        # Dictionary to store pose data
        pose_data = {
            'marker_id': markerID_list,
            'translation': translation,
            'rotation': rotation,
            'timestamp': elapsed_time
        }
        
        # Append pose_data to the list
        pose_estimation_list.append(pose_data)
        json_file_path = '/home/kimhotvedt/kand_ws/src/opencv_tools/opencv_tools/pose_est_data.json'
        with open(json_file_path, 'w') as jsonfile:
          json.dump(pose_estimation_list, jsonfile, indent=4)
              
 
        marker_frame_id = f"{self.aruco_marker_name}_{marker_id}"
        # Create the coordinate transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_depth_frame'
        #t.child_frame_id = self.aruco_marker_name # + marker_id ??? 
        t.child_frame_id = marker_frame_id
       
        # Store the translation (i.e. position) information
        t.transform.translation.x = tvecs[i][0][0]
        t.transform.translation.y = tvecs[i][0][1]
        t.transform.translation.z = tvecs[i][0][2]

        #print('Vector to JSON ', translation)
        #print("Stored translation: ", t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)
        #print("Attemped time: ", timestamp)
        print("ROS2 time stamp: ", t.header.stamp)
        # Store the rotation information
        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
        r = R.from_matrix(rotation_matrix[0:3, 0:3])
        quat = r.as_quat()   
         
        # Quaternion format     
        t.transform.rotation.x = quat[0] 
        t.transform.rotation.y = quat[1] 
        t.transform.rotation.z = quat[2] 
        t.transform.rotation.w = quat[3] 

        translation_vector = tvecs[i][0]
        rotation_quaternion = quat
 
        # Send the transform
        self.tfbroadcaster.sendTransform(t) #Rviz
        #self.publish_aruco_tf(translation_vector, rotation_quaternion)
        #self.gazebo_transform_publisher.publish(t) #Gazebo
        #lookuptransfrom, convertera till pose
                   
        # Draw the axes on the marker
        cv2.aruco.drawAxis(current_frame, self.mtx, self.dst, rvecs[i], tvecs[i], 0.15) 
  

  # Create a topic send to gazebo, make sure you can listen to the topic (the pose), send a message.
               
    # Display image
    cv2.imshow("camera", current_frame)
     
    cv2.waitKey(1)

  # def publish_transform(self, t):
  #   # Publish the transform to Gazebo's TF topic
  #   self.tfbroadcaster.sendTransform(t)

  # def save_pose_data_to_json(self, file_path):
  #     """
  #     Save pose data list to a JSON file.
  #     """
  #     try:
  #         pose_estimation = []
  #         for pose in pose_estimation_list:
              
  #             translation_list = pose['translation'].tolist()
  #             rotation_list = pose['rotation'].tolist()

  #             non_ndarray = {
  #                 'marker_id': pose['marker_id'],
  #                 'translation': translation_list,
  #                 'rotation': rotation_list,
  #                 'timestamp': pose['timestamp']
  #             }
  #             pose_estimation.append(non_ndarray)
  #         with open(file_path, 'w') as jsonfile:
  #             json.dump(pose_estimation, jsonfile, indent=4)
  #         self.get_logger().info(f"Saved pose data to {file_path}")
  #     except Exception as e:
  #         self.get_logger().error(f"Error saving JSON file: {str(e)}")
          
   
def main(args=None):
   
  # Initialize the rclpy library
  rclpy.init(args=args)
   
  # Create the node
  aruco_node = ArucoNode()
   
  
  # Spin the node so the callback function is called.
  rclpy.spin(aruco_node)

  
  #aruco_node.save_pose_data_to_json(json_file_path)
  #aruco_node.save_pose_data_to_json(json_file_path)
  #print(f"Saving JSON file to: {json_file_path}") # 

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  aruco_node.destroy_node()
   
  # Shutdown the ROS client library for Python
  rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     aruco_node = ArucoNode()

#     try:
#         # Spin the node until shutdown is requested
#         rclpy.spin(aruco_node)
#     finally:
#         # When shutting down, save pose data to JSON
#         #print(pose_estimation_list)
#         json_file_path = '/home/kimhotvedt/ros2_ws/src/opencv_tools/opencv_tools/pose_est_data.json'
#         aruco_node.save_pose_data_to_json(json_file_path)
#         print(f"Saved JSON file to: {json_file_path}")

#         # Destroy the node explicitly
#         aruco_node.destroy_node()
#         rclpy.shutdown()
   
if __name__ == '__main__':
  main()