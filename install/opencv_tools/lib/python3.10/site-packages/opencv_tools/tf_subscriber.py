import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
import tf2_ros

class TFSubscriberPlugin(Node):
    def __init__(self):
        super().__init__('gazebo_tf_subscriber')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.model_state_publisher = self.create_publisher(ModelState, '/gazebo/set_model_state', 1)
        self.print_all_frames()

    def print_all_frames(self):
        # Print all available frames in the transform buffer
        frames_str = self.tf_buffer.all_frames_as_string()
        self.get_logger().info(frames_str)  # Adjust the logging function here
        

    def update_model_position(self):
        try:
            transform = self.tf_buffer.lookup_transform('camera_depth_frame', 'Aruco', rclpy.time.Time()) # 'camera_depth_frame', 'marker_frame_id'
            model_state = ModelState()
            model_state.model_name = 'Aruco_Marker'  # Replace with the name of your robot model in Gazebo
            model_state.pose = transform.transform
            self.model_state_publisher.publish(model_state)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("Failed to lookup transform")

def main(args=None):
    rclpy.init(args=args)

    tf_subscriber = TFSubscriberPlugin()
    rate = tf_subscriber.create_rate(10)  # Adjust the rate as needed

    while rclpy.ok():
        tf_subscriber.update_model_position()
        rate.sleep()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
