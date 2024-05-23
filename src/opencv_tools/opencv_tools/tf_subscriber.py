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

    def update_model_position(self):
        try:
            transform = self.tf_buffer.lookup_transform('camera_depth_frame', 'aruco_marker_', rclpy.time.Time()) 
            model_state = ModelState()
            model_state.model_name = 'Aruco_Marker'
            model_state.pose = transform_to_pose(transform)
            self.model_state_publisher.publish(model_state)
            self.get_logger().info('Found transform')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("Failed to lookup transform")

def transform_to_pose(transform):
    pose = Pose()
    pose.position = transform.transform.translation
    pose.orientation = transform.transform.rotation
    return pose

def main(args=None):
    rclpy.init(args=args)

    tf_subscriber = TFSubscriberPlugin()
    rate = tf_subscriber.create_rate(10)  

    while rclpy.ok():
        tf_subscriber.update_model_position()
        rate.sleep()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
