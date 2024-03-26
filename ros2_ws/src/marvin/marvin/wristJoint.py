import rclpy
from rclpy.node import Node
from custom_interfaces.msg import PoseLandmark
import numpy as np
from std_msgs.msg import Float64  # Import Float64 message type
from .vector import vector_from_points, calculate_angle, project_vector_onto_plane

class WristAngleSubscriber(Node):

    def __init__(self):
        super().__init__('wrist_angle_subscriber')
        self.subscription = self.create_subscription(
            PoseLandmark,
            'pose_landmarks',
            self.listener_callback,
            10)
        # Change publisher to publish Float64 messages on the shoulder_joint topic
        self.wrist_angle_publisher = self.create_publisher(Float64, 'wrist', 10)

    def listener_callback(self, msg):
        labels = msg.label
        points = msg.point

        # Use hardcoded indices for shoulders, elbows, and wrists
        left_index_idx = labels.index('left_index')
        right_index_idx = labels.index('right_index')
        left_elbow_idx = labels.index('left_elbow')
        right_elbow_idx = labels.index('right_elbow')
        left_wrist_idx = labels.index('left_wrist')
        right_wrist_idx = labels.index('right_wrist')
        left_shoulder_idx = labels.index('left_shoulder')
        right_shoulder_idx = labels.index('right_shoulder')

        # Calculate vectors for arms
        left_forearm = vector_from_points(points[left_wrist_idx], points[left_elbow_idx])
        right_forearm = vector_from_points(points[right_wrist_idx], points[right_elbow_idx])
        left_wrist_arm = vector_from_points(points[left_index_idx], points[left_wrist_idx])
        right_wrist_arm = vector_from_points(points[right_index_idx], points[right_wrist_idx])
        shoulder_to_shoulder = vector_from_points(points[left_shoulder_idx], points[right_shoulder_idx])

        # project vector
        projected_left_wrist_arm = project_vector_onto_plane(left_wrist_arm, shoulder_to_shoulder)

        # Calculate angles
        left_wrist_angle = calculate_angle(left_wrist_arm, left_forearm)
        right_wrist_angle = calculate_angle(right_wrist_arm, right_forearm)
        print(left_wrist_angle)

        # Convert angle to radians and publish
        left_wrist_angle_msg = Float64()
        left_wrist_angle_msg.data = left_wrist_angle
        self.wrist_angle_publisher.publish(left_wrist_angle_msg)

def main(args=None):
    rclpy.init(args=args)
    wrist_angle_subscriber = WristAngleSubscriber()
    rclpy.spin(wrist_angle_subscriber)
    wrist_angle_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()