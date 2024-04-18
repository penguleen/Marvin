import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import PoseLandmark
from geometry_msgs.msg import Point
import numpy as np
from move_group_python_interface_tutorial import MoveGroupPythonInterfaceTutorial as tut


def vec_length(v: np.array):
    return np.sqrt(sum(i ** 2 for i in v))

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

def look_at(eye: np.array, target: np.array):
    axis_z = normalize((eye - target))
    if vec_length(axis_z) == 0:
            axis_z = np.array((0, -1, 0))

    axis_x = np.cross(np.array((0, 0, 1)), axis_z)
    if vec_length(axis_x) == 0:
            axis_x = np.array((1, 0, 0))

    axis_y = np.cross(axis_z, axis_x)
    rot_matrix = np.matrix([axis_x, axis_y, axis_z]).transpose()
    return rot_matrix

def rotation_matrix_to_quaternion(rot_matrix):
    """
    Convert a 3x3 rotation matrix to a unit quaternion.
    """
    # Extract components from rotation matrix
    trace = np.trace(rot_matrix)
    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2
        qw = 0.25 * S
        qx = (rot_matrix[2, 1] - rot_matrix[1, 2]) / S
        qy = (rot_matrix[0, 2] - rot_matrix[2, 0]) / S
        qz = (rot_matrix[1, 0] - rot_matrix[0, 1]) / S
    elif (rot_matrix[0, 0] > rot_matrix[1, 1]) and (rot_matrix[0, 0] > rot_matrix[2, 2]):
        S = np.sqrt(1.0 + rot_matrix[0, 0] - rot_matrix[1, 1] - rot_matrix[2, 2]) * 2
        qw = (rot_matrix[2, 1] - rot_matrix[1, 2]) / S
        qx = 0.25 * S
        qy = (rot_matrix[0, 1] + rot_matrix[1, 0]) / S
        qz = (rot_matrix[0, 2] + rot_matrix[2, 0]) / S
    elif rot_matrix[1, 1] > rot_matrix[2, 2]:
        S = np.sqrt(1.0 + rot_matrix[1, 1] - rot_matrix[0, 0] - rot_matrix[2, 2]) * 2
        qw = (rot_matrix[0, 2] - rot_matrix[2, 0]) / S
        qx = (rot_matrix[0, 1] + rot_matrix[1, 0]) / S
        qy = 0.25 * S
        qz = (rot_matrix[1, 2] + rot_matrix[2, 1]) / S
    else:
        S = np.sqrt(1.0 + rot_matrix[2, 2] - rot_matrix[0, 0] - rot_matrix[1, 1]) * 2
        qw = (rot_matrix[1, 0] - rot_matrix[0, 1]) / S
        qx = (rot_matrix[0, 2] + rot_matrix[2, 0]) / S
        qy = (rot_matrix[1, 2] + rot_matrix[2, 1]) / S
        qz = 0.25 * S

    # Normalize quaternion
    norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    return qx, qy, qz, qw

class PoseDetectionPublisher(Node):
    def __init__(self):
        super().__init__('pose_detection_publisher')
        self.publisher_ = self.create_publisher(PoseLandmark, 'pose_landmarks', 10)

    def run_pose_detection(self):
        """Main loop for pose detection and visualization."""
        with mp.solutions.pose.Pose(
                static_image_mode=False, model_complexity=2, smooth_landmarks=True,
                enable_segmentation=False, min_detection_confidence=0.5,
                min_tracking_confidence=0.5) as pose:

            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                raise IOError("Cannot open webcam")

            while cap.isOpened():
                success, image = cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                    continue

                image = self.process_image(image, pose)
                cv2.imshow('MediaPipe Pose', image)
                if cv2.waitKey(5) & 0xFF == 27:  # Exit loop if 'ESC' is pressed
                    break

            cap.release()
            cv2.destroyAllWindows()  # Clean up OpenCV windows


    def process_image(self, image, pose):
        """Process each image frame to detect and display pose landmarks."""
        image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        image.flags.writeable = False  # Optimize performance by making image read-only
        results = pose.process(image)  # Apply pose detection

        '''Prepare the image for displaying'''
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        rotation_matrix = None
        self.quaternion = []

        if results.pose_landmarks:
            mp.solutions.drawing_utils.draw_landmarks(
                image, results.pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS,
                landmark_drawing_spec=mp.solutions.drawing_utils.DrawingSpec(color=(245, 117, 66), thickness=2,
                                                                             circle_radius=2),
                connection_drawing_spec=mp.solutions.drawing_utils.DrawingSpec(color=(245, 66, 230), thickness=2,
                                                                               circle_radius=2))
            '''Publish the landmarks to a ROS topic'''
            self.plot_landmarks_and_publish(results.pose_world_landmarks)

            # Calculate rotation matrix
            eye = np.array([0, 0, 0])  # Assuming camera origin is at (0, 0, 0)
            target = np.array([0, 0, 1])  # Assuming camera is looking towards positive z-axis
            rotation_matrix = look_at(eye, target)
            print(rotation_matrix)

                # Apply rotation matrix to landmarks
            rotated_landmarks = np.dot(rotation_matrix, np.array(
                    [[lm.x, lm.y, lm.z] for lm in results.pose_world_landmarks.landmark]).T).T
            for lm, rotated_lm in zip(results.pose_world_landmarks.landmark, rotated_landmarks):
                lm.x, lm.y, lm.z = rotated_lm

                qx, qy, qz, qw = rotation_matrix_to_quaternion(rotated_lm)
                q_stuff = qx, qy, qz, qw
                self.quaternion.append(q_stuff)
                tut.go_to_pose_goal(self, lm.x, lm.y, lm.z, qx, qy, qz, qw)

        return image

    def plot_landmarks_and_publish(self, landmarks):
        """Prepare and publish pose landmarks as ROS messages."""
        landmarks_labels = {
            11: "left_shoulder", 12: "right_shoulder", 13: "left_elbow", 14: "right_elbow",
            15: "left_wrist", 16: "right_wrist", 17: "left_pinky", 18: "right_pinky",
            19: "left_index", 20: "right_index", 21: "left_thumb", 22: "right_thumb",
            23: "left_hip", 24: "right_hip",
        }
        pose_landmark_msg = PoseLandmark()
        pose_landmark_msg.label = []
        pose_landmark_msg.point = []

        '''Fill the message with landmarks data'''
        for idx, landmark in enumerate(landmarks.landmark):
            if idx in landmarks_labels:
                label = landmarks_labels[idx]
                pose_landmark_msg.label.append(label)

                point = Point(x=landmark.x, y=landmark.y, z=landmark.z)
                pose_landmark_msg.point.append(point)
        # print(pose_landmark_msg)
        self.publisher_.publish(pose_landmark_msg) #x,y,z landmark here


def main(args=None):
    rclpy.init(args=args)
    node = PoseDetectionPublisher()
    node.run_pose_detection()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()