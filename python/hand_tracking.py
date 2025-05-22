import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import numpy as np
import cv2
from imgTransferServer import VideoStreamReceiver

class HandLandmarkDetector:
    MARGIN = 10  # pixels
    FONT_SIZE = 1
    FONT_THICKNESS = 1
    HANDEDNESS_TEXT_COLOR = (88, 205, 54)  # vibrant green

    def __init__(self, model_path='/home/rahgirrafi/Gesture_Controlled_RoboCar/saved_models/hand_landmarker.task', num_hands=2):
        """Initialize the hand landmark detector with model path and number of hands."""
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.HandLandmarkerOptions(
            base_options=base_options,
            num_hands=num_hands
        )
        self.detector = vision.HandLandmarker.create_from_options(options)

    def detect(self, image):
        """Detect hand landmarks from a mediapipe image."""
        return self.detector.detect(image)

    def draw_landmarks(self, rgb_image, detection_result):
        """
        Draw landmarks and handedness on the image.
        Args:
            rgb_image: RGB numpy image
            detection_result: HandLandmarkerResult object
        Returns:
            Annotated RGB image as numpy array
        """
        hand_landmarks_list = detection_result.hand_landmarks
        handedness_list = detection_result.handedness
        self.annotated_image = np.copy(rgb_image)

        for idx in range(len(hand_landmarks_list)):
            hand_landmarks = hand_landmarks_list[idx]
            handedness = handedness_list[idx]

            # Draw hand landmarks
            hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
            hand_landmarks_proto.landmark.extend([
                landmark_pb2.NormalizedLandmark(x=lm.x, y=lm.y, z=lm.z) for lm in hand_landmarks
            ])
            solutions.drawing_utils.draw_landmarks(
                self.annotated_image,
                hand_landmarks_proto,
                solutions.hands.HAND_CONNECTIONS,
                solutions.drawing_styles.get_default_hand_landmarks_style(),
                solutions.drawing_styles.get_default_hand_connections_style()
            )

            # Draw handedness label
            height, width, _ = self.annotated_image.shape
            x_coords = [lm.x for lm in hand_landmarks]
            y_coords = [lm.y for lm in hand_landmarks]
            text_x = int(min(x_coords) * width)
            text_y = int(min(y_coords) * height) - self.MARGIN

            cv2.putText(self.annotated_image, f"{handedness[0].category_name}",
                        (text_x, text_y), cv2.FONT_HERSHEY_DUPLEX,
                        self.FONT_SIZE, self.HANDEDNESS_TEXT_COLOR, 
                        self.FONT_THICKNESS, cv2.LINE_AA)

        return self.annotated_image


if __name__ == "__main__":

    receiver = VideoStreamReceiver()
    receiver.start()
    
    detector = HandLandmarkDetector(model_path='/home/rahgirrafi/Gesture_Controlled_RoboCar/saved_models/hand_landmarker.task', num_hands=2)

    while receiver.running:
        if receiver.rgb_frame is not None:
            img_rgb = receiver.rgb_frame
            img_mp = mp.Image(image_format=mp.ImageFormat.SRGB, data=img_rgb)
            detection_result = detector.detect(img_mp)
            annotated_image = detector.draw_landmarks(receiver.current_frame, detection_result)
            cv2.imshow('Hand Landmarks', annotated_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            receiver.stop()
            break