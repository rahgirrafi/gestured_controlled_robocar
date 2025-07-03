import socket
import cv2
import numpy as np
from threading import Thread
import yaml

class VideoStreamReceiver:
    def __init__(self, host='0.0.0.0', port=8000):
        print("Initializing VideoStreamReceiver...")
        self.host = host
        self.port = port
        self.running = False
        self.socket = None
        self.thread = None
        self.current_frame = None
        self.frame_lock = True
        
        # Calibration parameters
        self.CHECKERBOARD_INNER_CORNERS = (8, 6)  # 8x6 board (inner corners)
        self.SQUARE_SIZE = 0.025  # 25mm in meters
        
        # Prepare 3D object points
        self.objp = np.zeros((self.CHECKERBOARD_INNER_CORNERS[0] * self.CHECKERBOARD_INNER_CORNERS[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.CHECKERBOARD_INNER_CORNERS[0], 0:self.CHECKERBOARD_INNER_CORNERS[1]].T.reshape(-1, 2)
        self.objp *= self.SQUARE_SIZE
        
        # Calibration storage
        self.objpoints = []  # 3D points in real world
        self.imgpoints = []  # 2D points in image plane
        
        print("VideoStreamReceiver initialized.")

    def start(self):
        print("Starting VideoStreamReceiver...")
        self.running = True
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.host, self.port))
        self.socket.listen(1)
        print(f"Listening on {self.host}:{self.port}")
        
        self.thread = Thread(target=self._receive_frames)
        self.thread.start()
        print("VideoStreamReceiver started.")

    def stop(self):
        print("Stopping VideoStreamReceiver...")
        self.running = False
        if self.socket:
            self.socket.close()
        if self.thread:
            self.thread.join()
        print("VideoStreamReceiver stopped.")

    def _receive_frames(self):
        conn, addr = self.socket.accept()
        print(f"Connected to {addr}")
        
        try:
            while self.running:
                # Read frame length header
                header = conn.recv(4)
                if len(header) != 4:
                    break
                    
                frame_len = int.from_bytes(header, byteorder='little')
                
                # Read frame data
                chunks = []
                bytes_received = 0
                while bytes_received < frame_len:
                    chunk = conn.recv(min(frame_len - bytes_received, 4096))
                    if not chunk:
                        break
                    chunks.append(chunk)
                    bytes_received += len(chunk)
                
                # Process complete frame
                if bytes_received == frame_len:
                    frame_data = b''.join(chunks)
                    image = np.frombuffer(frame_data, dtype=np.uint8)
                    frame = cv2.imdecode(image, cv2.IMREAD_COLOR)
                    
                    if frame is not None:
                        self.current_frame = frame

        finally:
            conn.close()
            print("Connection closed")

    def show_video(self):
        while self.running:
            if self.current_frame is not None:
                # Create a copy of the frame for processing
                frame = self.current_frame.copy()
                
                # Checkerboard detection
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                found, corners = cv2.findChessboardCorners(
                    gray, self.CHECKERBOARD_INNER_CORNERS, None
                )

                # If found, refine and draw corners
                if found:
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                    cv2.drawChessboardCorners(frame, self.CHECKERBOARD_INNER_CORNERS, corners, found)

                # Display frame
                cv2.imshow('ESP32-CAM Stream', frame)

                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.stop()
                    break
                elif key == ord('s') and found:
                    # Save calibration data
                    self.objpoints.append(self.objp)
                    self.imgpoints.append(corners)
                    print(f"Captured calibration frame ({len(self.objpoints)}/15)")
                elif key == ord('c'):
                    # Perform calibration
                    if len(self.objpoints) >= 10:
                        self.calibrate_camera()
                    else:
                        print(f"Need at least 10 frames (current: {len(self.objpoints)})")

        cv2.destroyAllWindows()

    def calibrate_camera(self):
        print("Starting camera calibration...")
        height, width = self.current_frame.shape[:2]
        
        # Perform calibration
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, (width, height), None, None
        )

        # Create ROS camera_info dictionary
        camera_info = {
            'image_width': width,
            'image_height': height,
            'camera_name': 'camera',
            'camera_matrix': {
                'rows': 3,
                'cols': 3,
                'data': camera_matrix.flatten().tolist()
            },
            'distortion_model': 'plumb_bob',
            'distortion_coefficients': {
                'rows': 1,
                'cols': 5,
                'data': dist_coeffs.flatten().tolist()
            },
            'rectification_matrix': {
                'rows': 3,
                'cols': 3,
                'data': np.eye(3).flatten().tolist()
            },
            'projection_matrix': {
                'rows': 3,
                'cols': 4,
                'data': np.hstack((camera_matrix, np.zeros((3, 1)))).flatten().tolist()
            }
        }

        # Save to YAML file
        with open('camera_calibration.yaml', 'w') as f:
            yaml.dump(camera_info, f, default_flow_style=False)

        print("Calibration successful! Data saved to camera_calibration.yaml")
        print(f"Reprojection error: {ret}")

if __name__ == "__main__":
    receiver = VideoStreamReceiver()
    receiver.start()
    
    try:
        receiver.show_video()
    except KeyboardInterrupt:
        receiver.stop()
