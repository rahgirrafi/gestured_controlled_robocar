import socket
import cv2
import numpy as np
from threading import Thread
import yaml

class CalibrationVisualizer:
    def __init__(self, calibration_file, host='0.0.0.0', port=8000):
        # Initialize network parameters
        self.host = host
        self.port = port
        self.running = False
        self.current_frame = None
        self.frame_lock = False  # Simplified lock flag
        
        # Load calibration data
        self.calibration_data = self.load_calibration(calibration_file)
        self.camera_matrix = np.array(self.calibration_data['camera_matrix']['data']).reshape(3, 3)
        self.dist_coeffs = np.array(self.calibration_data['distortion_coefficients']['data'])
        self.image_size = (self.calibration_data['image_width'], self.calibration_data['image_height'])
        
        # Checkerboard parameters (must match calibration settings)
        self.checkerboard_size = (7, 5)  # Inner corners (8x6 board)
        self.square_size = 0.025  # 25mm

    def load_calibration(self, file_path):
        with open(file_path, 'r') as f:
            return yaml.safe_load(f)

    def start_receiver(self):
        self.running = True
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.host, self.port))
        self.socket.listen(1)
        
        self.receiver_thread = Thread(target=self._receive_frames)
        self.receiver_thread.start()

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
                
                if bytes_received == frame_len:
                    frame_data = b''.join(chunks)
                    image = np.frombuffer(frame_data, dtype=np.uint8)
                    frame = cv2.imdecode(image, cv2.IMREAD_COLOR)
                    if frame is not None:
                        self.current_frame = frame
        finally:
            conn.close()

    def visualize_undistortion(self, frame):
        undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        comparison = np.hstack((frame, undistorted))
        cv2.putText(comparison, "Original", (10,30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        cv2.putText(comparison, "Undistorted", (frame.shape[1]+10,30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        return comparison

    def draw_3d_axes(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, self.checkerboard_size, None)
        
        if not found:
            return frame
            
        # Refine corners
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        
        # Calculate pose
        objp = np.zeros((self.checkerboard_size[0]*self.checkerboard_size[1],3), np.float32)
        objp[:,:2] = np.mgrid[0:self.checkerboard_size[0],0:self.checkerboard_size[1]].T.reshape(-1,2)
        objp *= self.square_size
        
        ret, rvec, tvec = cv2.solvePnP(objp, corners, self.camera_matrix, self.dist_coeffs)
        
        # Draw axes
        axis_length = 0.05  # 5cm
        axis_points = np.float32([[0,0,0], [axis_length,0,0], [0,axis_length,0], [0,0,axis_length]])
        img_points, _ = cv2.projectPoints(axis_points, rvec, tvec, 
                                         self.camera_matrix, self.dist_coeffs)
        
        origin = tuple(map(int, img_points[0].ravel()))
        cv2.line(frame, origin, tuple(map(int, img_points[1].ravel())), (0,0,255), 3)  # X
        cv2.line(frame, origin, tuple(map(int, img_points[2].ravel())), (0,255,0), 3)  # Y
        cv2.line(frame, origin, tuple(map(int, img_points[3].ravel())), (255,0,0), 3)  # Z
        
        return frame

    def visualize_distortion_field(self):
        # Create grid pattern matching calibration resolution
        grid = np.zeros((self.image_size[1], self.image_size[0], 3), dtype=np.uint8)
        grid[::20,:] = (255,255,255)
        grid[:,::20] = (255,255,255)
        
        # Undistort grid
        undist_grid = cv2.undistort(grid, self.camera_matrix, self.dist_coeffs)
        return np.hstack((grid, undist_grid))

    def run_visualization(self):
        self.start_receiver()
        distortion_field = self.visualize_distortion_field()
        
        while self.running:
            if self.current_frame is not None:
                try:
                    # Process current frame
                    frame = self.current_frame.copy()
                    axes_frame = self.draw_3d_axes(frame)
                    undist_comparison = self.visualize_undistortion(frame)
                    
                    # Combine visualizations
                    top_row = np.hstack((axes_frame, undist_comparison))
                    bottom_row = cv2.resize(distortion_field, (top_row.shape[1], top_row.shape[0]//2))
                    combined = np.vstack((top_row, bottom_row))
                    
                    cv2.imshow('Calibration Quality (Socket Stream)', combined)
                    
                except Exception as e:
                    print(f"Processing error: {str(e)}")
                
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break
        
        self.socket.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Socket-based Calibration Visualizer')
    parser.add_argument('calibration_file', default='/home/rahgirrafi/ESP32_cam_streamer/camera_calibration.yaml' ,help='Path to calibration YAML file')
    parser.add_argument('--host', default='0.0.0.0', help='Host IP address')
    parser.add_argument('--port', type=int, default=8000, help='Port number')
    
    args = parser.parse_args()
    
    visualizer = CalibrationVisualizer(args.calibration_file, args.host, args.port)
    visualizer.run_visualization()
