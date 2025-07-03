import socket
import cv2
import numpy as np
from threading import Thread


class VideoStreamReceiver:
    def __init__(self, host='0.0.0.0', port=8000):
        #log a msg
        print("Initializing VideoStreamReceiver...")
        self.host = host
        self.port = port
        self.running = False
        self.socket = None
        self.thread = None
        self.current_frame = None
        self.current_depth = None
        self.frame_lock = True
        self.rgb_frame = None
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
                print("Waiting for frame header...")
                header = conn.recv(4)
                if len(header) != 4:
                    break
                    
                frame_len = int.from_bytes(header, byteorder='little')
                
                # Read frame data
                print(f"Receiving frame of length: {frame_len}")
                chunks = []
                bytes_received = 0
                while bytes_received < frame_len:
                    print(f"Receiving chunk: {bytes_received}/{frame_len}")
                    chunk = conn.recv(min(frame_len - bytes_received, 4096))
                    if not chunk:
                        break
                    chunks.append(chunk)
                    bytes_received += len(chunk)
                
                # Process complete frame
                if bytes_received == frame_len:
                    print(f"Received complete frame of length: {bytes_received}")
                    frame_data = b''.join(chunks)
                    image = np.frombuffer(frame_data, dtype=np.uint8)
                    frame = cv2.imdecode(image, cv2.IMREAD_COLOR)
                    
                    if frame is not None:
                        self.current_frame = frame
                        self.rgb_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)

        finally:
            conn.close()
            print("Connection closed")

    def show_video(self):
        while self.running:
            if self.current_frame is not None:

                cv2.imshow('ESP32-CAM Stream', self.current_frame)
                
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.stop()
                break
        cv2.destroyAllWindows()

    def stream(self):
        try:
            receiver.show_video()
        except KeyboardInterrupt:
            receiver.stop()


if __name__ == "__main__":
    # First-time setup instructions
    print("Note: This code requires:")
    print("- torch and torchvision installed")
    print("- OpenCV-contrib installed")
    print("- Internet connection for model download")
    
    receiver = VideoStreamReceiver()
    receiver.start()
    receiver.stream()