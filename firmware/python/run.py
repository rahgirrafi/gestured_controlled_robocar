from ultralytics import YOLO
import cv2
import socket
import time

# Load custom trained model
model = YOLO("/run/media/rafi/Technical/Github/gestured_controlled_robocar/firmware/saved_models/yolo/1.pt")

# Define label mapping
SPEED_LABELS = {'1': '25%', '2': '50%', '3': '75%', '4': '100%'}
DIRECTION_LABELS = {'R': 'Right', 'S': 'Straight', 'L': 'Left'}

# Network configuration
ESP32_IP = "192.168.1.100"  # Replace with your ESP32's IP
PORT = 8080
RECONNECT_DELAY = 2  # Seconds between connection attempts
SEND_INTERVAL = 0.1  # Seconds between commands

# Initialize state variables
current_speed = "0%"
current_direction = "Stop"
last_send_time = time.time()

# Create TCP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(1.0)  # Set timeout for connection attempts

# Connection management function
def connect_to_esp():
    while True:
        try:
            print(f"Connecting to ESP32 at {ESP32_IP}:{PORT}...")
            sock.connect((ESP32_IP, PORT))
            print("Connection established!")
            return True
        except (socket.timeout, ConnectionRefusedError):
            print(f"Connection failed. Retrying in {RECONNECT_DELAY} seconds...")
            time.sleep(RECONNECT_DELAY)
        except Exception as e:
            print(f"Unexpected error: {e}")
            time.sleep(RECONNECT_DELAY)

# Establish initial connection
connect_to_esp()

# Run webcam inference
results = model(source=0, stream=True, verbose=False)  # Disable verbose output

try:
    for result in results:
        # Initialize frame detection trackers
        detected_speeds = {}
        detected_directions = {}
        
        # Process detections
        if result.boxes:
            for box in result.boxes:
                cls_id = int(box.cls.item())
                label = model.names[cls_id]
                conf = box.conf.item()
                
                # Categorize detections
                if label in SPEED_LABELS:
                    detected_speeds[label] = max(detected_speeds.get(label, 0), conf)
                elif label in DIRECTION_LABELS:
                    detected_directions[label] = max(detected_directions.get(label, 0), conf)
        
        # Update state with highest confidence detections
        if detected_speeds:
            current_speed = SPEED_LABELS[max(detected_speeds, key=detected_speeds.get)]
        if detected_directions:
            current_direction = DIRECTION_LABELS[max(detected_directions, key=detected_directions.get)]
        
        # Send commands to ESP32 at fixed interval
        current_time = time.time()
        if current_time - last_send_time >= SEND_INTERVAL:
            # Format command: "SPEED,DIRECTION"
            command = f"{current_speed},{current_direction}\n"
            
            try:
                sock.sendall(command.encode())
                print(f"Sent: {command.strip()}")
            except (BrokenPipeError, ConnectionResetError):
                print("Connection lost. Reconnecting...")
                sock.close()
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(1.0)
                connect_to_esp()
            except Exception as e:
                print(f"Send error: {e}")
            
            last_send_time = current_time
        
        # Prepare status display
        status = f"Speed: {current_speed}, Direction: {current_direction}"
        
        # Visualize results
        frame = result.plot()  # Get annotated frame
        cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('Gesture Control', frame)
        
        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Program interrupted by user")

finally:
    # Cleanup
    sock.close()
    cv2.destroyAllWindows()
    print("Resources released")