from ultralytics import YOLO
import cv2
import socket
import time
import threading

# Load custom trained model
model = YOLO("/media/rafi/Data/Github/gestured_controlled_robocar_/saved_models/yolo/1.pt")

# Define label mapping
SPEED_LABELS = {'1': '25%', '2': '50%', '3': '75%', '4': '100%'}
DIRECTION_LABELS = {'R': 'Right', 'S': 'Straight', 'L': 'Left'}

# Network configuration
HOST_IP = "0.0.0.0"  # Listen on all interfaces
PORT = 8080
SEND_INTERVAL = 0.1  # Seconds between commands

# Initialize state variables
current_speed = "0%"
current_direction = "Stop"
clients = []
client_lock = threading.Lock()
last_send_time = time.time()

# Create TCP server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind((HOST_IP, PORT))
server_socket.listen(5)
print(f"Server listening on {HOST_IP}:{PORT}")

# Client handling thread
def handle_client(client_socket):
    print(f"New client connected: {client_socket.getpeername()}")
    try:
        while True:
            # Check if client is still connected
            try:
                client_socket.send(b"ping")
            except:
                break
            time.sleep(1)
    except:
        pass
    
    with client_lock:
        if client_socket in clients:
            clients.remove(client_socket)
    print(f"Client disconnected: {client_socket.getpeername()}")
    client_socket.close()

# Accept connections in background
def accept_connections():
    while True:
        client_socket, addr = server_socket.accept()
        with client_lock:
            clients.append(client_socket)
        threading.Thread(target=handle_client, args=(client_socket,), daemon=True).start()

# Start connection acceptor
threading.Thread(target=accept_connections, daemon=True).start()

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
        
        # Send commands to all connected clients at fixed interval
        current_time = time.time()
        if current_time - last_send_time >= SEND_INTERVAL:
            # Format command: "SPEED,DIRECTION"
            command = f"{current_speed},{current_direction}\n"
            
            with client_lock:
                for client_socket in clients.copy():
                    try:
                        client_socket.sendall(command.encode())
                        print(f"Sent to {client_socket.getpeername()}: {command.strip()}")
                    except:
                        print(f"Failed to send to {client_socket.getpeername()}")
                        clients.remove(client_socket)
                        client_socket.close()
            
            last_send_time = current_time
        
        # Prepare status display
        status = f"Speed: {current_speed}, Direction: {current_direction}"
        status += f" | Clients: {len(clients)}"
        
        # Visualize results
        frame = result.plot()  # Get annotated frame
        cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('Gesture Control Server', frame)
        
        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Program interrupted by user")

finally:
    # Cleanup
    server_socket.close()
    with client_lock:
        for client_socket in clients:
            client_socket.close()
    cv2.destroyAllWindows()
    print("Server shutdown complete")