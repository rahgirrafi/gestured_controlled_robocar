from ultralytics import YOLO
import cv2

# Load a pretrained YOLO11n model
model = YOLO("/media/rafi/Data/Github/gestured_controlled_robocar_/saved_models/yolo/1.pt")


# Run inference on the source
results = model(source=0, stream=True)  # generator of Results objects

for result in results:
    boxes = result.boxes  # Boxes object for bounding box outputs

    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs
    obb = result.obb  # Oriented boxes object for OBB outputs
    #show result using opencv
    cv2.imshow('result', result.plot())
    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


