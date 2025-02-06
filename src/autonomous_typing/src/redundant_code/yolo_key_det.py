import cv2
from ultralytics import YOLO
import os

# Load YOLOv8 model
model = YOLO('trained_yolov8n.pt')

# Define class names
class_names = [
    'caret', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', 'ss', 'accent', 'del', 'tab', 'q', 'w', 'e', 'r', 't', 
    'z', 'u', 'i', 'o', 'p', 'ue', 'plus', 'enter', 'shift-lock', 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', 'oe', 
    'ae', 'hash', 'shift-left', 'less', 'y', 'x', 'c', 'v', 'b', 'n', 'm', 'comma', 'point', 'minus', 'shift-right', 
    'strg-left', 'alt-left', 'space', 'altgr-right', 'strg-right', 'keyboard'
]

# Load image
# Get the directory of the current file
current_dir = os.path.dirname(__file__)

# Construct the full path to the image
img_path = os.path.join(current_dir, '26.jpg')
img_path = '26.jpg'
img = cv2.imread(img_path)

# Perform inference
results = model(img)

# Plot the results
for result in results:
    boxes = result.boxes
    for box in boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        class_id = int(box.cls[0])
        label = class_names[class_id]
        confidence = box.conf[0]

        # Draw bounding box
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Draw label and confidence
        cv2.putText(img, f'{label} {confidence:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

cv2.imshow('YOLO Detection', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
