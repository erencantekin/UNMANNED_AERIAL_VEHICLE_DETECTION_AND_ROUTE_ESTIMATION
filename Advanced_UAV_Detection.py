import cv2
import torch
import numpy as np
from PIL import Image

# Load YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'custom', path='C:/Users/lenovo/Desktop/github/Advanced-Aerial-Drone-Detection-System/best.pt', source='github')


# Define the classes you want to detect
classes = ['Drone']

# Initialize the rectangle coordinates
rectangle_coords = [(50, 50), (250, 50), (250, 250), (50, 250)]
rectangle_drag = False
drag_corner = -1



class UAVDetector:
    
    
    
    def detect(self, frame):
    
        # Convert the frame to a format that YOLOv5 can process
        img = Image.fromarray(frame[...,::-1])
    
        # Run inference on the frame
        results = model(img, size=640)
        x1 : float = 0
        y1 : float = 0
        x2 : float = 0
        y2 : float = 0
        text_coords = '(0, 0)'
        # Process the results and draw bounding boxes on the frame
        for result in results.xyxy[0]:
            x1, y1, x2, y2, conf, cls = result.tolist()
            if conf > 0.5 and classes[int(cls)] in classes:
                # Draw the bounding box
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
    
                # Display the confidence score above the box
                text_conf = "{:.2f}%".format(conf * 100)
                cv2.putText(frame, text_conf, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                #print("TEXT CONF", text_conf)
                # Display the bounding box coordinates below the box
                text_coords = "({}, {})".format(int((x1 + x2) / 2), int(y2))
                cv2.putText(frame, text_coords, (int(x1), int(y2) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                #print("TEXT COORD", text_coords)
    
    
        return (int(x1), int(y1), int(x2), int(y2), eval(text_coords))
    