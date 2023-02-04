#! /usr/bin/python3


import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def detect_purple_sphere(img):
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define the range of purple color in HSV color space
    lower_purple = np.array([120, 50, 50])
    upper_purple = np.array([140, 255, 255])

    # Threshold the image to get only purple color
    mask = cv2.inRange(hsv, lower_purple, upper_purple)

    # Find the contours in the thresholded image
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Draw a circle around the detected sphere
    spheres_detected = False
    for contour in contours:
        (x, y), radius = cv2.minEnclosingCircle(contour)
        center = (int(x), int(y))
        radius = int(radius)
        img = cv2.circle(img, center, radius, (0, 255, 0), 2)
        spheres_detected = True

    if spheres_detected:
        # Display image with message
        cv2.putText(img, "Here's a purple sphere", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    return img

def image_callback(msg):
    # Convert image to opencv
    try:
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print('Failed to convert image:', e)
        return

    # Detect purple sphere
    img = detect_purple_sphere(img)
    cv2.imshow("Robutler's Camera", img)

    # Detect objects
    # img = image_classification(img)
    # cv2.imshow("Jota Tela", img)
    
    # Keyboard inputs
    key = cv2.waitKey(10) & 0xFF

    if key == ord('q') or key == 27:    # Q or ESC to exit
        pass

# def image_classification(img):
    
#     net = cv2.dnn.readNet("/home/jota/catkin_ws/src/YOLO/yolov3.weights", "/home/jota/catkin_ws/src/YOLO/yolov3.cfg")
#     classes = []
#     # with open("/home/jota/catkin_ws/src/YOLO/coco.names", "r") as f:
#     #     classes = [line.strip() for line in f.readlines()]
#     classes = open("/home/jota/catkin_ws/src/YOLO/coco.names").read().strip().split('\n')
#     #print(classes)
    
#     # determine the output layer
#     ln = net.getLayerNames()
#     ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    
#     colors = np.random.uniform(0, 255, size=(len(classes), 3))
    
#     # Loading image
#     img = cv2.resize(img, None, fx=0.4, fy=0.4)
#     height, width, channels = img.shape
    
#     # Detecting objects
#     blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
#     net.setInput(blob)
#     outs = net.forward(output_layers)
    
#     # Showing informations on the screen
#     class_ids = []
#     confidences = []
#     boxes = []
#     for out in outs:
#         for detection in out:
#             scores = detection[5:]
#             class_id = np.argmax(scores)
#             confidence = scores[class_id]
#             if confidence > 0.5:
#                 # Object detected
#                 center_x = int(detection[0] * width)
#                 center_y = int(detection[1] * height)
#                 w = int(detection[2] * width)
#                 h = int(detection[3] * height)
#                 # Rectangle coordinates
#                 x = int(center_x - w / 2)
#                 y = int(center_y - h / 2)
#                 boxes.append([x, y, w, h])
#                 confidences.append(float(confidence))
#                 class_ids.append(class_id)

#     indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
    
#     font = cv2.FONT_HERSHEY_PLAIN
#     for i in range(len(boxes)):
#         if i in indexes:
#             x, y, w, h = boxes[i]
#             label = str(classes[class_ids[i]])
#             color = colors[i]
#             cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
#             cv2.putText(img, label, (x, y + 30), font, 3, color, 3)
    
#     return img

def main():
    # Image subscriber
    rospy.init_node('camera', anonymous=True)
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)

    # Keep ros awake (10Hz refresh rate)
    while not rospy.is_shutdown():
        rospy.sleep(10)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()