#! /usr/bin/python3


import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

bridge = CvBridge()


def imageCallback(msg):
    # Convert image to opencv
    try:
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception:
        print('Failed to convert image')
        return

    # Convert image to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define lower and upper bounds for purple color in HSV color space
    lower_purple = np.array([125, 0, 0])
    upper_purple = np.array([175, 255, 255])

    # Threshold the image to get only the purple pixels
    mask = cv2.inRange(hsv, lower_purple, upper_purple)

    # Perform morphological operations to remove noise
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Perform blob detection to detect the purple sphere
    params = cv2.SimpleBlobDetector_Params()
    params.filterByColor = True
    params.blobColor = 255
    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(mask)

    # Draw a circle around the detected purple sphere
    for kp in keypoints:
        x, y = int(kp.pt[0]), int(kp.pt[1])
        size = int(kp.size)
        cv2.circle(img, (x, y), size, (0, 0, 255), 2)

    # Display a text label if a purple sphere is detected
    if len(keypoints) > 0:
        cv2.putText(img, "Here is a purple sphere", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Display the image
    cv2.imshow("Robutler's Camera", img)

    # Keyboard inputs
    key = cv2.waitKey(10) & 0xFF

    if key == ord('q') or key == 27:    # Q or ESC to exit
        pass



def main():

    # Image subscriber
    rospy.init_node('camera')
    rospy.Subscriber("/camera/rgb/image_raw", Image, imageCallback)

    # Spin until ctrl+c
    rospy.spin()


if __name__ == '__main__':
    main()