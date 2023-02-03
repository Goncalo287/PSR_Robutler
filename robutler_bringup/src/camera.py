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

    # Keyboard inputs
    key = cv2.waitKey(10) & 0xFF

    if key == ord('q') or key == 27:    # Q or ESC to exit
        pass

def main():
    # Image subscriber
    rospy.init_node('camera', anonymous=True)
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)

    # Spin until ctrl+c
    rospy.spin()

if __name__ == '__main__':
    main()