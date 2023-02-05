#! /usr/bin/python3


import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def detect_spheres(img):
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define the range of purple color in HSV color space
    lower_purple = np.array([125, 50, 50])
    upper_purple = np.array([135, 255, 255])

    # Threshold the image to get only purple color
    mask_purple = cv2.inRange(hsv, lower_purple, upper_purple)

    # Define the range of red color in HSV color space
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])

    # Threshold the image to get only red color
    mask_red = cv2.inRange(hsv, lower_red, upper_red)

    # Combine the two masks
    mask = cv2.bitwise_or(mask_purple, mask_red)

    # Smooth out the mask using morphological operations
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find the contours in the thresholded image
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Draw a circle around the detected spheres
    spheres_detected = False
    for contour in contours:
        if cv2.contourArea(contour) < 200:
            continue

        (x, y), radius = cv2.minEnclosingCircle(contour)
        center = (int(x), int(y))
        radius = int(radius)

        # Check if the sphere is purple or red
        if cv2.countNonZero(mask_purple[int(y) - int(radius):int(y) + int(radius), int(x) - int(radius):int(x) + int(radius)]) > cv2.countNonZero(mask_red[int(y) - int(radius):int(y) + int(radius), int(x) - int(radius):int(x) + int(radius)]):
            color = (0, 255, 0)
            text = "Here's a purple sphere"
        else:
            color = (0, 0, 255)
            text = "Here's a red sphere"

        img = cv2.circle(img, center, radius, color, 2)
        spheres_detected = True

    if spheres_detected:
        # Display image with message
        cv2.putText(img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    return img


def image_callback(msg):
    # Convert image to opencv
    try:
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print('Failed to convert image:', e)
        return

    # Detect spheres
    img = detect_spheres(img)
    cv2.imshow("Robutler's Camera", img)

    # Keyboard inputs
    key = cv2.waitKey(10) & 0xFF

    if key == ord('q') or key == 27:    # Q or ESC to exit
        pass


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