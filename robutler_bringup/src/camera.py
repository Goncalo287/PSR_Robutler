#! /usr/bin/python3


import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def detectSpheres(img, color):
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    contours_red = []

    # Threshold the image to get only desired color
    if color == "sphere_violet":
        lower_color = np.array([125, 50, 50])
        upper_color = np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower_color, upper_color)
    elif color == "sphere_red":
        lower_color = np.array([0, 50, 50])
        upper_color = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_color, upper_color)
    else:
        lower_purple = np.array([125, 50, 50])
        upper_purple = np.array([140, 255, 255])
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask_purple = cv2.inRange(hsv, lower_purple, upper_purple)
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        mask = cv2.bitwise_or(mask_purple, mask_red)
        kernel_red = np.ones((5, 5), np.uint8)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel_red)
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        

    

    # Smooth out the mask using morphological operations
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find the contours in the thresholded image
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Draw a circle around the detected spheres
    spheres_detected = False
    sphere_count = 0
    red_sphere_count = 0
    purple_sphere_count = 0
    for contour in contours_red:
        if cv2.contourArea(contour) < 200:
            continue

        (x, y), radius = cv2.minEnclosingCircle(contour)
        center = (int(x), int(y))
        radius = int(radius)
        red_sphere_count += 1
        
    for contour in contours:
        if cv2.contourArea(contour) < 200:
            continue

        (x, y), radius = cv2.minEnclosingCircle(contour)
        center = (int(x), int(y))
        radius = int(radius)

        if color == "sphere_violet":
            color = (0, 255, 0)
            text = "Here's a violet sphere"
        elif color == "sphere_red":
            color = (0, 255, 0)
            text = "Here's a red sphere"
        else:
            sphere_count += 1
            purple_sphere_count = sphere_count-red_sphere_count
            color = (255, 0, 0)
            if sphere_count==0:
                text = "No spheres"
            else:
                if red_sphere_count == 1 and purple_sphere_count == 0:
                    text = "Here's a red sphere "
                elif red_sphere_count == 0 and purple_sphere_count == 1:
                    text = "Here's a purple sphere "
                elif red_sphere_count > 0 and purple_sphere_count == 0:
                    text = "Here are " + str(red_sphere_count) + " red spheres "
                elif red_sphere_count == 0 and purple_sphere_count > 0:
                    text = "Here are " + str(purple_sphere_count) + " purple spheres "
                elif red_sphere_count == 1 and purple_sphere_count == 1:
                    text = "Here is a red and a purple sphere "
                else:
                    text = "Here is " + str(red_sphere_count) + " red and " + str(purple_sphere_count) + " violet spheres"
                

        img = cv2.circle(img, center, radius, color, 2)
        spheres_detected = True

    if spheres_detected:
        # Display image with message
        cv2.putText(img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    return img, spheres_detected


def image_callback(msg):
    # Convert image to opencv
    try:
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print('Failed to convert image:', e)
        return

    # Detect spheres
    img, sucess = detectSpheres(img,color= None)
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