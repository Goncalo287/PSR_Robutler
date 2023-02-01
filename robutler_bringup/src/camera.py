#! /usr/bin/python3


import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError


bridge = CvBridge()


def imageCallback(msg):

    # Convert image to opencv
    try:
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception:
        print('Failed to convert image')
        return



    # Processamento de imagem






    # Display image
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