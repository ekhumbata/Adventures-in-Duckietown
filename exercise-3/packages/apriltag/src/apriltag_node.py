#!/usr/bin/env python3
import apriltag
import argparse
import cv2

import os
import rospy
from duckietown.dtros import DTROS, NodeType
import numpy as np

#from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage


class apriltag_node(DTROS):

    def __init__(self, node_name):
        super(apriltag_node, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        self.node_name = node_name
        print("==========================")
        print("successfully compiled")
        print("==========================")

        self.grey_img = None
        self.run = True

        # subscribers
        img_topic = f"""/{os.environ['VEHICLE_NAME']}/camera_node/image/compressed"""
        self.img_sub = rospy.Subscriber(img_topic, CompressedImage, self.cb_img, queue_size = 1)

        #publishers
        self.pub = rospy.Publisher('/grey_img/compressed', CompressedImage)

    def cb_img(self, msg):
        data_arr = np.fromstring(data.data, np.uint8)
        grey_img = cv2.imdecode(data_arr, cv2.COLOR_BGR2GRAY)
        self.grey_img = grey_img

        self.detect_tag(grey_img)

    def img_pub(self):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', self.grey_img)[1]).tostring()

        self.pub.publish(msg)

    def detect_tag(self, img)
        # construct the argument parser and parse the arguments
        # ap = argparse.ArgumentParser()
        # ap.add_argument("-i", "--image", required=True,
        #     help="path to input image containing AprilTag")
        # args = vars(ap.parse_args())
        gray = img
        # define the AprilTags detector options and then detect the AprilTags
        # in the input image
        print("[INFO] detecting AprilTags...")
        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        results = detector.detect(gray)
        print("[INFO] {} total AprilTags detected".format(len(results)))
        for r in results:
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            # draw the bounding box of the AprilTag detection
            cv2.line(image, ptA, ptB, (0, 255, 0), 2)
            cv2.line(image, ptB, ptC, (0, 255, 0), 2)
            cv2.line(image, ptC, ptD, (0, 255, 0), 2)
            cv2.line(image, ptD, ptA, (0, 255, 0), 2)
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
            # draw the tag family on the image
            tagFamily = r.tag_family.decode("utf-8")
            cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print("[INFO] tag family: {}".format(tagFamily))
        # show the output image after AprilTag detection
        cv2.imshow("Image", image)
        cv2.waitKey(0)

if __name__ == '__main__':
    # create the node
    node = apriltag_detector_node(node_name='april_tag_detector')

    rate = rospy.Rate(100) # 1Hz
    while not rospy.is_shutdown() and node.run:
        node.img_pub()
        rate.sleep()
    
