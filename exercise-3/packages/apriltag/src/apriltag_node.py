#!/usr/bin/env python3
import dt_apriltags
import argparse
import cv2

import os
import rospy
from duckietown.dtros import DTROS, NodeType
import numpy as np

#from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from duckietown_msgs.srv import ChangePattern
from duckietown_msgs.msg import LEDPattern
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_msgs.msg import ColorRGBA

class apriltag_node(DTROS):

    def __init__(self, node_name):
        super(apriltag_node, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        self.node_name = node_name

        self.grey_img = np.array([])
        self.run = True
        self.prev_img = None
        #no detection
        self.curr_col = "WHITE"
        self.sign_to_col = {
            #blue = T-intersection
            153: "BLUE",
            58: "BLUE", 
            133: "BLUE", 
            62: "BLUE", 
            #red = stop sign
            162: "RED",
            169: "RED",
            #green = UofA Tag
            201: "GREEN",
            200: "GREEN",
            94: "GREEN",
            93: "GREEN"
        }

        self.pub_led = rospy.Publisher("/" + os.environ['VEHICLE_NAME'] + "/led_emitter_node/led_pattern", LEDPattern, queue_size=10)


        # subscribers
        img_topic = f"""/{os.environ['VEHICLE_NAME']}/camera_node/image/compressed"""
        self.img_sub = rospy.Subscriber(img_topic, CompressedImage, self.cb_img, queue_size = 1)

        # publishers
        self.pub = rospy.Publisher('/grey_img/compressed', CompressedImage)

        # services 
        # led_topic = "/%s" % os.environ['VEHICLE_NAME'] + "/led_emitter_node/set_pattern"
        # os.system(f"dts duckiebot demo --demo_name led_emitter_node --duckiebot_name {os.environ['VEHICLE_NAME']} --package_name led_emitter --image duckietown/dt-core:daffy-arm64v8 && echo RAN LIGHTING DEMO")
        # rospy.wait_for_service(led_topic)
        # self.change_led = rospy.ServiceProxy(led_topic, ChangePattern)

        self.publishLEDs(1.0, 0.0, 0.0)

    def cb_img(self, msg):
        data_arr = np.fromstring(msg.data, np.uint8)
        col_img = cv2.imdecode(data_arr, cv2.IMREAD_COLOR)
        grey_img = cv2.cvtColor(col_img, cv2.COLOR_BGR2GRAY)
        self.grey_img = grey_img

        self.detect_tag(col_img)

    def img_pub(self):
        if self.grey_img.any():
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', self.grey_img)[1]).tostring()

            self.pub.publish(msg)

    def publishLEDs(self, red, green, blue):
        set_led_cmd = LEDPattern()

        for i in range(5):
            rgba = ColorRGBA()
            rgba.r = red
            rgba.g = green
            rgba.b = blue
            rgba.a = 1.0
            set_led_cmd.rgb_vals.append(rgba)

        self.pub_led.publish(set_led_cmd)

    def change_led_to(self, new_col):
        # print("col:", new_col)

        if(new_col == "RED"):
            self.publishLEDs(1.0, 0.0, 0.0)

        elif(new_col == "GREEN"):
            self.publishLEDs(0.0, 1.0, 0.0)

        elif(new_col == "BLUE"):
            self.publishLEDs(0.0, 0.0, 1.0)

        else:
            self.publishLEDs(1.0, 1.0, 1.0)

        # col = String()
        # col.data = new_col
        # self.change_led(col)

    def detect_tag(self, img):
        # convert the img to greyscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # define the AprilTags detector options and then detect the AprilTags in the input image
        detector = dt_apriltags.Detector()
        results = detector.detect(gray)
        print("[INFO] {} total AprilTags detected".format(len(results)))

        if len(results) == 0:
            #self.change_led_to("WHITE")
            self.curr_col = "WHITE"
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()
            self.pub.publish(msg)
            return

        closest_col = None 
        closest = 0

        for r in results:
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = r.corners
            diff = abs(ptB[0] - ptA[0])
            print(diff)
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))

            # draw the bounding box of the AprilTag detection
            line_col = (125, 125, 0)
            cv2.line(img, ptA, ptB, line_col, 2)
            cv2.line(img, ptB, ptC, line_col, 2)
            cv2.line(img, ptC, ptD, line_col, 2)
            cv2.line(img, ptD, ptA, line_col, 2)

            # get the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            #cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)

            # draw the tag id on the image
            txt_col = (25, 25, 200)
            tag_id = r.tag_id
            cv2.putText(img, str(tag_id), (cX - 9, cY + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, txt_col, 2)
            print("[INFO] tag id: {}".format(tag_id))

            # if multiple seen, set col to the closest tag
            if diff > closest:
                closest = diff
                closest_col = self.sign_to_col[tag_id]
        
        # change the led based on the tag id
        #self.change_led_to(closest_col)
        self.curr_col = closest_col

        # publish the image with the tag id and box to a custom topic
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()
        self.pub.publish(msg)

if __name__ == '__main__':
    # create the node
    node = apriltag_node(node_name='april_tag_detector')

    rate = rospy.Rate(10) # 1Hz
    while not rospy.is_shutdown() and node.run:
        #node.img_pub()
        node.change_led_to(node.curr_col)
        rate.sleep()
    
