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
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from std_msgs.msg import String


class lane_follow_node(DTROS):

    def __init__(self, node_name):
        super(lane_follow_node, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        self.node_name = node_name

        self.pub_img = None
        self.run = True
        self.prev_img = None
        self.curr_col = "WHITE"

        

        self.lower_bound = np.array([20, 45, 25])
        self.upper_bound = np.array([35, 255, 255])

        self.detection_range = 0
        self.lower_detection = 0
        self.upper_detection = 0

        self.vel = [0.2, 0.2]

        self.drive = True
        self.speed = 0.2
        self.omega = 0
        self.size_ratio = 0.9

        self.sign_to_col = {
            153: "BLUE",
            58: "BLUE", 
            133: "BLUE", 
            62: "BLUE", 
            162: "RED",
            169: "RED",
            201: "GREEN",
            200: "GREEN",
            94: "GREEN",
            93: "GREEN"
        }


        # subscribers
        img_topic = f"""/{os.environ['VEHICLE_NAME']}/camera_node/image/compressed"""
        self.img_sub = rospy.Subscriber(img_topic, CompressedImage, self.cb_img, queue_size = 1)

        # publishers
        self.img_publisher = rospy.Publisher('/masked_image/compressed', CompressedImage)

        twist_topic = f"/{os.environ['VEHICLE_NAME']}/car_cmd_switch_node/cmd"
        self.twist_publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

        wheel_topic = f"/{os.environ['VEHICLE_NAME']}/wheels_driver_node/wheels_cmd"
        self.wheel_pub = rospy.Publisher(wheel_topic, WheelsCmdStamped, queue_size=1)

        # services 
        # led_topic = "/%s" % os.environ['VEHICLE_NAME'] + "/led_emitter_node/set_pattern"
        # os.system(f"dts duckiebot demo --demo_name led_emitter_node --duckiebot_name {os.environ['VEHICLE_NAME']} --package_name led_emitter --image duckietown/dt-core:daffy-arm64v8 && echo RAN LIGHTING DEMO")
        # rospy.wait_for_service(led_topic)
        # self.change_led = rospy.ServiceProxy(led_topic, ChangePattern)

    def cb_img(self, msg):
        data_arr = np.fromstring(msg.data, np.uint8)
        col_img = cv2.imdecode(data_arr, cv2.IMREAD_COLOR)
        hsv = cv2.cvtColor(col_img, cv2.COLOR_BGR2HSV)
        #grey_img = cv2.cvtColor(col_img[len(col_img) // 3 :], cv2.COLOR_BGR2GRAY)
        #imagemask = cv2.inRange(grey_img, self.lower_bound, self.upper_bound)
        imagemask = np.asarray(cv2.inRange(hsv[len(col_img) // 3 :], self.lower_bound, self.upper_bound))

        
        contours, hierarchy = cv2.findContours(imagemask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        largest = max(contours, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(largest)
        conts = [largest]
        if y > 200:
            contours.remove(largest)
            largest = max(contours, key = cv2.contourArea)
            conts.append(largest)
            x,y,w,h = cv2.boundingRect(largest)
        
        image = cv2.drawContours(col_img[len(col_img) // 3 :], conts, -1, (0,255,0), 3)
        #image = cv2.drawContours(col_img[len(col_img) // 3 :], [largest], 0, (0,255,0), 3)

        image = cv2.line(image, (x, y+h//2), (x + int((self.size_ratio*(y+h))), y+h), (0,255,0), 2)

        imx, imy, d = image.shape
        r = imy - y
        image = cv2.circle(image, (int((len(image[0]) // 2) - r * np.sin(self.omega)), int(imy - r * np.cos(self.omega))), 4, (0, 0, 255), -1)



        for i in range(len(image)):
            image[i][len(image[i]) // 2] = [255, 0, 0]

        self.pub_img = image

        if self.drive:
            self.pid(x, y+ h//2, len(image[i]) // 2)
        
        
        
        
        """
        self.detection_range = len(imagemask)// 2 - 3 * len(imagemask) // 8 
        self.lower_detection = len(imagemask)// 2
        self.upper_detection = 3 * len(imagemask) // 8 

        x = np.zeros(len(imagemask))
        #print(len(x[3 * len(imagemask) // 8 : len(imagemask)// 2]))
        x[self.upper_detection: self.lower_detection] = np.ones(self.detection_range) * 255



        # x = np.ones(len(grey_img)) * 255
        # x[len(x) // 2:] = np.zeros(len(x) - (len(x) // 2))
        #print(imagemask.shape, 2 * len(imagemask[0]) // 3)

        # do err correction
        #self.pid(imagemask, 2 * len(imagemask[0])// 3)

        # show error range on image
        imagemask[:, 2 * len(imagemask[0])// 3] = x

        # set publish image
        

        #self.detect_tag(col_img)

        """

    def img_pub(self):
        if self.pub_img is not None:
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', self.pub_img)[1]).tostring()

            self.img_publisher.publish(msg)

    def vel_pub(self):
        msg = WheelsCmdStamped()
        msg.vel_left = self.vel[0]
        msg.vel_right = self.vel[1]

        self.wheel_pub.publish(msg)

    def twist_pub(self):
        if self.drive:
            msg = Twist2DStamped()
            msg.v = self.speed
            msg.omega = self.omega

            self.twist_publisher.publish(msg)

    # def change_led_to(self, new_col):
    #     col = String()
    #     col.data = new_col
    #     self.change_led(col)


    def pid(self, x, y, goal):
        # proprtional part
        diff = ((x + int((self.size_ratio*y))) - goal) * 0.01
        if True:#abs(diff) <= np.pi / 10:# or diff * self.omega < 0:
            self.omega = -diff
        print(diff, self.omega)


        # for i in range(self.detection_range):
        #     #print(img[self.lower_detection - i][region])
        #     if img[self.lower_detection - i][region] == 0:
        #         left_err += 1
        #     if img[self.upper_detection + i][region]== 0:
        #         right_err += 1
        # diff = abs(left_err - right_err) * 0.001
        # if left_err > right_err:
        #     self.vel[0] += diff / 2
        #     self.vel[1] -= diff / 3
        # else:
        #     self.vel[0] -= diff / 3
        #     self.vel[1] += diff / 2
        # print(left_err, right_err, self.vel)
        # self.vel[0] = self.vel[0] * left_err
        # self.vel[1] = self.vel[1] * right_err

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
    node = lane_follow_node(node_name='custom_lane_follow')

    rate = rospy.Rate(100) # 1Hz
    while not rospy.is_shutdown() and node.run:
        node.img_pub()
        node.twist_pub()
        #node.change_led_to(node.curr_col)
        rate.sleep()
    
