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

        # hsv color values to mask
        self.lower_bound = np.array([20, 45, 25])
        self.upper_bound = np.array([35, 255, 255]) 

        self.red_upper = np.array([0, 100, 204])
        self.red_lower = np.array([20, 195, 225])
        # drive speed and ratio of goal vs distance from bot
        self.drive = False
        self.speed = 0.3
        self.omega = 0
        self.size_ratio = 0.8   #distance from centre of duckiebot to dotted line

        self.prev_time = 0
        self.prev_diff = None
        # how much each PID param effects change in omega
        self.PID = [1, 1, 0]



        # subscribers
        img_topic = f"""/{os.environ['VEHICLE_NAME']}/camera_node/image/compressed"""
        self.img_sub = rospy.Subscriber(img_topic, CompressedImage, self.cb_img, queue_size = 1)

        # publishers
        self.img_publisher = rospy.Publisher('/masked_image/compressed', CompressedImage)

        twist_topic = f"/{os.environ['VEHICLE_NAME']}/car_cmd_switch_node/cmd"
        self.twist_publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)


    def cb_imgRed(self, msg):
        # get the image from camera and mask over the hsv range set in init
        data_arr = np.fromstring(msg.data, np.uint8)
        col_img = cv2.imdecode(data_arr, cv2.IMREAD_COLOR)
        crop = [len(col_img) // 3, -1]
        hsv = cv2.cvtColor(col_img, cv2.COLOR_BGR2HSV)
        imagemask = np.asarray(cv2.inRange(hsv[crop[0] : crop[1]], self.red_lower, self.red_upper))

        # find all the red dotted lines
        contours, hierarchy = cv2.findContours(imagemask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # get the largest red stripe
        largest = max(contours, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(largest)
        conts = [largest]
        
        # ignore the largest stripe if it is too close to the bot
        if y > 200:
            contours.remove(largest)
            largest = max(contours, key = cv2.contourArea)
            conts.append(largest)
            x,y,w,h = cv2.boundingRect(largest)
        
        # draw visulaization stuff
        image = cv2.drawContours(col_img[crop[0] : crop[1]], conts, -1, (30, 89, 227), 3)

        self.pub_img = image

        # if only move the bot if drive is true
        if self.drive:
            # set this to y - h//2 for english driver mode
            # set this to y + h//2 for american driver mode
            self.pid(x, y + h//2, len(image[i]) // 2) 



    def lanelogic(self, imagemask, col_img,crop):
         # find the current color in the FOV
        contours, hierarchy = cv2.findContours(imagemask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # get the largest current color stripe
        largest = max(contours, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(largest)
        conts = [largest]
        
        # ignore the largest stripe if it is too close to the bot
        if y > 200:
            contours.remove(largest)
            largest = max(contours, key = cv2.contourArea)
            conts.append(largest)
            x,y,w,h = cv2.boundingRect(largest)
        
        #return 
        return x, y, w, h, conts
        


    def cb_img(self, msg):
        # get the image from camera and mask over the hsv range set in init
        data_arr = np.fromstring(msg.data, np.uint8)
        col_img = cv2.imdecode(data_arr, cv2.IMREAD_COLOR)
        crop = [len(col_img) // 3, -1]
        hsv = cv2.cvtColor(col_img, cv2.COLOR_BGR2HSV)
        yellow_imagemask = np.asarray(cv2.inRange(hsv[crop[0] : crop[1]], self.lower_bound, self.upper_bound)) #get yellow boxes
        red_imagemask = np.asarray(cv2.inRange(hsv[crop[0] : crop[1]], self.red_lower, self.red_upper)) #get red boxes

        yellow_x, yellow_y, yellow_w, yellow_h, yellow_conts = self.lanelogic(yellow_imagemask,col_img,crop)
        red_x, red_y, red_w, red_h, red_conts = self.lanelogic(red_imagemask,col_img,crop)

        # draw visulaization stuff for red stop
        image = cv2.drawContours(col_img[crop[0] : crop[1]], red_conts, -1, (45, 227, 224), 3)

        image = cv2.line(image, (red_x, red_y+red_h//2), (red_x + int((self.size_ratio*(red_y+red_h))), red_y+red_h), (45, 227, 224), 2)

        #red publisher
        self.pub_img = image

        # draw visulaization stuff for yellow lane 
        image = cv2.drawContours(col_img[crop[0] : crop[1]], yellow_conts, -1, (0,255,0), 3)

        image = cv2.line(image, (yellow_x, yellow_y+yellow_h//2), (yellow_x + int((self.size_ratio*(yellow_y+yellow_h))), yellow_y+yellow_h), (0,255,0), 2)

        imx, imy, d = image.shape
        r = imy - yellow_y
        image = cv2.circle(image, (int((len(image[0]) // 2) - r * np.sin(self.omega)), int(imy - r * np.cos(self.omega))), 4, (0, 0, 255), -1)

        for i in range(len(image)):
            image[i][len(image[i]) // 2] = [255, 0, 0]

        #yellow publisher
        self.pub_img = image

        # if only move the bot if drive is true
        if self.drive:
            # American driver
            self.pid(yellow_x, yellow_y + yellow_h//2, len(image[i]) // 2) 
            #English Driver
            #self.pid(yellow_x, yellow_y - yellow_h//2, len(image[i]) // 2) 


       
        

    def img_pub(self):
        if self.pub_img is not None:
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', self.pub_img)[1]).tostring()

            self.img_publisher.publish(msg)

    # controll the speed and angle of the bot
    def twist_pub(self):
        if self.drive:
            msg = Twist2DStamped()
            msg.v = self.speed
            msg.omega = self.omega

            self.twist_publisher.publish(msg)

    def pid(self, x, y, goal):
        # proprtional part
        scale_for_pixel_area = 0.02
        diff = ((x + int((self.size_ratio*y))) - goal) * scale_for_pixel_area  
        self.omega = -self.PID[0] * diff

        # derivative part
        # curr_time = rospy.get_time()
        # dt = curr_time - self.prev_time
        # self.prev_time = curr_time
        # if self.prev_diff != None:
        #     self.omega += self.PID[2] * (diff - self.prev_diff) / dt
        # self.prev_diff = diff

        # integral part?

        print(diff, self.omega)

if __name__ == '__main__':
    # create the node
    node = lane_follow_node(node_name='custom_lane_follow')

    rate = rospy.Rate(100) # 1Hz
    while not rospy.is_shutdown() and node.run:
        node.img_pub()
        node.twist_pub()
        #node.change_led_to(node.curr_col)
        rate.sleep()
    
