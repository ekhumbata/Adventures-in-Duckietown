#!/usr/bin/env python3
import argparse
import cv2

import os
import rospy
from duckietown.dtros import DTROS, NodeType
import numpy as np

#from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from duckietown_msgs.srv import ChangePattern
from sensor_msgs.msg import CompressedImage, Range
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from std_msgs.msg import String, Float32


class lane_follow_node(DTROS):

    def __init__(self, node_name):
        super(lane_follow_node, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        self.node_name = node_name

        self.pub_img = None
        self.run = True

        # hsv color values to mask
        self.yellow_upper = np.array([35, 255, 255]) 
        self.yellow_lower = np.array([20, 45, 25])

        self.red_upper = np.array([185, 175, 242])
        self.red_lower = np.array([171, 50, 100])
        # drive speed and ratio of goal vs distance from bot
        self.stopped_t = 0
        self.prev_omega = 0
        self.drive = True
        self.speed = 0.3
        self.omega = 0
        self.size_ratio = 0.8   #distance from centre of duckiebot to dotted line

        self.prev_time = 0
        self.prev_diff = None
        # how much each PID param effects change in omega
        self.PID = [1, 1, 0]

        self.col = String()
        self.collide = False
        self.prev_range = 10
        self.prev_t = 0

        # subscribers
        img_topic = f"""/{os.environ['VEHICLE_NAME']}/camera_node/image/compressed"""
        self.img_sub = rospy.Subscriber(img_topic, CompressedImage, self.cb_img, queue_size = 1)
        self.dist_sub = rospy.Subscriber(f"/{os.environ['VEHICLE_NAME']}/duckiebot_distance_node/distance", Float32, self.cb_dist, queue_size = 1)
        self.tof_sub = rospy.Subscriber(f"/{os.environ['VEHICLE_NAME']}/front_center_tof_driver_node/range", Range, self.cb_tof, queue_size = 1)

        # publishers
        self.img_publisher = rospy.Publisher('/masked_image/compressed', CompressedImage)

        twist_topic = f"/{os.environ['VEHICLE_NAME']}/car_cmd_switch_node/cmd"
        self.twist_publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

        # services
        led_topic = "/%s" % os.environ['VEHICLE_NAME'] + "/led_emitter_node/set_pattern"
        os.system(f"dts duckiebot demo --demo_name led_emitter_node --duckiebot_name {os.environ['VEHICLE_NAME']} --package_name led_emitter --image duckietown/dt-core:daffy-arm64v8 && echo RAN LIGHTING DEMO")
        rospy.wait_for_service(led_topic)
        self.change_led = rospy.ServiceProxy(led_topic, ChangePattern)


    # def cb_img(self, msg):
    #     # get the image from camera and mask over the hsv range set in init
    #     data_arr = np.fromstring(msg.data, np.uint8)
    #     col_img = cv2.imdecode(data_arr, cv2.IMREAD_COLOR)
    #     crop = [len(col_img) // 3, -1]
    #     hsv = cv2.cvtColor(col_img, cv2.COLOR_BGR2HSV)
    #     imagemask = np.asarray(cv2.inRange(hsv[crop[0] : crop[1]], self.lower_bound, self.upper_bound))

    def lane_logic(self, imagemask):
        # find the current color in the FOV
        contours, hierarchy = cv2.findContours(imagemask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


        if len(contours) == 0:
            print(contours)
            return 0, 0, 0, 0, []

        # get the largest current color stripe
        largest = max(contours, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(largest)
        conts = [largest]
        
        # ignore the largest stripe if it is too close to the bot
        # if y > 200:
        #     contours.remove(largest)
        #     largest = max(contours, key = cv2.contourArea)
        #     conts.append(largest)
        #     x,y,w,h = cv2.boundingRect(largest)
        
        #return 
        return x, y, w, h, conts
        
    def change_led_col(self, col):
        self.col.data = col

    def pub_col(self):
        self.change_led(self.col)

    
    def cb_dist(self, msg):
        d = msg.data
        print(f"AHHHHHHHHHHHHHHHHHH {d}")
        if d < 0.2:
            self.collide = True
            self.drive = False
            self.prev_omega = self.omega
        else:
            self.collide = False

    def cb_tof(self, msg):
        r = msg.range
        if r < 0.15 and abs(r - self.prev_range) < 0.01:
            print(f"SHIIIIIIIIT {r}, {abs(r - self.prev_range)}")
            self.collide = True
            self.drive = False
            self.prev_omega = self.omega
        else:
            self.collide = False
        self.prev_t += 1
        if self.prev_t % 30 == 0:
            self.prev_range = r
            

    def cb_img(self, msg):
        # get the image from camera and mask over the hsv range set in init
        data_arr = np.fromstring(msg.data, np.uint8)
        col_img = cv2.imdecode(data_arr, cv2.IMREAD_COLOR)
        crop = [len(col_img) // 3, -1]
        hsv = cv2.cvtColor(col_img, cv2.COLOR_BGR2HSV)
        yellow_imagemask = np.asarray(cv2.inRange(hsv[crop[0] : crop[1]], self.yellow_lower, self.yellow_upper)) #get yellow boxes
        red_imagemask = np.asarray(cv2.inRange(hsv[crop[0] : crop[1]], self.red_lower, self.red_upper)) #get red boxes

        yellow_x, yellow_y, yellow_w, yellow_h, yellow_conts = self.lane_logic(yellow_imagemask)
        red_x, red_y, red_w, red_h, red_conts = self.lane_logic(red_imagemask)

        # Stop driving driving at a line
        if red_y > 200 and self.drive and rospy.Time.now().to_sec() - self.stopped_t >= 5:
            self.change_led_col("CAR_SIGNAL_LEFT")

            self.drive = False
            self.prev_omega = self.omega
            self.stopped_t = rospy.Time.now().to_sec()

        # Start driving again
        if not self.drive and rospy.Time.now().to_sec() - self.stopped_t >= 2:
            self.change_led_col("GREEN")

            self.speed = 0.3
            self.omega = self.prev_omega
            if np.random.randint(2, size = 1)[0] == 0:
                print("TURN!!!!!!!!!!!!")
                self.omega = -np.pi / 4
            self.drive = True
            self.stopped_t = rospy.Time.now().to_sec()

        # draw visulaization stuff for red stop
        image = cv2.drawContours(col_img[crop[0] : crop[1]], red_conts, -1, (45, 227, 224), 3)

        image = cv2.line(image, (red_x, red_y+red_h//2), (red_x + int((self.size_ratio*(red_y+red_h))), red_y+red_h), (45, 227, 224), 2)

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

            # test led service
            col = String()
            col.data = "BLUE"
            self.change_led(col)

            self.img_publisher.publish(msg)


    # control the speed and angle of the bot
    def twist_pub(self):
        if not self.drive or self.collide:
            self.speed = 0
            self.omega = 0
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
        curr_time = rospy.get_time()
        dt = curr_time - self.prev_time
        self.prev_time = curr_time
        if self.prev_diff != None:
            self.omega += self.PID[2] * (diff - self.prev_diff) / dt
        self.prev_diff = diff

        # integral part?

        print(diff, self.omega)


if __name__ == '__main__':
    # create the node
    node = lane_follow_node(node_name='custom_lane_follow')

    rate = rospy.Rate(100) # 1Hz
    while not rospy.is_shutdown() and node.run:
        node.img_pub()
        node.twist_pub()
        #node.pub_col()
        #node.change_led_to(node.curr_col)
        rate.sleep()
    
