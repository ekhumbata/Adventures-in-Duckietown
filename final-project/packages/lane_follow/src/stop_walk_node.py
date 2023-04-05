#!/usr/bin/env python3
import dt_apriltags
import cv2

import os
import rospy
from duckietown.dtros import DTROS, NodeType
import numpy as np

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Bool, Int32
import time



class apriltag_node(DTROS):

    def __init__(self, node_name):
        super(apriltag_node, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        self.node_name = node_name
        self.camera_calibration = None
        self.camera_parameters = None
        self.safeToRunProgram = False

        self.pub_img = np.array([])
        self.grey_img = np.array([])
        self.col_img = None
        self.curr_msg = None
        self.detector = dt_apriltags.Detector()

        self.run = True
        #no detection

        self.p = 0
        self.q = 0
        self.new_num = False
        self.prev_tag = 0
        self.dist_from_april = 999
        self.pub_rate = 30
        self.default_pub_rate = 10
        self.boosted_pub_rate = 30
        self.boosted_pub_rate_cycles = 5 # how many iterations to run the boosted pub rate (aka the number of times we drop clock cycles on the boosted rate to accomidate missed identifications)
        self.boosted_pub_rate_count = 999

        self.yellow_lower = np.array([9, 91, 163])
        self.yellow_upper = np.array([22, 255, 255])
        self.run_pid = True
        self.stop_time = 0
        



        # subscribers
        img_topic = f"""/{os.environ['VEHICLE_NAME']}/camera_node/image/compressed"""
        self.img_sub = rospy.Subscriber(img_topic, CompressedImage, self.cb_img, queue_size = 1)
        self.kill_sub = rospy.Subscriber(f"/{os.environ['VEHICLE_NAME']}/shutdown", Bool, self.cb_kill, queue_size = 1)
        self.april_dist_sub = rospy.Subscriber(f"/{os.environ['VEHICLE_NAME']}/dist_from_april", Float32, self.cb_april_dist, queue_size = 1)

        # publishers
        # self.pub = rospy.Publisher('/grey_img/compressed', CompressedImage, queue_size=10)
        self.pub = rospy.Publisher("/" + os.environ['VEHICLE_NAME'] + '/masked_img/compressed', CompressedImage, queue_size=1)
        self.lane_follow_pub = rospy.Publisher("/" + os.environ['VEHICLE_NAME'] + '/run_lane_follow', Bool, queue_size=1)

    def cb_kill(self, msg):
        self.run = msg.data

    def cb_april_dist(self, msg):
        self.dist_from_april = msg.data

    def cb_img(self, msg):
        # data_arr = np.fromstring(msg.data, np.uint8)
        data_arr = np.frombuffer(msg.data, np.uint8)
        col_img = cv2.imdecode(data_arr, cv2.IMREAD_COLOR)
        hsv = cv2.cvtColor(col_img, cv2.COLOR_BGR2HSV)
        duck_mask = np.asarray(cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)) #get yellow ducks
        contours, hierarchy = cv2.findContours(duck_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        print("DATA", self.dist_from_april, time.time() - self.stop_time, self.run_pid)
        if self.dist_from_april < 0.3:
            if self.run_pid and time.time() - self.stop_time > 5:
                self.stop_time = time.time()
            self.run_pid = False
            if time.time() - self.stop_time > 2:
                self.run_pid = True
                try:
                    self.run_pid = cv2.contourArea(max(contours, key = cv2.contourArea)) < 500
                except ValueError:
                    self.run_pid = True
        else:
            self.run_pid = True
                

        self.pub_img = duck_mask

    def img_pub(self):
        if self.pub_img.any():
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', self.pub_img)[1]).tostring()

            self.pub.publish(msg)

    def pub_lane_follow(self):
        msg = Bool()
        msg.data = self.run_pid

        self.lane_follow_pub.publish(msg)

    def check_shutdown(self):
         if not self.run:
              rospy.signal_shutdown("all tags detected")

if __name__ == '__main__':
    # create the node
    node = apriltag_node(node_name='april_tag_detector')

    # rate = rospy.Rate(10) # once every 10s
    # rate = rospy.Rate(node.pub_rate)
    while not rospy.is_shutdown() and node.run:
        node.img_pub()
        node.check_shutdown()
        node.pub_lane_follow()

        rate = rospy.Rate(node.pub_rate)   # placed here to enable variable refresh
        rate.sleep()
    
