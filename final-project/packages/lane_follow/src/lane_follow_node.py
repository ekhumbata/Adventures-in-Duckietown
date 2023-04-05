#!/usr/bin/env python3

import rospy

import os
import random
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Float32, Bool, Int32
from turbojpeg import TurboJPEG
import cv2
import numpy as np
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
import time

ROAD_MASK = [(20, 60, 0), (50, 255, 255)]
DEBUG = False
ENGLISH = False

ID_LIST = {"right": 48,
           "left": 50,
           "straight": 56}

class LaneFollowNode(DTROS):

    def __init__(self, node_name):
        super(LaneFollowNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.node_name = node_name
        self.veh = rospy.get_param("~veh")

        # Publishers & Subscribers
        self.stop_sub = rospy.Subscriber(f"/{os.environ['VEHICLE_NAME']}/run_lane_follow", Bool, self.cb_run, queue_size = 1)
        self.kill_sub = rospy.Subscriber(f"/{os.environ['VEHICLE_NAME']}/shutdown", Bool, self.cb_kill, queue_size = 1)

        self.pub = rospy.Publisher("/" + self.veh + "/output/image/mask/compressed",
                                   CompressedImage,
                                   queue_size=1)
        self.sub = rospy.Subscriber("/" + self.veh + "/camera_node/image/compressed",
                                    CompressedImage,
                                    self.callback,
                                    queue_size=1,
                                    buff_size="20MB")
        self.vel_pub = rospy.Publisher("/" + self.veh + "/car_cmd_switch_node/cmd",
                                       Twist2DStamped,
                                       queue_size=1)
        self.tagIdSub = rospy.Subscriber("/" + self.veh + "/april_id",
                            Int32,
                            self.tagIdCallback,
                            queue_size=1,
                            buff_size="20MB")
        self.tagIdSub = rospy.Subscriber("/" + self.veh + "/dist_from_april",
                            Float32,
                            self.tagDistCallback,
                            queue_size=1,
                            buff_size="20MB")

        self.jpeg = TurboJPEG()


        # PID Variables
        self.proportional = None
        if ENGLISH:
            self.offset = -240
        else:
            self.offset = 240
        self.velocity = 0.25 # 0.25 (cameron's bot is weak)
        self.twist = Twist2DStamped(v=self.velocity, omega=0)

        # self.P = 0.08 # P for csc22910
        self.P = 0.04   # P for csc22904
        # self.P = 0.04   # P for csc22930
        self.D = -0.004
        self.I = 0.008
        self.last_error = 0
        self.last_time = rospy.get_time()
        self.run = True

        self.stop_t = 0

        # Wait a litcallbackg motor commands
        rospy.Rate(0.20).sleep()

        # Shutdown hook
        rospy.on_shutdown(self.hook)

        # Force Turns
        self.lastTagId = None
        self.tagDist = 999
        self.permittedActions = [-1] # -1: lane follow, 0: straight, 1: left, 2: right
        self.forceTurnStraight = False
        self.forceTurnLeft = False
        self.forceTurnRight = False
        self.turnStartTime = rospy.Time.now().to_sec()
        self.turnStartDelay = 1.25 # how long to continue driving normally before turning
        self.turnTime = 2.25 # rospy time is in seconds (must be greater than turnStartDelay)
        self.randomPath = False
        self.run_pid = True


        self.loginfo("Initialized")



    def cb_kill(self, msg):
        self.run = msg.data

    def cb_run(self, msg):
        if not msg.data:
            self.run_pid = msg.data
            self.stop_t = time.time()

    def tagDistCallback(self, msg):
        self.tagDist = msg.data

    def tagIdCallback(self, msg):
        try: # This func will probably run before the var is defined, so just return early to avoid errors
            self.lastTagId
        except AttributeError:
            return
        self.lastTagId = msg.data


    def callback(self, msg):
        img = self.jpeg.decode(msg.data)
        crop = img[250:-1, :, :]
        crop_width = crop.shape[1]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, ROAD_MASK[0], ROAD_MASK[1])
        crop = cv2.bitwise_and(crop, crop, mask=mask)
        contours, hierarchy = cv2.findContours(mask,
                                               cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_NONE)

        # Search for lane in front
        max_area = 20
        max_idx = -1
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if area > max_area:
                max_idx = i
                max_area = area

        if max_idx != -1:
            M = cv2.moments(contours[max_idx])
            try:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                self.proportional = cx - int(crop_width / 2) + self.offset
                if DEBUG:
                    cv2.drawContours(crop, contours, max_idx, (0, 255, 0), 3)
                    cv2.circle(crop, (cx, cy), 7, (0, 0, 255), -1)
            except:
                pass
        else:
            self.proportional = None

        if DEBUG:
            rect_img_msg = CompressedImage(format="jpeg", data=self.jpeg.encode(crop))
            self.pub.publish(rect_img_msg)

    def drive(self):
        dtime = time.time() - self.stop_t
        if self.run_pid:
            if self.proportional is None:
                self.twist.omega = 0
                self.last_error = 0
            else:
                # P Term
                P = -self.proportional * self.P

                # D Term
                d_time = (rospy.get_time() - self.last_time)
                d_error = (self.proportional - self.last_error) / d_time
                self.last_error = self.proportional
                self.last_time = rospy.get_time()
                D = d_error * self.D

                # I Term
                I = -self.proportional * self.I * d_time

                self.twist.v = self.velocity
                self.twist.omega = P + I + D
                if DEBUG:
                    print(self.proportional, P, D, self.twist.omega, self.twist.v)
        
        elif dtime > 2 and dtime < 5:
            self.twist.omega = 0
            self.twist.v = self.velocity
            if self.lastTagId == ID_LIST["left"] and dtime > 3:
                self.twist.omega = np.pi / 2
            elif self.lastTagId == ID_LIST["right"] and dtime > 3:
                self.twist.omega = -np.pi / 2
        elif dtime < 2:
            self.twist.omega = 0
            self.twist.v = 0
            self.last_error = 0
        else:
            self.run_pid = True

        self.vel_pub.publish(self.twist)

    def check_shutdown(self):
         if not self.run:
              rospy.signal_shutdown("all tags detected")

    def hook(self):
        print("SHUTTING DOWN")
        self.twist.v = 0
        self.twist.omega = 0
        self.vel_pub.publish(self.twist)
        for i in range(8):
            self.vel_pub.publish(self.twist)


if __name__ == "__main__":
    node = LaneFollowNode("lanefollow_node")
    rate = rospy.Rate(10)  # 8hz
    while not rospy.is_shutdown():
        node.drive()
        node.check_shutdown()
        rate.sleep()
