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

ROAD_MASK = [(20, 60, 0), (50, 255, 255)]
DEBUG = False
ENGLISH = False

class LaneFollowNode(DTROS):

    def __init__(self, node_name):
        super(LaneFollowNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.node_name = node_name
        self.veh = rospy.get_param("~veh")

        # Publishers & Subscribers
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
        self.velocity = 0.35 # 0.25 (cameron's bot is weak)
        self.twist = Twist2DStamped(v=self.velocity, omega=0)

        # self.P = 0.08 # P for csc22910
        self.P = 0.04   # P for csc22904
        # self.P = 0.04   # P for csc22930
        self.D = -0.004
        self.I = 0.008
        self.last_error = 0
        self.last_time = rospy.get_time()
        self.run = True

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


        self.loginfo("Initialized")



    def cb_kill(self, msg):
        self.run = msg.data


    def tagDistCallback(self, msg):
        self.tagDist = msg.data

    def tagIdCallback(self, msg):
        try: # This func will probably run before the var is defined, so just return early to avoid errors
            self.lastTagId
        except AttributeError:
            return


        currTagId = msg.data
        # print("tag:", currTagId)
        

        # We've spotted a new tag (and are close enough to start the turn timer)!
        if(self.tagDist < 0.25): # we are close enough - once we fall under xcm then we can start the turn timer
            # if(currTagId != self.lastTagId):
            if(currTagId == 153):
                if(self.randomPath):  self.permittedActions = [0, 1]
                else:                 self.permittedActions = [1]
                
            elif(currTagId == 58):
                if(self.randomPath):  self.permittedActions = [0, 2]
                else:                 self.permittedActions = [2]

            elif(currTagId == 62):
                if(self.randomPath):  self.permittedActions = [0, 1]
                else:                 self.permittedActions = [0]

            elif(currTagId == 169):
                if(self.randomPath):  self.permittedActions = [1, 2]
                else:                 self.permittedActions = [2]

            elif(currTagId == 162):
                if(self.randomPath):  self.permittedActions = [1, 2]
                else:                 self.permittedActions = [1]

            elif(currTagId == 133):
                if(self.randomPath):  self.permittedActions = [0, 2]
                else:                 self.permittedActions = [0]
                
            else:
                self.permittedActions = [-1] # If we are anywhere else, just lane follow



            # After we've defined permitted actions - force once of them!
            if(len(self.permittedActions) > 0):
                action = random.choice(self.permittedActions)

                if(action == -1): # Just Lane Follow
                    self.turnStartTime = rospy.Time.now().to_sec()
                    self.forceTurnStraight = False
                    self.forceTurnLeft = False
                    self.forceTurnRight = False

                if(action == 0): # Force Straight
                    self.turnStartTime = rospy.Time.now().to_sec()
                    self.forceTurnStraight = True
                    self.forceTurnLeft = False
                    self.forceTurnRight = False

                elif(action == 1): # Force Left
                    self.turnStartTime = rospy.Time.now().to_sec()
                    self.forceTurnStraight = False
                    self.forceTurnLeft = True
                    self.forceTurnRight = False


                elif(action == 2): # Force Right
                    self.turnStartTime = rospy.Time.now().to_sec()
                    self.forceTurnStraight = False
                    self.forceTurnLeft = False
                    self.forceTurnRight = True


        self.lastTagId = currTagId




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


        ### Force Truns ###
        if(DEBUG):
            print("forceLeft?:", self.forceTurnLeft)
            print("forceRight?:", self.forceTurnRight)
            print("forceStraight?:", self.forceTurnStraight)
            print("check:", rospy.Time.now().to_sec() - self.turnStartTime, ">", self.turnTime)

        if(self.forceTurnLeft):
            # print("ready to turn left")
            pass
        if(self.forceTurnRight):
            # print("ready to turn right")
            pass

        if(self.forceTurnLeft or self.forceTurnRight or self.forceTurnStraight):
            self.twist.omega = 0  # As soon as we've clocked an apriltag we need to cut lane following so it doesn't do wacky stuff on right turns


        if(self.forceTurnLeft and (rospy.Time.now().to_sec() - self.turnStartTime > self.turnStartDelay)):
            print("Turning Left")
            self.twist.omega = 4

        elif(self.forceTurnRight and (rospy.Time.now().to_sec() - self.turnStartTime > self.turnStartDelay)):
            print("Turning Right")
            self.twist.omega = -6

        elif(self.forceTurnStraight and (rospy.Time.now().to_sec() - self.turnStartTime > self.turnStartDelay)):
            print("Straight")
            self.twist.omega = 0



        if(rospy.Time.now().to_sec() - self.turnStartTime > self.turnTime):
            self.forceTurnStraight = False
            self.forceTurnLeft = False
            self.forceTurnRight = False
        ### Force Turns ###

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
