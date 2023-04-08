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
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, WheelEncoderStamped
import time

ROAD_MASK = [(20, 60, 0), (50, 255, 255)]
DEBUG = False
ENGLISH = False
DRIVE = True

ID_LIST = {"right": 48,
           "left": 50,
           "straight": 56}

class LaneFollowNode(DTROS):

    def __init__(self, node_name):
        super(LaneFollowNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.node_name = node_name
        self.veh = os.environ["VEHICLE_NAME"]
        self.parking_stall_ids = [-1, 207, 226, 228, 75]  # spot 0 doesn't exist






        ### Parking Stall ###
        self.parking_stall = 1
        ### Parking Stall ###





        # Publishers & Subscribers
        self.stop_sub = rospy.Subscriber(f"/{os.environ['VEHICLE_NAME']}/run_lane_follow", Bool, self.cb_run, queue_size = 1)
        self.kill_sub = rospy.Subscriber(f"/{os.environ['VEHICLE_NAME']}/shutdown", Bool, self.cb_kill, queue_size = 1)
        self.sub_encoder_ticks_left = rospy.Subscriber("/" +  os.environ['VEHICLE_NAME'] + "/left_wheel_encoder_node/tick",WheelEncoderStamped,self.cb_encoder_data,callback_args="left")
        self.sub_encoder_ticks_right = rospy.Subscriber("/" + os.environ['VEHICLE_NAME'] + "/right_wheel_encoder_node/tick",WheelEncoderStamped,self.cb_encoder_data,callback_args="right")


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

        self.tagXErrorSub = rospy.Subscriber("/" + self.veh + "/april_x_error",
                            Int32,
                            self.tagXErrorCallback,
                            queue_size=1,
                            buff_size="20MB")

        self.tagIdPriorityPub = rospy.Publisher("/" + self.veh + "/april_priority",
                                Int32,
                                queue_size=1)

        self.jpeg = TurboJPEG()


        # PID Variables
        self.proportional = None
        if ENGLISH:
            self.offset = -240
        else:
            self.offset = 200
        # self.velocity = 0.22 # 22910
        # self.velocity = 0.22 # 22904
        self.velocity = 0.25 # 22930
        self.twist = Twist2DStamped(v=self.velocity, omega=0)

        # self.P = 0.08 # P for csc22910
        # self.P = 0.025   # P for csc22904
        self.P = 0.04   # P for csc22930
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
        self.tagXError = 0
        self.permittedActions = [-1] # -1: lane follow, 0: straight, 1: left, 2: right
        self.forceTurnStraight = False
        self.forceTurnLeft = False
        self.forceTurnRight = False
        self.turnStartTime = rospy.Time.now().to_sec()
        self.turnStartDelay = 1.25 # how long to continue driving normally before turning
        self.turnTime = 2.25 # rospy time is in seconds (must be greater than turnStartDelay)
        self.randomPath = False
        self.run_pid = True

        self.lcrop = 0
        self.rcrop = -1
        self.ticks = [0, 0]
        self.stop_ticks = [0, 0]
        self.april_priority = -1
        self.stage = 0
        self.hasClockedParkingLotTag = False

        self.creepingTicks = 0
        self.creepingInterval = 8
        self.missedDetectionCount = 0
        self.maxMissedDetectionCount = 2




        self.loginfo("Initialized")



    def cb_kill(self, msg):
        self.run = msg.data

    def cb_run(self, msg):
        try: # This func will probably run before the var is defined, so just return early to avoid errors
            self.ticks
        except AttributeError:
            return

        if not msg.data:
            self.run_pid = msg.data
            self.stop_t = time.time()
            self.stop_ticks[0] = self.ticks[0]
            self.stop_ticks[1] = self.ticks[1]
            # print("setting STOP TICKS")

    def tagPriorityPub(self):
        try: # This func will probably run before the var is defined, so just return early to avoid errors
            self.april_priority
        except AttributeError:
            return

        msg = Int32()
        msg.data = self.april_priority
        self.tagIdPriorityPub.publish(msg)

    def tagDistCallback(self, msg):
        self.tagDist = msg.data

    def tagXErrorCallback(self, msg):
        self.tagXError = msg.data

    def tagIdCallback(self, msg):
        try: # This func will probably run before the var is defined, so just return early to avoid errors
            self.lastTagId
        except AttributeError:
            return

        self.lastTagId = msg.data

    def cb_encoder_data(self, msg, wheel):
        try: # This func will probably run before the var is defined, so just return early to avoid errors
            self.ticks
        except AttributeError:
            return

        if wheel == "right":
            self.ticks[1] = msg.data
        else:
            self.ticks[0] = msg.data
        # print("ticks:", self.ticks)
        # self.run_pid = False
        


    def callback(self, msg):
        try: # This func will probably run before the var is defined, so just return early to avoid errors
            self.lcrop
        except AttributeError:
            return


        self.april_priority = 227  ## idk if this should really be in here.. this is just so we effectively go into parking state when it is time


        img = self.jpeg.decode(msg.data)
        # print(img.shape)
        crop = img[250:-1, int(self.lcrop):int(self.rcrop), :]
        # print(crop.shape)
        crop_width = crop.shape[1]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, ROAD_MASK[0], ROAD_MASK[1])
        crop = cv2.bitwise_and(crop, crop, mask=mask)
        contours, hierarchy = cv2.findContours(mask,
                                               cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_NONE)

        red_mask_lower = cv2.inRange(hsv, (0, 100, 150), (10, 200, 255))
        red_mask_upper = cv2.inRange(hsv, (170, 100, 150), (179, 200, 255))
        red_contours, hierarchy = cv2.findContours((red_mask_lower + red_mask_upper),
                                               cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_NONE)

        if len(red_contours) > 0:
            _, y, _, _ = cv2.boundingRect(max(red_contours, key = cv2.contourArea))
            # print("Y", y)
            if y > 100 and time.time() - self.stop_t > 8:
                # print("STOP", self.lastTagId, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
                self.run_pid = False
                self.stop_t = time.time()
                self.stop_ticks[0] = self.ticks[0]
                self.stop_ticks[1] = self.ticks[1]
        
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
        # turn_time = 5
        # dtime = time.time() - self.stop_t
        # # PID has been shut off, stop time has elapsed begin turn    
        # if dtime > 2 and dtime < turn_time:
        #     print(f"last tag: {self.lastTagId}, time since stopping: {dtime}, ")
        #     # drive straight 
        #     self.twist.omega = 0
        #     self.twist.v = self.velocity
        #     self.run_pid = False
        #     # once we have driven straight for 3 secs begin turn in correct dir
        #     if self.lastTagId == ID_LIST["left"] and dtime > 4:
        #         print("################################################## LEFT")
        #         self.lcrop = 0
        #         self.rcrop = 640 / 2
        #         self.run_pid = True
        #     elif self.lastTagId == ID_LIST["right"] and dtime > 4:
        #         print("################################################## RIGHT")
        #         self.lcrop = 640 / 2
        #         self.rcrop = -1
        #         self.run_pid = True
        #     # omega is already zero, no change needed
        #     elif self.lastTagId == ID_LIST["straight"] and dtime > 4:
        #         print("################################################## STRAIGHT")
        #         self.lcrop = 0
        #         self.rcrop = 640 / 2
        #         self.run_pid = True
        #     # if not at any intersection sign resume PID
        #     elif dtime > 4:
        #         print("****************************************************")
        #         self.stop_t = 0
        #         self.run_pid = True
        #         self.lcrop = 0
        #         self.rcrop = -1

        # if self.run_pid:
        #     print("RUNNING PID")
        #     if self.proportional is None:
        #         self.twist.omega = 0
        #         self.last_error = 0
        #     else:
        #         # P Term
        #         P = -self.proportional * self.P

        #         # D Term
        #         d_time = (rospy.get_time() - self.last_time)
        #         d_error = (self.proportional - self.last_error) / d_time
        #         self.last_error = self.proportional
        #         self.last_time = rospy.get_time()
        #         D = d_error * self.D

        #         # I Term
        #         I = -self.proportional * self.I * d_time

        #         self.twist.v = self.velocity
        #         self.twist.omega = P #+ I + D
        #         if DEBUG:
        #             print(self.proportional, P, D, self.twist.omega, self.twist.v)
        # # PID has been shut off, wait for stop time to elapse
        # elif dtime < 2:
        #     self.twist.omega = 0
        #     self.twist.v = 0
        #     self.last_error = 0
        # # resume PID
        # if dtime > turn_time:
        #     print("WHACK PRINT RESUME PID")
        #     self.run_pid = True
        #     self.lcrop = 0
        #     self.rcrop = -1


        # self.vel_pub.publish(self.twist)






        ## Stage 0: Turns ##
        # Runs by default

        ## Stage 1: Cross Walk ##
        # Runs by default

        ## Stage 2: Broken Duckie ##
        # Will likely need this to be its own state (disable default)

        ## Stage 3: Cross Walk ##
        # Runs by default

        ## Stage 4: Parking Lot ##
        if(self.lastTagId == 227):
            self.hasClockedParkingLotTag = True # Prime the bot to enter parking lot next time it leaves a stop sign after we clock the 227 apriltag




        #### Stage 0,1,2,3: Defualt ####
        if(self.stage == 0 or self.stage == 1 or self.stage == 2 or self.stage == 3):
            dtime = time.time() - self.stop_t
            if self.run_pid:
                # print("RUNNING PID")
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
                        # print(self.proportional, P, D, self.twist.omega, self.twist.v)
                        pass

            # PID has been shut off, stop time has elapsed begin turn
            elif dtime > 2 and not self.turn_is_complete(self.lastTagId):
                if(self.hasClockedParkingLotTag):
                    self.stage = 4 # ready to go into parking lot stage
                    return


                # print(f"last tag: {self.lastTagId}, time since stopping: {dtime}, ")
                # drive straight 
                self.twist.omega = 0
                self.twist.v = self.velocity
                # once we have driven straight for 3 secs begin turn in correct dir
                if self.lastTagId == ID_LIST["left"] and self.straight_is_complete(self.lastTagId):
                    # print("################################################## LEFT")
                    self.twist.omega = 4.5
                elif self.lastTagId == ID_LIST["right"] and self.straight_is_complete(self.lastTagId):
                    # print("################################################## RIGHT")
                    self.twist.omega = -4.5
                # omega is already zero, no change needed
                elif self.lastTagId == ID_LIST["straight"] and self.straight_is_complete(self.lastTagId):
                    # print("################################################## STRAIGHT")
                    pass
                # if not at any intersection sign resume PID
                # elif dtime > 4:
                #     print("****************************************************")
                #     self.stop_t = 0
                #     self.run_pid = True

            # PID has been shut off, wait for stop time to elapse
            elif dtime < 2:
                self.twist.omega = 0
                self.twist.v = 0
                self.last_error = 0

            # resume PID
            else:
                self.run_pid = True



        #### Temp, hardcode stage 4
        self.stage = 4

        #### Stage 4: Parking Lot ####
        if(self.stage == 4):

            ## temp
            self.last_error = 0 ### test if we actually need this
            ## temp



            

            target_depth = 0  # correct depth depends on self.parking_stall - 2 or 4: 34cm   -    1 or 3: 17cm
            if(self.parking_stall == 2 or self.parking_stall == 4):
                target_depth = 0.40
            else:
                target_depth = 0.20

            ## Tag close enough - STOP!
            if(self.tagDist < target_depth):
                print("******* DONE *******")
                self.twist.v = 0
                self.twist.omega = 0
                # self.stage = 5 ## temp - just murder this task to stop out bot

                ### Once we stop, correct twist



            ## PID middle of tag (only if it IS the priority tag)
            elif(self.lastTagId == self.april_priority and self.tagDist < 900): # if tag dis is huge, it is missing a detection
                self.missedDetectionCount = 0
                # self.creepingTicks = 0
                # print("PID TAG ID:", self.lastTagId)

                # self.tagXError <- thing to PID off of
                if(self.tagXError == 0):
                    self.proportional = None
                else:
                    self.proportional = self.tagXError*0.25 ## the tag error is kind large, correct for it less


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


                    # print(self.proportional, P, D, self.twist.omega, self.twist.v)
                    # if DEBUG:
                    #     pass


            ## We don't see the priority april - wait a couple frames and keep rolling then try creep
            elif(self.lastTagId != self.april_priority or self.tagDist > 900):
                self.missedDetectionCount += 1

                if(self.missedDetectionCount < self.maxMissedDetectionCount): # lost for a couple frames, keep driving in hopes you will see it.
                    print("lost priority tag, continuing for a couple time steps in case we see again")
                    self.twist.omega = 0
                    self.twist.v = 0.5*self.velocity

                else: # we properly lost the tag - start creeping
                    self.creepingTicks += 1

                    if(self.creepingTicks < self.creepingInterval):
                        print("creeping ticks:", self.creepingTicks)
                        if(self.lastTagId == self.parking_stall_ids[3] or self.lastTagId == self.parking_stall_ids[4]): #too far right, spin left a lil
                            print("left")
                            self.twist.omega = 10
                            self.twist.v = 0.1
                        elif(self.lastTagId == self.parking_stall_ids[1] or self.lastTagId == self.parking_stall_ids[2]): #too far left, spin right a lil
                            print("right")
                            self.twist.omega = -10
                            self.twist.v = 0.1
                        else:
                            print("Creeping for visibility")
                            self.twist.omega = 0
                            self.twist.v = 0.5*self.velocity


                    # pause creeping every couple ticks to potentially detect tags
                    elif(self.creepingTicks >= self.creepingInterval):
                        print("pause", self.creepingTicks)
                        self.twist.omega = 0
                        self.twist.v = 0

                        if(self.creepingTicks > 2*self.creepingInterval):
                            self.creepingTicks = 0 # all done break, continue


            
            # targetSpot = self.parking_stall_ids[self.parking_stall]











        if(DRIVE): self.vel_pub.publish(self.twist)


    def turn_is_complete(self, dir):
        # RIGHT TURN
        if dir == ID_LIST["right"]:
            if self.ticks[0] - self.stop_ticks[0] < 250:
                return False
        # LEFT TURN
        elif dir == ID_LIST["left"]:
            if self.ticks[1] - self.stop_ticks[1] < 260:
                return False
        # GO STRAIGHT
        else:
            if self.ticks[0] - self.stop_ticks[0] < 300 and self.ticks[1] - self.stop_ticks[1] < 300:
                return False
        # return true if none of the checks fail
        return True
    
    def straight_is_complete(self, dir):
        # RIGHT TURN
        if dir == ID_LIST["right"]:
            if self.ticks[0] - self.stop_ticks[0] < 140 and self.ticks[1] - self.stop_ticks[1] < 140:
                return False
        # LEFT TURN
        elif dir == ID_LIST["left"]:
            if self.ticks[0] - self.stop_ticks[0] < 150 and self.ticks[1] - self.stop_ticks[1] < 150:
                return False
        # GO STRAIGHT
        else:
            if self.ticks[0] - self.stop_ticks[0] < 300 and self.ticks[1] - self.stop_ticks[1] < 300:
                return False
        # return true if none of the checks fail
        return True
 
    def check_shutdown(self):
         if not self.run:
              rospy.signal_shutdown("all tags detected")

    def hook(self):
        # print("SHUTTING DOWN")
        self.twist.v = 0
        self.twist.omega = 0
        self.vel_pub.publish(self.twist)
        for i in range(8):
            self.vel_pub.publish(self.twist)


if __name__ == "__main__":
    node = LaneFollowNode("lanefollow_node")
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        node.drive()
        node.tagPriorityPub()
        node.check_shutdown()
        rate.sleep()
