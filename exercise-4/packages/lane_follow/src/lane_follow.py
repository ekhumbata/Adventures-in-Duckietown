#!/usr/bin/env python3
import argparse
import cv2
from collections import deque   #import deque for faster append and pop operations


import os
import rospy
from duckietown.dtros import DTROS, NodeType
import numpy as np
import random

#from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from duckietown_msgs.srv import ChangePattern
from sensor_msgs.msg import CompressedImage, Range
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, BoolStamped, VehicleCorners
from std_msgs.msg import String, Float32


class lane_follow_node(DTROS):

    def __init__(self, node_name):
        super(lane_follow_node, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        self.node_name = node_name

        self.change_led = None
        self.pub_img = None
        self.run = True

        # hsv color values to mask for yellow
        self.yellow_upper = np.array([50, 255, 255]) 
        self.yellow_lower = np.array([20, 60, 0])

        #hsv colour values to mask for red
        self.red_upper = np.array([185, 175, 242])  #[0, 107, 179]
        self.red_lower = np.array([171, 50, 100])   #[13, 190, 241]
        
        # drive speed and ratio of goal vs distance from bot
        self.stopped_t = 0      #realtime update on how long bot has been stopped for
        self.prev_omega = 0     #last angle bot was at 
        self.at_stop_line = False
        self.speed = 0.3        #current speed of bot
        self.omega = 0          #current angle bot is at
        self.size_ratio = 0.6   #distance from centre of duckiebot to dotted line

        #used for the PID control
        self.prev_time = 0      #last time stamp for PID
        self.prev_error = None
        # how much each PID param effects change in omega
        self.PID = [1, 1, 0]
        self.isRunningPID = True
        self.follow = False

        #used to change the color of the LEDS
        self.col = String()
        #used to detect if we are about to collide with another bot
        self.collide = False

        self.validNextTurnOptions = [-1, 0, 1] # TODO: need to add apriltag detection so the bot has some idea of where it is so it can init this

        #how long the PID is inactive while the bot is in the turn sequence 
        self.total_turning_time = 1.5
        #the beginning of the turn time
        self.turning_start_time = 0
        # sets the turn direction 0 -> straight; -1 -> right; 1 -> left
        self.signal = 0

        # self.showBrakeLights = None
        # self.showDrivingLights = None
        self.showLights = [None, None, None, None, None, None] # Driving, Brake, LeftDrive, RightDrive, LeftBrake, RightBrake
        
        #self.prev_range = 10
        #self.prev_t = 0

        self.leader_prev_x_pos = deque() #FIXME: need to init this at top of file, remove comment when done
        self.manual_turn = False    #FIXME: need to init this at top of file, remove comment when done


        # subscribers
        veh = os.environ['VEHICLE_NAME']
        img_topic = f"""/{veh}/camera_node/image/compressed"""
        self.img_sub = rospy.Subscriber(img_topic, CompressedImage, self.cb_img, queue_size = 1)
        self.dist_sub = rospy.Subscriber(f"/{veh}/duckiebot_distance_node/distance", Float32, self.cb_dist, queue_size = 1)
        self.tof_sub = rospy.Subscriber(f"/{veh}/front_center_tof_driver_node/range", Range, self.cb_tof, queue_size = 1)
        self.det_sub = rospy.Subscriber(f"/{veh}/duckiebot_detection_node/detection", BoolStamped, self.cb_bot_det, queue_size = 1)
        self.leader_sub = rospy.Subscriber(f"/{veh}/duckiebot_detection_node/centers", VehicleCorners, self.cb_checkTurn)

        # publishers
        self.img_publisher = rospy.Publisher('/masked_image/compressed', CompressedImage)

        twist_topic = f"/{veh}/car_cmd_switch_node/cmd"
        self.twist_publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

        # services
        led_topic = "/%s" % veh + "/led_emitter_node/set_pattern"
        os.system(f"dts duckiebot demo --demo_name led_emitter_node --duckiebot_name {veh} --package_name led_emitter --image duckietown/dt-core:daffy-arm64v8 && echo RAN LIGHTING DEMO")
        rospy.wait_for_service(led_topic)
        self.change_led = rospy.ServiceProxy(led_topic, ChangePattern)
        self.change_led_col("DRIVING")


    def lane_logic(self, imagemask):
        # find the current color in the FOV
        contours, hierarchy = cv2.findContours(imagemask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


        if len(contours) == 0:
            return 0, 0, 0, 0, []

        # get the largest current color stripe
        largest = max(contours, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(largest)
        conts = [largest]
        
        #ignore the largest stripe if it is too close to the bot
        # if y > 100 and len(contours) > 2:
        #     contours.remove(largest)
        #     largest = max(contours, key = cv2.contourArea)
        #     conts.append(largest)
        #     x,y,w,h = cv2.boundingRect(largest)
        
        #return 
        return x, y, w, h, conts
        
    def change_led_col(self, col):
        self.col.data = col
        #  self.col.data = "LIGHT_OFF"

    def pub_col(self):
        self.change_led(self.col)


    def cb_bot_det(self, msg):
        # print(msg.data)
        self.follow = msg.data
        if not msg.data:
            self.collide = False
    
    def cb_dist(self, msg):
        d = msg.data
        if d < 0.4:
            self.collide = True
            #self.at_stop_line = True
            self.prev_omega = self.omega
        else:
            self.collide = False

    def cb_tof(self, msg):
        r = msg.range
        # if r < 0.15 and abs(r - self.prev_range) < 0.01:
        #     print(f"SHIIIIIIIIT {r}, {abs(r - self.prev_range)}")
        #     self.collide = True
        #     self.at_stop_line = True
        #     self.prev_omega = self.omega
        # else:
        #     self.collide = False
        # self.prev_t += 1
        # if self.prev_t % 30 == 0:
        #     self.prev_range = r
            

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

        # 0 -> straight; -1 -> right; 1 -> left
        is_turning = False


        if self.showLights[0]: self.change_led_col("DRIVING")
        if self.showLights[1]: self.change_led_col("BRAKE")
        if self.showLights[2]: self.change_led_col("CAR_SIGNAL_LEFT_DRIVE")
        if self.showLights[3]: self.change_led_col("CAR_SIGNAL_RIGHT_DRIVE")
        if self.showLights[4]: self.change_led_col("CAR_SIGNAL_LEFT_BRAKE")
        if self.showLights[5]: self.change_led_col("CAR_SIGNAL_RIGHT_BRAKE")

        # Stop driving at a line
        if red_y > 200 and not self.at_stop_line and rospy.Time.now().to_sec() - self.stopped_t >= 5:   
            print("=== stop ===")
            self.showLights = [0,1,0,0,0,0] # Brake


            # Choose random direction to turn from valid list
            # self.signal = random.choice([-1, 0, 1])
            #self.signal = random.choice(self.validNextTurnOptions)
            self.signal = random.choice([0])


            if(self.signal == 1):
                self.showLights = [0,0,0,0,1,0] # LeftBrake
            elif(self.signal == -1):
                self.showLights = [0,0,0,0,0,1] # RightBrake


            self.at_stop_line = True
            self.speed = 0
            self.prev_omega = self.omega
            self.stopped_t = rospy.Time.now().to_sec()

        # Start driving again
        if self.at_stop_line and rospy.Time.now().to_sec() - self.stopped_t >= 2:
            # print("=== drive ===")
            # self.showLights = [1,0,0,0,0,0] # Driving
            # self.signal = 0 # We're done turning - don't turn at next stop

            self.speed = 0.3
            # self.omega = self.prev_omega
            is_turning = True
            self.at_stop_line = False
            self.stopped_t = rospy.Time.now().to_sec()
            self.isRunningPID = True
            self.omega = self.prev_omega

        self.turn(is_turning)

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
        if not self.at_stop_line and rospy.Time.now().to_sec() - self.turning_start_time >= self.total_turning_time:
            # if not following another bot lane follow
            if not self.follow:
                self.pid(yellow_x, yellow_y + yellow_h//2, len(image[i]) // 2) 
            #English Driver
            #self.pid(yellow_x, yellow_y - yellow_h//2, len(image[i]) // 2) 
        


    def img_pub(self):
        #publishs the open cv image 
        if self.pub_img is not None:
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            # msg.data = np.array(cv2.imencode('.jpg', self.pub_img)[1]).tostring()
            msg.data = np.array(cv2.imencode('.jpg', self.pub_img)[1]).tobytes()

            self.img_publisher.publish(msg)


    # control the speed and angle of the bot
    def twist_pub(self):
        if self.at_stop_line or self.collide:
            print("STOPED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            self.speed = 0
            self.omega = 0
        else:
            self.speed = 0.3
        print(f"PUB: {self.omega}")
        msg = Twist2DStamped()
        msg.v = self.speed
        msg.omega = self.omega

        self.twist_publisher.publish(msg)


    #forces a turn, take a variable stating 
    def turn(self, isTurn):
        # at the start of a turn set the omega to the correct dir (L, R or straight)
        if isTurn:
            # this is basically it - just nudge the bot extra in the right direction and hope it gets to where it should approximately
            self.prev_omega = self.omega
            self.omega = (np.pi) * self.signal
            self.turning_start_time = rospy.Time.now().to_sec()
            self.isRunningPID = False


            # Update valid turn options based on possible moves
            if(self.signal == 1):
                if(self.validNextTurnOptions.contains(0)): 
                    self.validNextTurnOptions = [-1, 1] # If we turned and last options contained straight, we must be in middle
                else: 
                    self.validNextTurnOptions = [0, 1] # Otherwise, we must have turned onto a straightaway
            if(self.signal == -1):
                if(self.validNextTurnOptions.contains(0)): 
                    self.validNextTurnOptions = [-1, 1]
                else: 
                    self.validNextTurnOptions = [-1, 0]


        # Stop turning once a certain amount of time has passed and resume lane follow
        isTurningTimeExpired = rospy.Time.now().to_sec() - self.turning_start_time >= self.total_turning_time
        if not isTurningTimeExpired:
            if(self.signal == 1):
                print("=== left drive ===")
                self.showLights = [0,0,1,0,0,0] # LeftDriving
            elif(self.signal == -1):
                print("=== right drive ===")
                self.showLights = [0,0,0,1,0,0] # RightDriving


        if isTurningTimeExpired:
            if not self.isRunningPID: # Run a single time as soon as we finish turn
                print("============= done turn ==============")
                self.showLights = [1,0,0,0,0,0] # Driving
                self.signal = 0 # We're done turning - don't turn at next stop
            self.isRunningPID = True
            self.omega = self.prev_omega




    
    def cb_checkTurn(self, msg):
        """
        This function checks to see if the duckiebot needs to turn.
        Pass in the next x value given from the duckiebot_detection_node.

        This function sets self.manual_turn to true/false if a manual turn is necessary.
        
        NOTE: self.manual_turn should be reset after the turn is complete in another method.

        self: the class object
        msg: the object containing detection data
        """
        # if it sees a bot don't lane follow
        self.follow = True
        # get the leader corners
        #if(len(msg.corners) == 0): return
        leader = msg.corners[-1]
        
        # TODO add rules of road

        # do bot follow based on corners
        print(f"follow pid: {not self.at_stop_line}")
        if not self.at_stop_line and rospy.Time.now().to_sec() - self.turning_start_time >= self.total_turning_time:
            self.follow_pid(leader.x, leader.y)

        # detects leaders prev turn and follow suite

        # only update if leader is detected
        if self.at_stop_line and rospy.Time.now().to_sec() - self.stopped_t >= 2:
            if len(self.leader_prev_x_pos) >= 2:
                #we only want to see the range over ~10 values since we should see a large change within those 10, remove oldest item from queue if over 10
                if(len(self.leader_prev_x_pos) > 10):
                    self.leader_prev_x_pos.popleft()
                self.leader_prev_x_pos.append(msg.corners[-1].x)

                #print(self.leader_prev_x_pos)

                #if there is a large change, return true.
                #FIXME: 40 is a estimate number, should be tweaked later
                change = max(self.leader_prev_x_pos) - min(self.leader_prev_x_pos)
                print(f"CHANGE IN X: {change}")
                if abs(change) > 40:
                    self.turn(True)
                    self.signal = change / abs(change)
            else:
                for i in range(len(self.leader_prev_x_pos)):
                    self.leader_prev_x_pos.pop()
            self.at_stop_line = False


    def pid(self, x, y, goal):
        # proprtional part
        scale_for_pixel_area = 0.02
        diff = ((x + int((self.size_ratio*y))) - goal) * scale_for_pixel_area  
        self.omega = -self.PID[0] * diff

        # derivative part
        curr_time = rospy.get_time()
        dt = curr_time - self.prev_time
        self.prev_time = curr_time
        if self.prev_error != None:
            self.omega += self.PID[2] * (diff - self.prev_error) / dt
        self.prev_error = diff

        # integral part?

        # print("PID: ", diff, self.omega)
        print(f"LANE FOLLOW: using folow speed: {self.speed}, omega: {self.omega}")

    
# x: 350, y: 156

    def follow_pid(self, x, y, goal = [350, 156]):
        scale_for_pixel_area = 0.02
        diff = (x - goal[0]) * scale_for_pixel_area  
        self.omega = -self.PID[0] * diff

        # diff = (y - goal[1]) * scale_for_pixel_area
        # self.speed -= diff
        self.speed = 0.3
        print(f"LEADER FOLLOW: using folow speed: {self.speed}, omega: {self.omega}")


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
    
