#!/usr/bin/env python3
import numpy as np
import os
import rospy
import math
import time
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped, LEDPattern
from std_msgs.msg import Header, Float32
from duckietown.dtros.utils import apply_namespace
from std_msgs.msg import String
from std_msgs.msg import ColorRGBA
import rosbag


class OdometryNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")
        self.duckieBotName = os.environ['HOSTNAME']
        # Get static parameters
        #self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)

        # self._led_gpio_pin = rospy.get_param("~led_gpio_pin")

        #set extra variables
        self.radius = 0.0318
        self.length = 0.05
        self.total_ticks = 135
        self.robot_frame_x = 0
        self.robot_frame_y = 0
        self.robot_frame_theta = 0 # in range [0,2pi]
        self.world_frame_x = 0.32
        self.world_frame_y = 0.32
        # self.world_frame_theta = 0 # DONT USE - JUST USE ROBOT FRAME THETA
        self.right_wheel_vel = 0
        self.left_wheel_vel = 0

        self.initial_left_ticks = 0
        self.initial_right_ticks = 0
        self.initial_avg_ticks = 0
        self.initial_l_r_ticks_diff = 0

        self.current_left_ticks = 0
        self.current_right_ticks = 0

        self.last_avg_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0

        self.trim = 0.01 # more positive = more right, (-1 to +1)

        # #list of tasks to preform
        # self.tasks = [self.moveDistance(1.5,0.5),self.moveDistance(1.5,-0.5)]

        #these are kinda hacky. it is so the initial tick values are only set once in the callback
        self.left_been_called_back = False
        self.right_been_called_back = False

        # self.firstDistanceCompleted = False
        # self.secondDistanceCompleted = False
        # self.firstRotationCompleted = False

        self.turningOvershootCorrection = 0
        # self.straightlineOvershootCorrection = -80
        self.strightlineScalingCorrection = 0.8

        # self.actionComplete = [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False]
        # self.actionFirstVisitCount = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.num_actions = 23
        self.actionComplete = [False for element in range(self.num_actions)]
        self.actionFirstVisitCount = [0 for element in range(self.num_actions)]
        
        self.waitTimeBetweenActions = 20
        self.state = 0

        self.rwDistance = 0
        self.lwDistance = 0
        self.rw_updated = False
        self.lw_updated = False


        # Subscribing to the wheel encoders
        #the fourth argument is the callback argument for the cb_encoder_data method
        #MAY NEED TO SAY callback_args = "left"/"right" for the fourth arg in the below two lines
        self.sub_encoder_ticks_left = rospy.Subscriber("/" +  os.environ['VEHICLE_NAME'] + "/left_wheel_encoder_node/tick",WheelEncoderStamped,self.cb_encoder_data,callback_args="left")
        self.sub_encoder_ticks_right = rospy.Subscriber("/" + os.environ['VEHICLE_NAME'] + "/right_wheel_encoder_node/tick",WheelEncoderStamped,self.cb_encoder_data,callback_args="right")

        #self.sub_executed_commands = rospy.Subscriber("/" +  os.environ['VEHICLE_NAME'] + "/wheels_driver_node/wheels_cmd_executed",WheelsCmdStamped,self.cb_executed_commands)

        # Publishers
        #self.pub_integrated_distance_left = rospy.Publisher(...)
        #self.pub_integrated_distance_right = rospy.Publisher(...)
        self.pub_velocity = rospy.Publisher("wheels_driver_node/wheels_cmd", WheelsCmdStamped)
        self.pub_led = rospy.Publisher("/" + os.environ['VEHICLE_NAME'] + "/led_emitter_node/led_pattern", LEDPattern, queue_size=10)

        self.bag = rosbag.Bag('coordinate.bag', 'w')


        self.log("Initialized")



    def publishVelocity(self):
         # move the bot
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.header.stamp = rospy.Time.now()
        msg_wheels_cmd.vel_right = self.right_wheel_vel
        msg_wheels_cmd.vel_left = self.left_wheel_vel
        self.pub_velocity.publish(msg_wheels_cmd)

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


    def cb_encoder_data(self, msg, wheels):
        """ 
        Update encoder distance information from ticks.
        Formula for ticks: (deltaX = 2 * pi * radius * ticks) / total

        deltaX = distance travelled for one wheel
        radius = 0.0318m
        ticks = number of ticks measured from each wheel
        total = the number of ticks in one full revolution (135)

        length of axel on duckiebot = 0.05m
        """
        #deltaX = 2 * math.pi * self.radius * msg.data * self.total_ticks

        #get the init tick count for each wheel
        if wheels == "left" and self.left_been_called_back == False:
            #set the member variables for initial left ticks
            self.initial_left_ticks = msg.data 
            self.left_been_called_back = True
        elif wheels == "right" and self.right_been_called_back == False:
            #set the member variables for initial right ticks
            self.initial_right_ticks = msg.data 
            self.right_been_called_back = True
        
        if(self.initial_avg_ticks == 0 and self.right_been_called_back and self.left_been_called_back):
            self.initial_avg_ticks = (self.initial_right_ticks + self.initial_left_ticks)/2

        #get the updated tick counts for each wheel
        if wheels == "left" and self.left_been_called_back == True:
            #set the member variables for current left ticks
            self.current_left_ticks = msg.data 
        elif wheels == "right" and self.right_been_called_back == True:
            #set the member variables for current right ticks
            self.current_right_ticks = msg.data 



        ## I couldn't get theta tracking to work - hardcode it
        if(self.state == 0 or self.state == 1):
            self.robot_frame_theta = 0
        if(self.state == 2 or self.state == 3 or self.state == 4):
            self.robot_frame_theta = np.pi*1.5
        if(self.state == 5 or self.state == 6):
            self.robot_frame_theta = 0
        if(self.state == 7 or self.state == 8 or self.state == 9 or self.state == 10 or self.state == 11):
            self.robot_frame_theta = np.pi * 0.5
        if(self.state == 12 or self.state == 13):
            self.robot_frame_theta = np.pi
        if(self.state == 14 or self.state == 15 or self.state == 16 or self.state == 17 or self.state == 18):
            self.robot_frame_theta = 0
        # circle doesn't work
        ## I couldn't get theta tracking to work - hardcode it





        ### First State: Wait 5s ###
        # set LED colour Red
        if  (self.actionComplete[0] == False):
            if(self.waitAndResetEncoderInitReadings(0)):
                self.publishLEDs(1.0, 0.0, 0.0)
                self.actionComplete[0] = True

        # wait 5s
        elif(self.actionComplete[1] == False):
            if(self.waitAndResetEncoderInitReadings(1)):
                self.actionComplete[1] = self.wait(1, 550)
        ### First State: Wait 5s ###




        ### Second State: Draw a C ###
        # set LED colour Green
        elif(self.actionComplete[2] == False):
            if(self.waitAndResetEncoderInitReadings(2)):
                self.publishLEDs(0.0, 1.0, 0.0)
                self.actionComplete[2] = True

        # Turn right pi/2
        elif(self.actionComplete[3] == False):
            if(self.waitAndResetEncoderInitReadings(3)):
                # self.actionComplete[3] = self.rotateDegree(math.pi * 0.5, 0.5)
                self.actionComplete[3] = self.rotateDegree(math.pi * 0.55, 0.5) 

        # go straight 1.25m (now in bottom right)
        elif(self.actionComplete[4] == False):
            if(self.waitAndResetEncoderInitReadings(4)):
                self.actionComplete[4] = self.moveDistance(1.25, 0.5)

        # turn left pi/2
        elif(self.actionComplete[5] == False):
            if(self.waitAndResetEncoderInitReadings(5)):
                # self.actionComplete[5] = self.rotateDegree(math.pi * 0.5, -0.5)
                self.actionComplete[5] = self.rotateDegree(math.pi * 0.42, -0.5)

        # go straight 1.25m (now in top right)
        elif(self.actionComplete[6] == False):
            if(self.waitAndResetEncoderInitReadings(6)):
                self.actionComplete[6] = self.moveDistance(1.25, 0.5)

        # turn left pi/2
        elif(self.actionComplete[7] == False):
            if(self.waitAndResetEncoderInitReadings(7)):
                # self.actionComplete[7] = self.rotateDegree(math.pi * 0.5, -0.5)
                self.actionComplete[7] = self.rotateDegree(math.pi * 0.45, -0.5)

        # go straight 1.25m (now in top left, facing left)
        elif(self.actionComplete[8] == False):
            if(self.waitAndResetEncoderInitReadings(8)):
                self.actionComplete[8] = self.moveDistance(1.1, 0.5) #1.25 is too long for some reason

        # set LED colour Red
        elif(self.actionComplete[9] == False):
            if(self.waitAndResetEncoderInitReadings(9)):
                self.publishLEDs(1.0, 0.0, 0.0)
                self.actionComplete[9] = True

        # wait 5s
        elif(self.actionComplete[10] == False):
            if(self.waitAndResetEncoderInitReadings(10)):
                self.actionComplete[10] = self.wait(1, 850)
        ### Second State: Draw a C ###




        ### Third State: Return to home (use world coordinates??) ###
        # set LED colour Blue
        elif(self.actionComplete[11] == False):
            if(self.waitAndResetEncoderInitReadings(11)):
                self.publishLEDs(0.0, 0.0, 1.0)
                self.actionComplete[11] = True

        # turn right pi/2
        elif(self.actionComplete[12] == False):
            if(self.waitAndResetEncoderInitReadings(12)):
                self.actionComplete[12] = self.rotateDegree(math.pi * 0.5, -0.5)


        # go straight 1.25m (now in bottom left)
        elif(self.actionComplete[13] == False):
            if(self.waitAndResetEncoderInitReadings(13)):
                # self.actionComplete[13] = self.moveDistance(1.25, 0.5)
                self.actionComplete[13] = self.moveDistance(1.0, 0.5)

        # turn right pi
        elif(self.actionComplete[14] == False):
            if(self.waitAndResetEncoderInitReadings(14)):
                self.actionComplete[14] = self.rotateDegree(math.pi, -0.5)

        # set LED colour Red
        elif(self.actionComplete[15] == False):
            if(self.waitAndResetEncoderInitReadings(15)):
                self.publishLEDs(1.0, 0.0, 0.0)
                self.actionComplete[15] = True

        # wait 5s
        elif(self.actionComplete[16] == False):
            if(self.waitAndResetEncoderInitReadings(16)):
                self.actionComplete[16] = self.wait(1, 1200)
        ### Third State: Return to home (use world coordinates??) ###




        ### Fourth State: Clockwise circle ###
        # set LED colour Pink
        elif(self.actionComplete[17] == False):
            if(self.waitAndResetEncoderInitReadings(17)):
                self.publishLEDs(1.0, 0.0, 1.0)
                self.actionComplete[17] = True

        # drive forward 0.625m
        elif(self.actionComplete[18] == False):
            if(self.waitAndResetEncoderInitReadings(18)):
                self.actionComplete[18] = self.moveDistance(0.3, 0.5)

        
        # drive in a circle of r=0.625m for pi*1.5 rad
        elif(self.actionComplete[19] == False):
            if(self.waitAndResetEncoderInitReadings(19)):
                # self.actionComplete[19] = self.driveInCircle(0.625, 0.5)
                self.actionComplete[19] = self.driveInCircle(0.625, 0.5)

        # drive forward 0.625m
        elif(self.actionComplete[20] == False):
            if(self.waitAndResetEncoderInitReadings(20)):
                self.actionComplete[20] = self.moveDistance(0.3, 0.5)

        # turn right pi/2
        elif(self.actionComplete[21] == False):
            if(self.waitAndResetEncoderInitReadings(21)):
                self.actionComplete[21] = self.rotateDegree(math.pi * 0.5, 0.5)

        # We're done. Turn off LEDs
        elif(self.actionComplete[22] == False):
            if(self.waitAndResetEncoderInitReadings(22)):
                self.publishLEDs(0.0, 0.0, 0.0)
                self.actionComplete[22] = True
        ### Fourth State: Clockwise circle ###


        if(all(completed for completed in self.actionComplete)):
            print("All Done. Shutting Down.")
            time.sleep(2) # if we shutdown too fast the other processes do not terminate properly.
            self.bag.close()
            rospy.signal_shutdown("Task completed.")



    '''
    stops the bot from moving
    '''
    def stop(self):
        self.left_wheel_vel = 0
        self.right_wheel_vel = 0

    def resetInitEncoderReadings(self):
        # reset straigtline driving ticks
        self.initial_avg_ticks = (self.current_left_ticks + self.current_right_ticks)/2
        self.initial_l_r_ticks_diff = self.current_left_ticks - self.current_right_ticks

        # reset turn ticks
        self.initial_left_ticks = self.current_left_ticks
        self.initial_right_ticks = self.current_right_ticks

        # reset tracking ticks
        self.last_avg_ticks = (self.current_left_ticks + self.current_right_ticks)/2
        self.last_left_ticks = self.current_left_ticks
        self.last_right_ticks = self.current_right_ticks


    def waitAndResetEncoderInitReadings(self, actionNum):
        self.state = actionNum
        self.actionFirstVisitCount[actionNum] += 1
        # print(actionNum, self.actionFirstVisitCount[actionNum], self.actionFirstVisitCount)
        if(self.actionFirstVisitCount[actionNum] < self.waitTimeBetweenActions):
            self.resetInitEncoderReadings()
            return False # Waiting, do not run action.
        else:
            return True # Done waiting, can run action.

    def wait(self, actionNum, ticks):
        self.actionFirstVisitCount[actionNum] += 1
        if(self.actionFirstVisitCount[actionNum] < ticks):
            return False # Waiting, do not run action.
        else:
            return True # Done waiting, can run action.





    def driveInCircle(self, circleRadius, vel):
        outerWheelTravelDist = 2 * math.pi * (circleRadius + self.length)
        innerWheelTravelDist = 2 * math.pi * (circleRadius - self.length)

        outerTicks = (outerWheelTravelDist * self.total_ticks) / (2 * math.pi * self.radius)
        innerTicks = (innerWheelTravelDist * self.total_ticks) / (2 * math.pi * self.radius)

        # We only want 3/4 of a full circle
        # outerTicks *= 0.75
        # innerTicks *= 0.75
        # We only want 3/4 of a full circle

        distRatio = innerTicks / (1.7 * outerTicks)


        self.left_wheel_vel = vel
        self.right_wheel_vel = vel * distRatio

        leftDiff = abs(self.current_left_ticks - self.initial_left_ticks)
        rightDiff = abs(self.current_right_ticks - self.initial_right_ticks)

        isGoodToMove = leftDiff < outerTicks and rightDiff < innerTicks

        # hidden for ros bagging
        # rospy.loginfo(f"cL: {self.current_left_ticks} initL: {self.initial_left_ticks} diffL: {self.current_left_ticks - self.initial_left_ticks} | cR: {self.current_right_ticks} initR: {self.initial_right_ticks} diffR: {self.current_right_ticks - self.initial_right_ticks} | {leftDiff} < {outerTicks} ~ {rightDiff} < {innerTicks} move?: {isGoodToMove}")

        if(isGoodToMove):
            return False
        else:
            self.stop()
            return True





    '''
    moves the duckiebot a requested distance at a requested velocity
    deltax = distance in m
    vel = value betweeen -1 and 1
    returns true or false based on completion
    '''
    def moveDistance(self, deltax, vel):

        deltax *= self.strightlineScalingCorrection # scale input for the correction amount

        #get the required number of ticks to move deltax distance
        numTicks = (deltax * self.total_ticks) / (2 * math.pi * self.radius)
        # numTicks += self.straightlineOvershootCorrection

        #now get the current avg ticks
        currAvgTicks = (self.current_left_ticks + self.current_right_ticks)/2

        #assign equal wheel velocity, determine if positive or negative
        self.left_wheel_vel = vel + self.trim # doesn't consider when trim sets wheel vel > 1 todo update
        self.right_wheel_vel = vel - self.trim

        # live perturb trim (later)
        # if(self.current_left_ticks - self.current_right_ticks > self.initial_l_r_ticks_diff + 10):
        #     self.trim += 0.01
        # if(self.current_right_ticks - self.current_left_ticks < self.initial_l_r_ticks_diff - 10):
        #     self.trim -= 0.01

        # print(self.current_left_ticks - self.current_right_ticks, " | ", self.initial_l_r_ticks_diff , " | ", (self.current_right_ticks - self.current_left_ticks) - self.initial_l_r_ticks_diff, " | ", self.trim)
        # live perturb trim (later)

        ## Update the world frame based on distance moved ##
        changeInRobotY = ((currAvgTicks - self.last_avg_ticks) * 2 * math.pi * self.radius) / self.total_ticks
        changeInRobotX = 0

        self.world_frame_x += (np.cos(self.robot_frame_theta) * changeInRobotX) - (np.sin(self.robot_frame_theta) * changeInRobotY)
        self.world_frame_y += (np.sin(self.robot_frame_theta) * changeInRobotX) + (np.cos(self.robot_frame_theta) * changeInRobotY)

        self.last_avg_ticks = currAvgTicks
        ## Update the world frame based on distance moved ##

        isGoodToMove = abs(currAvgTicks - self.initial_avg_ticks) < numTicks

        # world x, world y, robot x, robot y, theta, state num
        rospy.loginfo(f"{self.world_frame_x}, {self.world_frame_y}, {changeInRobotX}, {changeInRobotY}, {self.robot_frame_theta}, {self.state}")

        # record in rosbag
        s = String()
        s.data = f"{self.world_frame_x}, {self.world_frame_y}, {changeInRobotX}, {changeInRobotY}, {self.robot_frame_theta}, {self.state}"
        self.bag.write('time', s)
        # record in rosbag
        
        # rospy.loginfo(f"currAvgTicks: {currAvgTicks}  initAvgTicks: {self.initial_avg_ticks}  diff: {currAvgTicks - self.initial_avg_ticks} numTicks: {numTicks} move?: {isGoodToMove}")

        if(isGoodToMove):
            # #update the currAvgTicks
            # currAvgTicks = (self.current_left_ticks + self.current_right_ticks)/2
            return False
        else:
            self.stop()
            return True


    # theta in radians
    def rotateDegree(self, deltaTheta, vel):

        # if given negative theta, cast to positive and swap velocity
        if(deltaTheta < 0):
            deltaTheta = -1 * deltaTheta
            vel = -1 * vel

        #get the required number of ticks to turn deltaTheta in rotation
        # right = self.radius * vel
        # left = self.radius * -1 * vel
        # degTurned = (right - left) / (2 * self.length)

        # deltaTheta

        wheelTravelDistanceInMetersToTurnThetaRadians = deltaTheta * self.length
        numTicks = (wheelTravelDistanceInMetersToTurnThetaRadians * self.total_ticks) / (2 * math.pi * self.radius)
        numTicks += self.turningOvershootCorrection


        # check - this function should produce no forward/backward motion in the robot frame
        # if(right + left != 0): # x = (r+l)/2, /2 not necessary for check
        #     print("Some error has occured")

        #now get the current avg ticks
        # currAvgTicks = (abs(self.current_left_ticks) + abs(self.current_right_ticks))
        # currAvgTicks = 0
        # if(vel > 0): # rotating right
        #     currAvgTicks = (self.current_left_ticks - self.current_right_ticks)
        # else: # rotating left
        #     currAvgTicks = (self.current_right_ticks - self.current_left_ticks)

        #assign equal wheel velocity, determine if positive or negative
        #TODO: fix trim
        # self.left_wheel_vel = (vel + self.trim) # doesn't consider when trim sets wheel vel > 1 TODO update
        # self.right_wheel_vel = -1 * (vel - self.trim) # * -1 so that right wheel spins opposite

        self.left_wheel_vel = vel # doesn't consider when trim sets wheel vel > 1 TODO update
        self.right_wheel_vel = -1 * vel # * -1 so that right wheel spins opposite

        # if(self.initial_avg_ticks > 0 and vel < 0):
        #     print("SWAPPING")
        #     currAvgTicks = currAvgTicks * -1 # if we are moving backwards, the ticks will be negative. Swap so we can terminate properly
        # We are just going to pay attention to a single wheel for finding cutoff

        ## Update the world frame based on distance moved ##
        if(self.rwDistance != (abs(self.last_right_ticks - self.current_right_ticks) * (2*np.pi*self.radius))/self.total_ticks): #right wheel update
            self.rw_updated = True
        if(self.lwDistance != (abs(self.last_left_ticks - self.current_left_ticks) * (2*np.pi*self.radius))/self.total_ticks): #left wheel update
            self.lw_updated = True

        if(self.lw_updated and self.rw_updated): # this is called each time a wheel updates, if we do the calculation every time it will not work.
            self.rwDistance = (abs(self.last_right_ticks - self.current_right_ticks) * (2*np.pi*self.radius))/self.total_ticks
            self.lwDistance = (abs(self.last_left_ticks - self.current_left_ticks) * (2*np.pi*self.radius))/self.total_ticks

            changeInTheta = (((self.radius * self.lwDistance) - (self.radius * self.rwDistance)) / (2 * self.length)) * 10 # it is like way off - this may help
            # self.robot_frame_theta += changeInTheta

            # print("CHANGE", changeInTheta, "rw:", self.rwDistance, "lw:", self.lwDistance)

            if(self.robot_frame_theta > 2*np.pi):
                self.robot_frame_theta -= 2*np.pi
            if(self.robot_frame_theta < 0):
                self.robot_frame_theta += 2*np.pi

            self.world_frame_x += 0
            self.world_frame_y += 0

            self.lw_updated = False
            self.rw_updated = False

        changeInRobotY = 0
        changeInRobotX = 0
        ## Update the world frame based on distance moved ##


        leftDiff = abs(self.current_left_ticks - self.initial_left_ticks)
        rightDiff = abs(self.current_right_ticks - self.initial_right_ticks)
        isGoodToMove = (leftDiff + rightDiff)/2 < numTicks

        # world x, world y, robot x, robot y, theta, state num
        rospy.loginfo(f"{self.world_frame_x}, {self.world_frame_y}, {changeInRobotX}, {changeInRobotY}, {self.robot_frame_theta}, {self.state}")

        # rospy.loginfo(f"cL: {self.current_left_ticks} initL: {self.initial_left_ticks} diffL: {self.current_left_ticks - self.initial_left_ticks} | cR: {self.current_right_ticks} initR: {self.initial_right_ticks} diffR: {self.current_right_ticks - self.initial_right_ticks} | ave: {(leftDiff + rightDiff)/2} numTicks: {numTicks} move?: {isGoodToMove}")

        if(isGoodToMove):
            # #update the currAvgTicks
            # currAvgTicks = (self.current_left_ticks + self.current_right_ticks)/2
            return False
        else:
            self.stop()
            return True


    def cb_executed_commands(self, msg):
        """ Use the executed commands to determine the direction of travel of each wheel.
        """



if __name__ == '__main__':
    node = OdometryNode(node_name='my_odometry_node')
    # Keep it spinning to keep the node alive
    
    
    rate = rospy.Rate(10) # 1Hz
    while not rospy.is_shutdown():

        node.publishVelocity()
        rate.sleep()


    rospy.loginfo("wheel_encoder_node is up and running...")