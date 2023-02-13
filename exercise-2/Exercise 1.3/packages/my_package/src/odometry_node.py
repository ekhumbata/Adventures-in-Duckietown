#!/usr/bin/env python3
import numpy as np
import os
import rospy
import math
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32

class OdometryNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
        #self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)

        #set extra variables
        self.radius = 0.0318
        self.total_ticks = 135
        self.robot_frame_x = 0
        self.robot_frame_y = 0
        self.robot_frame_theta = 0
        self.right_wheel_vel = 0
        self.left_wheel_vel = 0

        self.initial_left_ticks = 0
        self.initial_right_ticks = 0
        self.initial_avg_ticks = 0

        self.current_left_ticks = 0
        self.current_right_ticks = 0

        # #list of tasks to preform
        # self.tasks = [self.moveDistance(1.5,0.5),self.moveDistance(1.5,-0.5)]

        #these are kinda hacky. it is so the initial tick values are only set once in the callback
        self.left_been_called_back = False
        self.right_been_called_back = False


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

        self.log("Initialized")



    def publishVelocity(self):
         # move the bot
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.header.stamp = rospy.Time.now()
        msg_wheels_cmd.vel_right = self.right_wheel_vel
        msg_wheels_cmd.vel_left = self.left_wheel_vel
        self.pub_velocity.publish(msg_wheels_cmd)


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


        #Put the wheel commands in a message and publish
        # msg_wheels_cmd = WheelsCmdStamped()
        # msg_wheels_cmd.header.stamp = msg.header.stamp
        # msg_wheels_cmd.vel_right = 0
        # msg_wheels_cmd.vel_left = 0
        # self.pub_velocity.publish(msg_wheels_cmd)

        firstDistance = False
        secondDistance = False
        
        if (firstDistance == False):
            firstDistance = self.moveDistance(1.5,0.5)
        elif (secondDistance == False):
            self.initial_avg_ticks = 0
            secondDistance = self.moveDistance(1.5,-0.5)
        

    '''
    stops the bot from moving
    '''
    def stop(self):
        self.left_wheel_vel = 0
        self.right_wheel_vel = 0

    '''
    moves the duckiebot a requested distance at a requested velocity
    deltax = distance in m
    vel = value betweeen -1 and 1
    returns true or false based on completion
    '''
    def moveDistance(self, deltax, vel):

        #get the required number of ticks to move deltax distance
        numTicks = (deltax * self.total_ticks) / (2 * math.pi * self.radius)

        #now get the current avg ticks
        currAvgTicks = (self.current_left_ticks + self.current_right_ticks)/2

        #assign equal wheel velocity, determine if positive or negative
        self.left_wheel_vel = vel
        self.right_wheel_vel = vel

        if((currAvgTicks - self.initial_avg_ticks) < numTicks):
            #update the currAvgTicks
            currAvgTicks = (self.current_left_ticks + self.current_right_ticks)/2
            rospy.loginfo(f"currAvgTicks: {currAvgTicks}  initAvgTicks: {self.initial_avg_ticks}  numTicks: {numTicks}")
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