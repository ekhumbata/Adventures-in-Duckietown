#!/usr/bin/env python3
import cv2

import os
import rospy
from duckietown.dtros import DTROS, NodeType
import numpy as np

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Bool, Int32
import time



class stop_walk_node(DTROS):
    """
    This node detects peduckstrians at cross walks 

    Note: many of these values are hard coded to fit with duckiebot: CSC22910
    """

    def __init__(self, node_name):
        super(stop_walk_node, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        self.node_name = node_name
        self.pub_img = np.array([])
        self.col_img = None
        self.run = True

        self.yellow_lower = np.array([9, 91, 163])      #lower HSV bound of ducks
        self.yellow_upper = np.array([22, 255, 255])    #higher HSV bound of ducks 
        self.run_pid = True                             #determine if the PID should be running                 
        self.stop_time = 0                              #time since last stop

        # subscribers
        img_topic = f"""/{os.environ['VEHICLE_NAME']}/camera_node/image/compressed"""
        self.img_sub = rospy.Subscriber(img_topic, CompressedImage, self.cb_img, queue_size = 1)
        self.kill_sub = rospy.Subscriber(f"/{os.environ['VEHICLE_NAME']}/shutdown", Bool, self.cb_kill, queue_size = 1)
        self.sub_shutdown = rospy.Subscriber("/" + os.environ['VEHICLE_NAME'] + "/kill_nodes",Bool,self.cb_check_shutdown)

        # publishers
        self.pub = rospy.Publisher("/" + os.environ['VEHICLE_NAME'] + '/masked_img/compressed', CompressedImage, queue_size=1)
        self.lane_follow_pub = rospy.Publisher("/" + os.environ['VEHICLE_NAME'] + '/run_lane_follow', Bool, queue_size=1)

    def cb_kill(self, msg):
        """
        Legacy callback for shutting down nodes - need to refactor
        """
        self.run = msg.data

    def cb_check_shutdown(self, msg):
        """
        Callback checks if we should shut down this node.

        Should only shut down after parking is complete.
        """
        if msg.data:
            rospy.signal_shutdown("PARKED")

    def cb_img(self, msg):
        """
        This callback turns the PID off if the driver is within a certain distance of the peduckstrians

        Will publish the stop command for as long as we see peduckstrians
        """
        # data_arr = np.fromstring(msg.data, np.uint8)
        data_arr = np.frombuffer(msg.data, np.uint8)
        col_img = cv2.imdecode(data_arr, cv2.IMREAD_COLOR)
        hsv = cv2.cvtColor(col_img, cv2.COLOR_BGR2HSV)
        
        duck_mask = np.asarray(cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)) #get yellow ducks
        contours, hierarchy = cv2.findContours(duck_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        try:
            self.run_pid = cv2.contourArea(max(contours, key = cv2.contourArea)) < 500
        except ValueError:
            self.run_pid = True
                

        self.pub_img = duck_mask

    def img_pub(self):
        """
        Publishes the masked image to the masked_img/compressed topic showing only the duckiebots
        """
        if self.pub_img.any():
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', self.pub_img)[1]).tostring()

            self.pub.publish(msg)

    def pub_lane_follow(self):
        """
        Publishes to the run_lane_follow topic to determine if lane follow should be running
        """
        msg = Bool()
        msg.data = self.run_pid

        self.lane_follow_pub.publish(msg)

    def check_shutdown(self):
        """
        Legacy - Need to refactor
        """
        if not self.run:
            rospy.signal_shutdown("all tags detected")

if __name__ == '__main__':
    # create the node
    node = stop_walk_node(node_name='stop_walk_node')
    # print("STARTING SLEEP...")
    time.sleep(30)
    print("WAKING UP STOP WALK")

    while not rospy.is_shutdown() and node.run:
        node.img_pub()
        node.check_shutdown()
        node.pub_lane_follow()

        rate = rospy.Rate(10)   # placed here to enable variable refresh
        rate.sleep()
    
