#!/usr/bin/env python3

import os
import rospy
import numpy as np
import cv2
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String

#found this by: running gui tools -> cmd 'rostopic list' -> find topic you want -> 'rostopic info [topic name]' 
#then import the files wanted
#from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage


class MySubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        # construct subscriber string and subscriber
        subscriberString = "/" +  os.environ['VEHICLE_NAME'] + "/camera_node/image/compressed"
        #publisherString = "mycameranode/camera_node/image/compressed"

        self.sub = rospy.Subscriber(subscriberString, CompressedImage, self.callback)
        self.pub = rospy.Publisher("/image_raw/compressed", CompressedImage)
        #print("got here!")
        

    def callback(self, data):
        rate = rospy.Rate(30) # 30Hz
        #print("in the callback")
        #rospy.loginfo(f"""Width of image {data.width}, Height of image {data.height}""")
        
        
        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = data.data
        
        self.pub.publish(msg)
        rate.sleep()



if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='my_subscriber_node')
    # keep spinning
    rospy.spin()