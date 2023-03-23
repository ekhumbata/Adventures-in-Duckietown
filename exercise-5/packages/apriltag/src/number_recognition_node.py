#!/usr/bin/env python3
import cv2

import os
import rospy
from duckietown.dtros import DTROS, NodeType
import numpy as np

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, Int8, Bool

# imports for NN
import torch
import torch.nn as nn
import torch.nn.functional as F

import numpy as np

import time
import cv2

# NN node
class MLP(nn.Module):
    def __init__(self, input_dim, output_dim):
        super().__init__()

        self.input_fc = nn.Linear(input_dim, 250)
        self.hidden_fc = nn.Linear(250, 100)
        self.output_fc = nn.Linear(100, output_dim)

    def forward(self, x):

        # x = [batch size, height, width]

        batch_size = x.shape[0]

        x = x.view(batch_size, -1)
        x = x.to(torch.float32)


        # x = [batch size, height * width]

        h_1 = F.relu(self.input_fc(x))

        # h_1 = [batch size, 250]

        h_2 = F.relu(self.hidden_fc(h_1))

        # h_2 = [batch size, 100]

        y_pred = self.output_fc(h_2)

        # y_pred = [batch size, output dim]

        return y_pred, h_2
    
class LeNet(nn.Module):
	def __init__(self, numChannels, classes):
		# call the parent constructor
		super(LeNet, self).__init__()
		# initialize first set of CONV => RELU => POOL layers
		self.conv1 = nn.Conv2d(in_channels=numChannels, out_channels=20,
			kernel_size=(5, 5))
		self.relu1 = nn.ReLU()
		self.maxpool1 =nn.MaxPool2d(kernel_size=(2, 2), stride=(2, 2))
		# initialize second set of CONV => RELU => POOL layers
		self.conv2 = nn.Conv2d(in_channels=20, out_channels=50,
			kernel_size=(5, 5))
		self.relu2 = nn.ReLU()
		self.maxpool2 = nn.MaxPool2d(kernel_size=(2, 2), stride=(2, 2))
		# initialize first (and only) set of FC => RELU layers
		self.fc1 = nn.Linear(in_features=800, out_features=500)
		self.relu3 = nn.ReLU()
		# initialize our softmax classifier
		self.fc2 = nn.Linear(in_features=500, out_features=classes)
		self.logSoftmax = nn.LogSoftmax(dim=1)
                
	def forward(self, x):
		# pass the input through our first set of CONV => RELU =>
		# POOL layers
		x = self.conv1(x)
		x = self.relu1(x)
		x = self.maxpool1(x)
		# pass the output from the previous layer through the second
		# set of CONV => RELU => POOL layers
		x = self.conv2(x)
		x = self.relu2(x)
		x = self.maxpool2(x)
		# flatten the output from the previous layer and pass it
		# through our only set of FC => RELU layers
		x = torch.flatten(x, 1)
		x = self.fc1(x)
		x = self.relu3(x)
		# pass the output to our softmax classifier to get our output
		# predictions
		x = self.fc2(x)
		# print(x)
		output = self.logSoftmax(x)
		# return the output predictions
		return output

# ros node
class num_recog_node(DTROS):

    def __init__(self, node_name):
        super(num_recog_node, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        self.node_name = node_name

        INPUT_DIM = 28 * 28
        OUTPUT_DIM = 10

        # self.model = MLP(INPUT_DIM, OUTPUT_DIM)
        self.model = LeNet(1, OUTPUT_DIM)
        self.model.load_state_dict(torch.load('/data/tut1-model.pt'))

        self.run_fwd = False
        self.num_img = np.array([])
        self.num = -1
        self.dict_tag = {} # dict of all prev seen tags, and prev seen preds with counters
        self.curr_tag = None
        self.verified = set() # set of all verified tags
        self.run = True
        self.num_tags = 10 # num of tags to find
        self.verification_thresh = 3 # number of consistent predictions


        # subscribers
        img_topic = f"""/{os.environ['VEHICLE_NAME']}/num_img/compressed"""
        self.img_sub = rospy.Subscriber(img_topic, CompressedImage, self.cb_img, queue_size = 1)
        self.id_sub = rospy.Subscriber(f"/{os.environ['VEHICLE_NAME']}/april_id", Int32, self.cb_tag, queue_size = 1)
        self.kill_sub = rospy.Subscriber(f"/{os.environ['VEHICLE_NAME']}/shutdown", Bool, self.cb_kill, queue_size = 1)

        # publishers
        self.num_pub = rospy.Publisher("/" + os.environ['VEHICLE_NAME'] + '/predicted_num', Int8, queue_size=1)
        self.kill_pub = rospy.Publisher("/" + os.environ['VEHICLE_NAME'] + '/shutdown', Bool, queue_size=1)




    def cb_img(self, msg):
        # data_arr = np.fromstring(msg.data, np.uint8)
        data_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(data_arr, cv2.IMREAD_GRAYSCALE)
        print("HERE")
        self.run_fwd = True
        self.num_img = img

    def cb_tag(self, msg):
         self.curr_tag = msg.data
    
    def cb_kill(self, msg):
         self.run = msg.data

    def get_num(self):
        # only run fwd pass if and image has been detected and the current tag has not been verified
        if self.run_fwd:
            print("RUNNING FWD PASS...")
            t0 = time.time()
            with torch.no_grad():
                #x = x.to(device)

                y_pred = self.model(torch.tensor([[self.num_img]], dtype=torch.float32))

                y_prob = F.softmax(y_pred, dim=-1)

                
            pred = int(torch.argmax(y_prob, 1)[0])
            print(f"predicted digit: {pred}, with {y_prob[0][pred] * 100}% certianty, in {(time.time() - t0) * 100} secs")
            self.num = int(pred)
            self.run_fwd = False

            
            # if has already been seen
            if self.curr_tag in self.dict_tag:
                print("WHERE")
                # if it has already been predicted as pred add one to the counter
                if pred in self.dict_tag[self.curr_tag]:
                    print("HERE")
                    self.dict_tag[self.curr_tag][pred] += 1
                    # if it has been predicted as pred 3 times it can be considered verified
                    if self.dict_tag[self.curr_tag][pred] == self.verification_thresh:
                        self.verified.add(self.curr_tag)
                # if this is a new prediction set a new pred counter to 1
                else:
                    print("THERE")
                    self.dict_tag[self.curr_tag][pred] = 1
            # if this tag has never been seen create a new pred counter dict
            else:
                self.dict_tag[self.curr_tag] = {pred: 1}
            print(self.dict_tag, self.verified)

        # shutdown all nodes, and print the sign nums once all have been verified
        if len(self.verified) == self.num_tags:
            for k, v in self.dict_tag.items():
                print(f"tag {k} is a {max(v, key=v.get)}")

    # always publish -1 until a number has been detected then publish that number
    def pub_num(self):
        msg = Int8()
        msg.data = self.num
        self.num_pub.publish(msg)

        self.num = -1

    def pub_kill(self):
        msg = Bool()
        try:
            msg.data = (len(self.verified) != self.num_tags)
        except TypeError:
             msg.data = True

        self.kill_pub.publish(msg)
        
    def check_shutdown(self):
        if not self.run:
            rospy.signal_shutdown("all tags detected")


if __name__ == '__main__':
    # create the node
    node = num_recog_node(node_name='number_recog')

    # rate = rospy.Rate(10) # 10Hz
    rate = rospy.Rate(1) # once every 2 s
    while not rospy.is_shutdown():
        node.get_num()
        node.pub_num()
        node.check_shutdown()
        node.pub_kill()
        rate.sleep()
    
