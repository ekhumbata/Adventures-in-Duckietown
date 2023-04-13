#!/usr/bin/env python3
import dt_apriltags
import cv2
import tf

import os
import rospy
from duckietown.dtros import DTROS, NodeType
import numpy as np

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Bool, Int32



class apriltag_node(DTROS):

    def __init__(self, node_name):
        super(apriltag_node, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        self.node_name = node_name
        self.camera_calibration = None
        self.camera_parameters = None
        self.safeToRunProgram = False

        self.num_img = np.array([])
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
        self.dist_from_april = 999/2
        self.error_from_april = 0
        self.april_priority = -1
        self.pub_rate = 30
        self.default_pub_rate = 10
        self.boosted_pub_rate = 30
        self.boosted_pub_rate_cycles = 5 # how many iterations to run the boosted pub rate (aka the number of times we drop clock cycles on the boosted rate to accomidate missed identifications)
        self.boosted_pub_rate_count = 999
        

        self.print_publishers = False



        # subscribers
        img_topic = f"""/{os.environ['VEHICLE_NAME']}/camera_node/image/compressed"""
        info_topic = f"""/{os.environ['VEHICLE_NAME']}/camera_node/camera_info"""
        self.img_sub = rospy.Subscriber(img_topic, CompressedImage, self.cb_img, queue_size = 1)
        self.subscriberCameraInfo = rospy.Subscriber(info_topic, CameraInfo, self.camera_info_callback,  queue_size=1)
        self.kill_sub = rospy.Subscriber(f"/{os.environ['VEHICLE_NAME']}/shutdown", Bool, self.cb_kill, queue_size = 1)
        self.april_priority_sub = rospy.Subscriber(f"/{os.environ['VEHICLE_NAME']}/april_priority", Int32, self.cb_april_priority, queue_size = 1)
        self.sub_shutdown = rospy.Subscriber("/" + os.environ['VEHICLE_NAME'] + "/kill_nodes",Bool,self.cb_check_shutdown)


        # publishers
        # self.pub = rospy.Publisher('/grey_img/compressed', CompressedImage, queue_size=10)
        self.pub = rospy.Publisher("/" + os.environ['VEHICLE_NAME'] + '/grey_img/compressed', CompressedImage, queue_size=1)
        self.num_pub = rospy.Publisher("/" + os.environ['VEHICLE_NAME'] + '/num_img/compressed', CompressedImage, queue_size=1)
        self.dist_from_pub = rospy.Publisher("/" + os.environ['VEHICLE_NAME'] + '/dist_from_april', Float32, queue_size=1)
        self.april_id = rospy.Publisher("/" + os.environ['VEHICLE_NAME'] + '/april_id', Int32, queue_size=1)
        self.april_x_error = rospy.Publisher("/" + os.environ['VEHICLE_NAME'] + '/april_x_error', Int32, queue_size=1)


    def cb_april_priority(self, msg):
        self.april_priority = msg.data

    def cb_check_shutdown(self, msg):
         if msg.data:
              rospy.signal_shutdown("PARKED")


    def camera_info_callback(self, msg):
        self.camera_calibration = msg

        # print("== Calibrating Camera ==")

        # currRawImage_height = img.shape[0]
        # currRawImage_width = img.shape[1]
        currRawImage_height = 640
        currRawImage_width = 480

        scale_matrix = np.ones(9)
        if self.camera_calibration.height != currRawImage_height or self.camera_calibration.width != currRawImage_width:
            scale_width = float(currRawImage_width) / self.camera_calibration.width
            scale_height = float(currRawImage_height) / self.camera_calibration.height
            scale_matrix[0] *= scale_width
            scale_matrix[2] *= scale_width
            scale_matrix[4] *= scale_height
            scale_matrix[5] *= scale_height

        self.tag_size = 0.065 #rospy.get_param("~tag_size", 0.065)
        rect_K, _ = cv2.getOptimalNewCameraMatrix(
            (np.array(self.camera_calibration.K)*scale_matrix).reshape((3, 3)),
            self.camera_calibration.D,
            (640,480),
            1.0
        )
        self.camera_parameters = (rect_K[0, 0], rect_K[1, 1], rect_K[0, 2], rect_K[1, 2])


        try:
            self.subscriberCameraInfo.shutdown()
            self.safeToRunProgram = True
            # print("== Camera Info Subscriber successfully killed ==")
        except BaseException:
            pass

    def cb_kill(self, msg):
        self.run = msg.data

    def cb_img(self, msg):
        # data_arr = np.fromstring(msg.data, np.uint8)
        data_arr = np.frombuffer(msg.data, np.uint8)
        col_img = cv2.imdecode(data_arr, cv2.IMREAD_COLOR)
        grey_img = cv2.cvtColor(col_img, cv2.COLOR_BGR2GRAY)
        self.grey_img = grey_img[: 2 * len(col_img) // 3]
        self.col_img = col_img[: 2 * len(col_img) // 3]
        self.curr_msg = msg

    def img_pub(self):
        if self.grey_img.any():
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', self.grey_img)[1]).tostring()

            self.pub.publish(msg)

    def pub_id(self):
        msg = Int32()
        msg.data = self.prev_tag
        self.april_id.publish(msg)


    def pub_april_x_error(self):
        msg = Int32()
        msg.data = self.error_from_april
        self.april_x_error.publish(msg)


    def dist_pub(self):
        if(self.print_publishers): print("Apriltag Distance: {:<10} | Detect: {:<5} | Priority: {:<5} | X: {:<10}".format( (str(round(self.dist_from_april*2, 5))+"m"), str(self.prev_tag), str(self.april_priority), round(self.error_from_april, 5) ))
        msg = Float32()
        msg.data = self.dist_from_april*2  # the distance estimate is 50% short, so publish double
        self.dist_from_pub.publish(msg)

    
    def _matrix_to_quaternion(self, r):
        T = np.array(((0, 0, 0, 0), (0, 0, 0, 0), (0, 0, 0, 0), (0, 0, 0, 1)), dtype=np.float64)
        T[0:3, 0:3] = r
        return tf.transformations.quaternion_from_matrix(T)


    def detect_tag(self):
        if(not self.safeToRunProgram):
            return


        # convert the img to greyscale
        img = self.col_img
        try:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        except cv2.error:
            return

        tags = self.detector.detect(gray, True, self.camera_parameters, self.tag_size)

        # print("[INFO] {} total AprilTags detected".format(len(tags)))


        # Varibale refresh rate
        # if(self.boosted_pub_rate_count < self.boosted_pub_rate_cycles):
        #     # print("boosted cycle running -", self.dist_from_april)
        #     self.pub_rate = self.boosted_pub_rate
        #     self.boosted_pub_rate_count += 1
        # else:
        #     # # print("default cycle running")
        #     self.pub_rate = self.default_pub_rate


        if len(tags) == 0:
            self.dist_from_april = 999/2
            self.error_from_april = -0 #should be 0

            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()
            self.pub.publish(msg)
            return


        # # print("netDets", tags)

        closest = 0
        priority_found = False
        priority_centre = 320
        priority_p = -1
        for tag in tags:
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = tag.corners
            diff = abs(ptA[0] - ptB[0])
            # # print("dif:", diff)
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            

            # draw the bounding box of the AprilTag detection
            line_col = (125, 125, 0)
            num_col = (0, 125, 125)
            cv2.line(img, ptA, ptB, line_col, 2)
            cv2.line(img, ptB, ptC, line_col, 2)
            cv2.line(img, ptC, ptD, line_col, 2)
            cv2.line(img, ptD, ptA, line_col, 2)

            

            # get the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(tag.center[0]), int(tag.center[1]))
            #cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)
            # print("aprilTagX = ", cX, "radfrommid=", rad_from_middle)

            # draw the tag id on the image
            txt_col = (25, 25, 200)
            tag_id = tag.tag_id
            cv2.putText(img, str(tag_id), (cX - 9, cY + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, txt_col, 2)
            # print("[INFO] tag id: {}".format(tag_id))



            # if multiple seen, set col to the closest tag
            if diff > closest:                
                closest = diff

                # turn rotation matrix into quaternion
                self.q = self._matrix_to_quaternion(tag.pose_R)
                self.p = tag.pose_t.T[0]
                self.prev_tag = tag_id
                # print("p:", self.p, "q:", self.q)

                closest = diff
            if(tag_id == self.april_priority):
                priority_found = True
                priority_p = tag.pose_t.T[0]
                priority_centre = cX
        

        # set the dist from april to the dist to the april tag
        self.dist_from_april = self.p[2] # just the camera x dist

        # if self.dist_from_april < 0.5:
        #     # print("starting boosted cycles")
        #     self.boosted_pub_rate_count = 0 # we are good to boost the rate, reset the iter count to 0 to start it
        if priority_found:
            # print(f"found priority dist to {self.april_priority}: {priority_p[2]}")
            self.dist_from_april = priority_p[2] ## just the camera z dist
            self.error_from_april = priority_centre-320   ## Use pixel coords instead of real world coordinates, pid likes it more (and this is easier to do math with)

        # publish the image with the tag id and box to a custom topic
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()
        self.pub.publish(msg)

    def check_shutdown(self):
         if not self.run:
              rospy.signal_shutdown("all tags detected")

if __name__ == '__main__':
    # create the node
    node = apriltag_node(node_name='april_tag_detector')

    # rate = rospy.Rate(10) # once every 10s
    # rate = rospy.Rate(node.pub_rate)
    while not rospy.is_shutdown() and node.run:
        node.detect_tag()
        node.dist_pub()
        node.pub_id()
        node.pub_april_x_error()
        # node.check_shutdown()

        rate = rospy.Rate(10)   # placed here to enable variable refresh
        rate.sleep()
    

