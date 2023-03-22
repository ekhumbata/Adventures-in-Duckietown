#!/usr/bin/env python3
import dt_apriltags
import argparse
import cv2
import tf

import os
import rospy
from duckietown.dtros import DTROS, NodeType
import numpy as np

from sensor_msgs.msg import CameraInfo
from duckietown_msgs.srv import ChangePattern
from duckietown_msgs.msg import LEDPattern
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Float32
from std_msgs.msg import ColorRGBA



class apriltag_node(DTROS):

    def __init__(self, node_name):
        super(apriltag_node, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        self.node_name = node_name
        self.camera_calibration = None
        self.camera_parameters = None
        self.safeToRunProgram = False
        self._tf_bcaster = tf.TransformBroadcaster()

        self.num_img = np.array([])
        self.grey_img = np.array([])
        self.col_img = None
        self.curr_msg = None
        self.detector = dt_apriltags.Detector()

        self.run = True
        self.prev_img = None
        #no detection
        self.curr_col = "WHITE"
        self.sign_col_map = {
            #blue = T-intersection
            153: "BLUE",
            58: "BLUE", 
            133: "BLUE", 
            62: "BLUE", 
            #red = stop sign
            162: "RED",
            169: "RED",
            #green = UofA Tag
            201: "GREEN",
            200: "GREEN",
            94: "GREEN",
            93: "GREEN"
        }

        self.p = 0
        self.q = 0
        self.new_num = False
        self.prev_tag = 0
        self.dist_from_april = 999



        # subscribers
        img_topic = f"""/{os.environ['VEHICLE_NAME']}/camera_node/image/compressed"""
        info_topic = f"""/{os.environ['VEHICLE_NAME']}/camera_node/camera_info"""
        self.img_sub = rospy.Subscriber(img_topic, CompressedImage, self.cb_img, queue_size = 1)
        self.subscriberCameraInfo = rospy.Subscriber(info_topic, CameraInfo, self.camera_info_callback,  queue_size=1)

        # publishers
        # self.pub = rospy.Publisher('/grey_img/compressed', CompressedImage, queue_size=10)
        self.pub = rospy.Publisher("/" + os.environ['VEHICLE_NAME'] + '/grey_img/compressed', CompressedImage, queue_size=1)
        self.pub_led = rospy.Publisher("/" + os.environ['VEHICLE_NAME'] + "/led_emitter_node/led_pattern", LEDPattern, queue_size=1)
        self.pub_april = rospy.Publisher("/april_topic", String, queue_size=1)
        self.num_pub = rospy.Publisher("/" + os.environ['VEHICLE_NAME'] + '/num_img/compressed', CompressedImage, queue_size=1)
        self.dist_from_pub = rospy.Publisher("/" + os.environ['VEHICLE_NAME'] + '/dist_from_april', Float32, queue_size=1)

        # services 
        # led_topic = "/%s" % os.environ['VEHICLE_NAME'] + "/led_emitter_node/set_pattern"
        # os.system(f"dts duckiebot demo --demo_name led_emitter_node --duckiebot_name {os.environ['VEHICLE_NAME']} --package_name led_emitter --image duckietown/dt-core:daffy-arm64v8 && echo RAN LIGHTING DEMO")
        # rospy.wait_for_service(led_topic)
        # self.change_led = rospy.ServiceProxy(led_topic, ChangePattern)

        self.publishLEDs(1.0, 0.0, 0.0)

    def sign_to_col(self, id):
        if(id in self.sign_col_map):
            return self.sign_col_map[id]
        else:
            print("[INFO] Recognized AprilTag - but the ID is not valid.")
            return "WHITE"


    def camera_info_callback(self, msg):
        self.camera_calibration = msg

        print("== Calibrating Camera ==")

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
            print("== Camera Info Subscriber successfully killed ==")
        except BaseException:
            pass


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
    
    def pub_num(self):
        # only publish img if it exists, and the pariltag is new
        if self.num_img.any() and self.new_num:
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', self.num_img)[1]).tostring()

            self.num_pub.publish(msg)
            self.new_num = False


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

    def april_pub(self):
        msg = String()
        msg.data = f"{self.p}:{self.q}"
        print("april", msg.data)

        self.pub_april.publish(msg)

    def dist_pub(self):
        msg = Float32()
        msg.data = self.dist_from_april

        self.dist_from_pub.publish(msg)


    def change_led_to(self, new_col):
        # print("col:", new_col)

        if(new_col == "RED"):
            self.publishLEDs(1.0, 0.0, 0.0)

        elif(new_col == "GREEN"):
            self.publishLEDs(0.0, 1.0, 0.0)

        elif(new_col == "BLUE"):
            self.publishLEDs(0.0, 0.0, 1.0)

        else:
            self.publishLEDs(1.0, 1.0, 1.0)

        # col = String()
        # col.data = new_col
        # self.change_led(col)


    def convert_pixel_x_to_rad_from_middle(self, pixel_x):
        # lol turns out this was the wrong approach
        # Camera FOV = 160deg => right edge is 80deg from midpoint (and left edge is -80deg)
        # 80 deg in rad
        frame_edge_rad = 1.39626
        camera_res_x = 640 # so here x=640 => +1.39626 rad from centre, x=0 => -1.3626

        percent_offset_from_midpoint = (pixel_x - (camera_res_x/2)) / (camera_res_x/2) # gives value in [-1, 1] with 0 being perfect middle

        rad_from_middle = frame_edge_rad * percent_offset_from_midpoint # gives value in [-1.3626, 1.3626] rad = [-80, 80] deg

        return rad_from_middle

    
    def _matrix_to_quaternion(self, r):
        T = np.array(((0, 0, 0, 0), (0, 0, 0, 0), (0, 0, 0, 0), (0, 0, 0, 1)), dtype=np.float64)
        T[0:3, 0:3] = r
        return tf.transformations.quaternion_from_matrix(T)


    def detect_tag(self):
        if(not self.safeToRunProgram):
            return


        # convert the img to greyscale
        img =  self.col_img
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        tags = self.detector.detect(gray, True, self.camera_parameters, self.tag_size)

        print("[INFO] {} total AprilTags detected".format(len(tags)))


        if len(tags) == 0:
            self.change_led_to("WHITE")
            self.curr_col = "WHITE"
            self.dist_from_april = 999

            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()
            self.pub.publish(msg)
            return

        closest_col = "WHITE" 
        closest = 0

        # print("netDets", tags)

        for tag in tags:
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = tag.corners
            diff = abs(ptA[0] - ptB[0])
            # print("dif:", diff)
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

            # rad_from_middle = self.convert_pixel_x_to_rad_from_middle(cX)
            # print("aprilTagX = ", cX, "radfrommid=", rad_from_middle)

            # draw the tag id on the image
            txt_col = (25, 25, 200)
            tag_id = tag.tag_id
            cv2.putText(img, str(tag_id), (cX - 9, cY + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, txt_col, 2)
            print("[INFO] tag id: {}".format(tag_id))

            # if multiple seen, set col to the closest tag
            if diff > closest:
                # get the points of the box to draw around the closest number
                w = int(ptB[0]) - int(ptA[0])
                h = int(ptA[1]) - int(ptD[1])

                num_top_left = (ptD[0], ptD[1] - (9*h // 8))
                num_bottom_left = (ptD[0], ptD[1] - h // 8)
                num_top_right = (ptC[0], ptC[1] - (9*h // 8))
                num_bottom_right = (ptC[0], ptC[1] - h // 8)
                
                closest = diff
                closest_col = self.sign_to_col(tag_id)





            # turn rotation matrix into quaternion
            self.q = self._matrix_to_quaternion(tag.pose_R)
            self.p = tag.pose_t.T[0]

            print("p:", self.p, "q:", self.q)
        

            # publish tf
            self._tf_bcaster.sendTransform(
                self.p.tolist(),
                self.q.tolist(),
                self.curr_msg.header.stamp,
                "tag/{:s}".format(str(tag.tag_id)),
                self.curr_msg.header.frame_id,
            )

        # set the dist from april to the dist to the april tag
        self.dist_from_april = 45

        # draw a box around the closest number
        cv2.line(img, num_top_left, num_bottom_left, num_col, 2)
        cv2.line(img, num_bottom_left, num_bottom_right, num_col, 2)
        cv2.line(img, num_bottom_right, num_top_right, num_col, 2)
        cv2.line(img, num_top_right, num_top_left, num_col, 2)

        if self.prev_tag != tag_id:
            # set the masked image of the number to be published to /{bot_name}/num_img/compressed
            self.num_img = gray[num_top_left[1]: num_bottom_left[1], num_bottom_left[0]: num_bottom_right[0]]
            # self.num_img = cv2.inRange(self.num_img, 0, 75)                                    # masking here gives more gradient b/w black and white pixels
            self.num_img = cv2.resize(self.num_img, dsize=(28, 28), interpolation=cv2.INTER_CUBIC)
            self.num_img = cv2.inRange(self.num_img, 0, 75)                                      # masking here gives a sharper image 
            self.new_num = True
            self.prev_tag = tag_id

        # change the led based on the tag id
        self.change_led_to(closest_col)
        self.curr_col = closest_col

        # publish the image with the tag id and box to a custom topic
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()
        self.pub.publish(msg)




if __name__ == '__main__':
    # create the node
    node = apriltag_node(node_name='april_tag_detector')

    # rate = rospy.Rate(10) # 10Hz
    rate = rospy.Rate(1) # once every 2 s
    while not rospy.is_shutdown() and node.run:
        node.pub_num()
        node.change_led_to(node.curr_col)
        node.detect_tag()
        node.april_pub()
        node.dist_pub()
        rate.sleep()
    
