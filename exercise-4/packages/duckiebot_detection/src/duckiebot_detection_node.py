#!/usr/bin/env python3

import cv2

import rospy
from cv_bridge import CvBridge
import os
from duckietown.dtros import DTParam, DTROS, NodeType, ParamType
from duckietown_msgs.msg import BoolStamped, VehicleCorners
from geometry_msgs.msg import Point32
from sensor_msgs.msg import CompressedImage


class DuckiebotDetectionNode(DTROS):
    """
    This node detects if there is another Duckiebot in the image. This is done by recognizing the pattern of circles on
    the back of every robot.

    Subscriber:
        ~image (:obj:`sensor_msgs.msg.CompressedImage`): Input image

    Publishers:
        ~centers (:obj:`duckietown_msgs.msg.VehicleCorners`): Detected pattern (if any)
        ~detection_image/compressed (:obj:`sensor_msgs.msg.CompressedImage`): shows the detected pattern, you may want to remove this one to reduce the delay
        ~detection (:obj:`boolStamped`): If a duckiebot is detected or not
    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(DuckiebotDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # Initialize the parameters
        
        self.host = str(os.environ['VEHICLE_NAME'])
        
        #Frequency at which to process the incoming images
        self.process_frequency = 2
        
        #Number of dots in the pattern, two elements: [number of columns, number of rows]
        self.circlepattern_dims = [7, 3]
        
        #Parameters for the blob detector, passed to `SimpleBlobDetector <https://docs.opencv.org/4.3.0/d0/d7a/classcv_1_1SimpleBlobDetector.html>`_
        self.blobdetector_min_area = 10
        self.blobdetector_min_dist_between_blobs = 2


        self.cbParametersChanged() 

        self.bridge = CvBridge()

        self.last_stamp = rospy.Time.now()

        # Subscriber
        self.sub_image = rospy.Subscriber("/{}/camera_node/image/compressed".format(self.host), CompressedImage, self.cb_image, queue_size=1)

        # Publishers
        self.pub_centers = rospy.Publisher("/{}/duckiebot_detection_node/centers".format(self.host), VehicleCorners, queue_size=1)
        self.pub_circlepattern_image = rospy.Publisher("/{}/duckiebot_detection_node/detection_image/compressed".format(self.host), CompressedImage, queue_size=1)
        self.pub_detection = rospy.Publisher("/{}/duckiebot_detection_node/detection".format(self.host), BoolStamped, queue_size=1)
        self.log("Detection Initialization completed.")

    def cbParametersChanged(self):

        self.publish_duration = rospy.Duration.from_sec(1.0 / self.process_frequency)
        params = cv2.SimpleBlobDetector_Params()
        params.minArea = self.blobdetector_min_area
        params.minDistBetweenBlobs = self.blobdetector_min_dist_between_blobs
        self.simple_blob_detector = cv2.SimpleBlobDetector_create(params)

    def cb_image(self, image_msg):
        """
        Callback for processing a image which potentially contains a back pattern. Processes the image only if
        sufficient time has passed since processing the previous image (relative to the chosen processing frequency).

        The pattern detection is performed using OpenCV's `findCirclesGrid <https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html?highlight=solvepnp#findcirclesgrid>`_ function.

        Args:
            image_msg (:obj:`sensor_msgs.msg.CompressedImage`): Input image

        """
        now = rospy.Time.now()
        if now - self.last_stamp < self.publish_duration:
            return
        else:
            self.last_stamp = now

        vehicle_centers_msg_out = VehicleCorners()
        detection_flag_msg_out = BoolStamped()
        image_cv = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")

        (detection, centers) = cv2.findCirclesGrid(
            image_cv,
            patternSize=tuple(self.circlepattern_dims),
            flags=cv2.CALIB_CB_SYMMETRIC_GRID,
            blobDetector=self.simple_blob_detector,
        )

        # if the pattern is detected, cv2.findCirclesGrid returns a non-zero result, otherwise it returns 0
        # vehicle_detected_msg_out.data = detection > 0
        # self.pub_detection.publish(vehicle_detected_msg_out)

        vehicle_centers_msg_out.header = image_msg.header
        vehicle_centers_msg_out.detection.data = detection > 0
        detection_flag_msg_out.header = image_msg.header
        detection_flag_msg_out.data = detection > 0

        # if the detection is successful add the information about it,
        # otherwise publish a message saying that it was unsuccessful
        if detection > 0:
            points_list = []
            for point in centers:
                center = Point32()
                center.x = point[0, 0]
                center.y = point[0, 1]
                center.z = 0
                points_list.append(center)
            vehicle_centers_msg_out.corners = points_list
            vehicle_centers_msg_out.H = self.circlepattern_dims[1]
            vehicle_centers_msg_out.W = self.circlepattern_dims[0]

        self.pub_centers.publish(vehicle_centers_msg_out)
        self.pub_detection.publish(detection_flag_msg_out)
        if self.pub_circlepattern_image.get_num_connections() > 0:
            cv2.drawChessboardCorners(image_cv, tuple(self.circlepattern_dims), centers, detection)
            image_msg_out = self.bridge.cv2_to_compressed_imgmsg(image_cv)
            self.pub_circlepattern_image.publish(image_msg_out)


if __name__ == "__main__":
    duckiebot_detection_node = DuckiebotDetectionNode("duckiebot_detection")
    rospy.spin()
