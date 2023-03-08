#!/usr/bin/env python3

import cv2
import numpy as np

import rospy
import os
from cv_bridge import CvBridge
from duckietown.dtros import DTParam, DTROS, NodeType, ParamType
from duckietown_msgs.msg import BoolStamped, VehicleCorners
from duckietown_msgs.srv import ChangePattern
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String, Float32


class DuckiebotDistanceNode(DTROS):
    """
    responsible for estimating the relative pose to a detected back pattern of a robot
    """

    def __init__(self, node_name):
    
    	
        # Initialize the DTROS parent class
        super(DuckiebotDistanceNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.host = str(os.environ['VEHICLE_NAME'])
	
	#Distance between the centers of the circles on the back
        self.distance_between_centers = 0.0125
        
        #Maximum tolerable reprojection error.
        #If a reprojection error higher than that is observed. May require some actions
        self.max_reproj_pixelerror_pose_estimation = 1.5


        self.bridge = CvBridge()

        # these will be defined on the first call to calc_circle_pattern
        self.last_calc_circle_pattern = None
        self.circlepattern_dist = None
        self.circlepattern = None
        
        # subscribers
        self.sub_centers = rospy.Subscriber("/{}/duckiebot_detection_node/centers".format(self.host), VehicleCorners, self.cb_process_centers, queue_size=1)
        self.sub_info = rospy.Subscriber(
            "/{}/camera_node/camera_info".format(self.host), CameraInfo, self.cb_process_camera_info, queue_size=1
        )


        # publishers
        self.pub_distance_to_robot_ahead = rospy.Publisher("/{}/duckiebot_distance_node/distance".format(self.host), Float32, queue_size=1)
        self.pcm = PinholeCameraModel()
        
        self.log("Initialization completed")


    def cb_process_camera_info(self, msg):
        """
        Callback that stores the intrinsic calibration into a PinholeCameraModel object.

        Args:

            msg (:obj:`sensor_msgs.msg.CameraInfo`): Intrinsic properties of the camera.
        """

        self.pcm.fromCameraInfo(msg)

    def cb_process_centers(self, vehicle_centers_msg):
        """
        Callback that processes a back pattern detection. If no detection was made, publishes a dummy stop
        line message.

        Args:
            vehicle_centers_msg (:obj:`duckietown_msgs.msg.VehicleCorners`): Detected pattern (if any)

        """

        # check if there actually was a detection
        detection = vehicle_centers_msg.detection.data
        if detection:
            self.calc_circle_pattern(vehicle_centers_msg.H, vehicle_centers_msg.W)
            points = np.zeros((vehicle_centers_msg.H * vehicle_centers_msg.W, 2))
            for i in range(len(points)):
                points[i] = np.array([vehicle_centers_msg.corners[i].x, vehicle_centers_msg.corners[i].y])

            success, rotation_vector, translation_vector = cv2.solvePnP(
                objectPoints=self.circlepattern,
                imagePoints=points,
                cameraMatrix=self.pcm.intrinsicMatrix(),
                distCoeffs=self.pcm.distortionCoeffs(),
            )

            if success:
                points_reproj, _ = cv2.projectPoints(
                    objectPoints=self.circlepattern,
                    rvec=rotation_vector,
                    tvec=translation_vector,
                    cameraMatrix=self.pcm.intrinsicMatrix(),
                    distCoeffs=self.pcm.distortionCoeffs(),
                )

                mean_reproj_error = np.mean(
                    np.sqrt(np.sum((np.squeeze(points_reproj) - points) ** 2, axis=1))
                )

                if mean_reproj_error < self.max_reproj_pixelerror_pose_estimation:
                    (R, jac) = cv2.Rodrigues(rotation_vector)
                    R_inv = np.transpose(R)
                    translation_vector = -np.dot(R_inv, translation_vector)
                    distance_to_vehicle = -translation_vector[2]
                    
                    #####publish the distance information to a topic###
                    self.pub_distance_to_robot_ahead.publish(Float32(distance_to_vehicle))


                else:
                    self.log(
                        "Pose estimation failed, too high reprojection error. "
                        "Reporting detection at 0cm for safety."
                    )
            else:
                self.log("Pose estimation failed. " "Reporting detection at 0cm for safety.")


    def calc_circle_pattern(self, height, width):
        """
        Calculates the physical locations of each dot in the pattern.

        Args:
            height (`int`): number of rows in the pattern
            width (`int`): number of columns in the pattern

        """
        # check if the version generated before is still valid, if not, or first time called, create

        if self.last_calc_circle_pattern is None or self.last_calc_circle_pattern != (height, width):
            self.circlepattern_dist = self.distance_between_centers
            self.circlepattern = np.zeros([height * width, 3])
            for i in range(0, width):
                for j in range(0, height):
                    self.circlepattern[i + j * width, 0] = (
                        self.circlepattern_dist * i - self.circlepattern_dist * (width - 1) / 2
                    )
                    self.circlepattern[i + j * width, 1] = (
                        self.circlepattern_dist * j - self.circlepattern_dist * (height - 1) / 2
                    )


if __name__ == "__main__":
    duckiebot_distance_node = DuckiebotDistanceNode(node_name="duckiebot_distance_node")
    rospy.spin()
