#! /usr/bin/env python3
"""
    Script ...
"""

import pyrealsense2 as rs
import cv2
from cv2 import aruco
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from mavros import setpoint as SP
import transform as tr
from std_msgs.msg import Bool

class MarkerDetector():
    def __init__(self):
        rospy.init_node('aruco_detector', anonymous=True)
        mavros.set_namespace('mavros')
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # self.cap = cv2.VideoCapture(0)
        # self.mtx = np.array([5.9778919413469134e+02, 0.0, 3.2893543056979632e+02, 0.0, 6.0031367126366081e+02, 2.4530312117189993e+02, 0.0, 0.0, 1.0]).reshape(3,3)
        self.mtx = np.array([917.497, 0.0, 635.002, 0.0, 915.865, 368.915, 0.0, 0.0, 1.0]).reshape(3,3)
        self.dist = np.array([7.2697873963251586e-02, -1.4749282442847444e-01, -2.3233094539353212e-03, 8.9165121414591982e-03,-2.6332902664556002e-01])

        # matrix from imu to camera
        self.imu_cam = np.zeros((4,4), dtype=np.float)

        self.dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.param = cv2.aruco.DetectorParameters_create()
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.aruco_marker_pos_pub = rospy.Publisher('/aruco_marker_pos', PS, queue_size=10)
        self.fly_pos_pub = rospy.Publisher('/fly_pos', PS, queue_size=10)
        # self.target_position = rospy.Publisher('/target_position', PS, queue_size=10)
        # self.check_move_position = rospy.Publisher('/move_position', Bool, queue_size=10) 
        # self.check_error_pos = rospy.Publisher('/check_error_pos', Float64, queue_size=10)
        # /mavros/local_position/pose
        # local_position_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
        #     SP.PoseStamped, self._local_position_callback)
        # Initialize the parameters
        # self.local_pos = [0.0] * 4
        # self.beta = [0.0] * 2
        self.ids_target = [0.0] * 2
        # self.altitude = 7.0
        self.corners = [0.0] * 4

        self.rate = rospy.Rate(20)

class ImageConverter():
    def __init__(self):
        self.__bridge = CvBridge()
        self.__image_sub = rospy.Subscriber("image_rgb_topic", Image, self.get_image_callback)
        self.cv_image

    def get_image_callback(self, image_data):
        self.cv_image = self.__bridge.imgmsg_to_cv(image_data, "bgr8")

class MarkerDetector(ImageConverter):
    def __init__(self):
        super().__init__()
        self.cam_mtx = np.array([917.497, 0.0, 635.002, 0.0, 915.865, 368.915, 0.0, 0.0, 1.0]).reshape(3,3)
        self.cam_dist = np.array([7.2697873963251586e-02, -1.4749282442847444e-01, -2.3233094539353212e-03, 8.9165121414591982e-03,-2.6332902664556002e-01])
        self.dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.param = aruco.DetectorParameters_create()
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.param.adaptiveThreshConstant = 7
        # setup matrix from imu to cam
        self.imu_cam[0][1] = -1.0
        self.imu_cam[0][3] = 0.06
        self.imu_cam[1][0] = -1.0
        self.imu_cam[1][3] = 0.04
        self.imu_cam[2][2] = -1.0
        self.imu_cam[2][3] = -0.08
        self.imu_cam[3][3] = 1.0

        # create vector tvec1, tvec2
        tvec1 = np.zeros((4,1), dtype=np.float)
        tvec1[3][0] = 1.0
        tvec2 = np.zeros((4,1), dtype=np.float)
        # tvec2[3][0] = 1.0

        # define the ids of marker
        # a = 0 # the index of first marker need to detect
        self.ids_target[0] = 11
        self.ids_target[1] = 15

    def marker_get_pose(self):
        while not rospy.is_shutdown():
            gray_image = cv2.cvtColor(self.cv_image, cv2.COLOR_RGB2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_image, self.dict, parameters=self.param)
            if np.all(ids is not None):
                for i in range(0, ids.size):
                    if ids[i][0] == self.ids_target[0]:
                        self.corners = corners[i]
                        markerLength=0.4

                        ret1 = aruco.estimatePoseSingleMarkers(corners = self.corners,
                                                                markerLength = markerLength,
                                                                cameraMatrix = self.cam_mtx,
                                                                distCoeffs = self.cam_dist)
                        rvec, tvec = ret1[0][0, 0, :], ret1[1][0, 0, :]
                        
                        (rvec - tvec).any()  # get rid of that nasty numpy value array error
                        
                        tvec1[0][0] = tvec[0]
                        tvec1[1][0] = tvec[1]
                        tvec1[2][0] = tvec[2]

                        # marker in the body (UAV frane)
                        fly_pos = PS()
                        tvec2 = np.matmul(self.imu_cam, tvec1)
                        fly_pos.pose.position.x = -tvec2[0][0]
                        fly_pos.pose.position.y = -tvec2[1][0]
                        fly_pos.pose.position.z = -tvec2[2][0]
                        # publish marker in body frame
                        self.fly_pos_pub.publish(fly_pos)

                        # if (self.local_pos[3] > -0.1 and self.local_pos[3] < 0.1):
                        marker_pos = PS()
                        tvec2 = np.matmul(self.imu_cam, tvec1)
                        marker_pos.pose.position.x = tvec2[0][0]
                        marker_pos.pose.position.y = tvec2[1][0]
                        marker_pos.pose.position.z = tvec2[2][0]
                        self.aruco_marker_pos_pub.publish(marker_pos)
                       
                        # -- Draw the detected marker and put a reference frame over it
                        # aruco.drawDetectedMarkers(frame, corners, ids)
                        aruco.drawAxis(self.cv_image, self.mtx, self.dist, rvec, tvec, 0.1)
                        str_position0 = "Marker Position in Camera frame: x=%f  y=%f  z=%f" % (tvec2[0][0], tvec2[1][0], tvec2[2][0])
                        cv2.putText(self.cv_image, str_position0, (0, 50), self.font, 0.7, (0, 255, 0), 1, cv2.LINE_AA)
                        self.rate.sleep()

            cv2.imshow("frame", self.cv_image)
            cv2.waitKey(1)

def main():
    rospy.init_node('aruco_detector', anonymous=True)

if __name__ == '__main__':
    try:
        main()
    finally:
        print("Shutting down")
