import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2 as cv
import glob
from cv_bridge import CvBridge


class Defisheye:
    def __init__(self):
        self.raw_image_sub = rospy.Subscriber("/raw_image", Image, self.raw_image_callback)
        self.rectified_image_pub = rospy.Publisher("/rectified_image", Image, queue_size=1)

    def raw_image_callback(self, raw_image):
        # Camera Parameters
        # parameters based on goPro Hero10, can be calibrated using file name cameracalibration
        camera_matrix = np.array([[695.76215217, 0, 578.14392399],
                                  [0, 700.61833152, 414.92365056],
                                  [0, 0, 1]])
        dist = np.array([[0.06150531, -0.31313744, -0.00593448, 0.00619686, 0.40787546]])

        # converting ROS message to open
        img = bridge.imgmsg_to_cv2(raw_image, desired_encoding='passthrough')

        h, w = img.shape[:2]
        new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(camera_matrix, dist, (w, h), 1, (w, h))

        # Undistort
        dst = cv.undistort(img, camera_matrix, dist, None, new_camera_matrix)

        # crop the image
        x, y, w, h = roi
        dst = dst[y:y + h, x:x + w]

        # Undistort with Remapping
        mapx, mapy = cv.initUndistortRectifyMap(camera_matrix, dist, None, new_camera_matrix, (w, h), 5)
        dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)

        undistorted = cv.imwrite('undistorted.png', dst)

        # convert opencv image to ros message
        bridge = CvBridge()
        rectified_image = bridge.cv2_to_imgmsg(undistorted, encoding="passthrough")

        self.rectified_image_pub.publish(rectified_image)

    @staticmethod
    def spin():
        rospy.spin()
