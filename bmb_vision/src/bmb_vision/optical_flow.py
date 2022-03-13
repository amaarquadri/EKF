import rospy
from bmb_msgs.msg import OpticalFlowReading
from sensor_msgs.msg import Image

import cv2
import numpy as np
import matplotlib.pyplot as plt
from time import perf_counter


class OpticalFlow:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/rectified_image", Image, self.image_callback)
        self.optical_flow_pub = rospy.Publisher("/optical_flow_reading", OpticalFlowReading, queue_size=1)
        self.prvs = None

    def image_callback(self, image):
        # converting ROS message to opencv image
        frame2 = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        next = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

        if self.prvs is not None:
            flow = cv2.calcOpticalFlowFarneback(self.prvs, next, None, 0.5, 3, 15, 3, 5, 1.2, 0)
            if False:  # np.var(flow) > ? ---- TODO: still determining best way to do variance formula. Will update.
                velocity = (0, 0)
            else:
                velocity = np.mean(flow, axis=(0, 1))
        else:
            velocity = (0, 0)

        self.prvs = next

        optical_flow_reading.x_pixel_velocity = velocity[0]
        optical_flow_reading.y_pixel_velocity = velocity[1]
        self.optical_flow_pub.publish(optical_flow_reading)

    @staticmethod
    def spin():
        rospy.spin()
