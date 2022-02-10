#!/usr/bin/env python3

import cv_bridge 
import rospy
from sensor_msgs.msg import Image

class Depth_Retriever():
    def __init__(self):

        self.bridge = cv_bridge.CvBridge()

        self.sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.callback)

    def callback(self,ros_pics):
        try: 
            my_image = self.bridge.imgmsg_to_cv2(ros_pics, encoding = "passthrough")

        except cv_bridge.CvBridgeError as e:
            print("CvBridge could not convert images from realsense to opencv")
        height,width, channels = my_image.shape
        my_height = my_image.shape[0]
        print(my_height)

        self.vert_len = ros_pics.height # retrieve height information from Image msg
        print(self.vert_len)
        rospy.logwarn("HERE")


def main():
    rospy.init_node("Depth",anonymous = True)

    rospy.spin()


if __name__ == '__main__':

    main()