#!/usr/bin/env python3

import cv_bridge 
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image

'''
https://answers.ros.org/question/354676/how-can-i-bridge-and-display-images-from-realsense-d435-camera/
http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
http://wiki.ros.org/rqt_image_view
'''

class Depth_Retriever():
    def __init__(self):
    
        self.image_pub = rospy.Publisher("depth_topic", Image)

        # self.bridge = cv_bridge.CvBridge()

        #self.sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.callback)
        self.sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback)

    def callback(self,ros_pics):

        my_image = np.frombuffer(ros_pics.data, dtype=np.uint8).reshape(ros_pics.height, ros_pics.width, -1)

        # height,width, channels = my_image.shape
        
        cv2.imshow("Image window", my_image)
        cv2.waitKey(3)

        
        my_image = np.frombuffer(ros_pics.data, dtype=np.uint8).reshape(ros_pics.height, ros_pics.width, -1)

        self.image_pub.publish(my_image)
   
def main():

    dr = Depth_Retriever()
    rospy.init_node("Depth",anonymous = True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':

    main()