import cv2
import numpy as np
#import imutils
import math
import os
#import imutils
import random
# from sklearn.metrics import mean_squared_error
# from sklearn import linear_model
# from PIL import Image
# import matplotlib.image as mpimg 
import matplotlib.pyplot as plt

def detectAndDescribe(image1, image2):
    gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)
    sift = cv2.SIFT_create()
    kp1 = sift.detect(gray1,None)
    kp2 = sift.detect(gray2,None)
    img1=cv2.drawKeypoints(gray1,kp1,image1)
    img2=cv2.drawKeypoints(gray2,kp2,image2)

    images = np.hstack((img1, img2))

    while(True):
        plt.imshow(images)
        plt.show()

path1 = './my_image_1.jpg'
path2 = './my_image_2.jpg'
img1 = cv2.imread(path1)
img2 = cv2.imread(path2)


detectAndDescribe(img1, img2)