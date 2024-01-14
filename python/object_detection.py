# Source: https://how2electronics.com/esp32-cam-based-object-detection-identification-with-opencv/
#
# Prerequisites:
#   pip install numpy
#   pip install opencv-python
#   pip install cvlib
#       https://pypi.org/project/cvlib/
#   ESP-EYE_OpenCV.ino uploaded into esp32-cam
#
# opencv-python: https://pypi.org/project/opencv-python/
# cvlib: https://pypi.org/project/cvlib/
#
# We have used urllib.request to retrieve the frames from the URL and the library for image processing is OpenCV. 
# For Object detection, we have used the Cvlib library that uses an AI model for detecting objects. 
# Since the whole process requires a good amount of processing power, thus we have used multiprocessing which 
# utilizes multiple cores of our CPU.
#
# Note about ESP23cam files and code:
# Here we will not use the general ESP webserver example rather another streaming process. 
# Therefore we need to add another ESPCAM library. 
# The esp32cam library provides an object oriented API to use OV2640 camera on ESP32 microcontroller. 
# It is a wrapper of esp32-camera library: esp32cam-main.zip.
# ESP23cam file: ESP-EYE_OpenCV.ino


import cv2
import matplotlib.pyplot as plt
import cvlib as cv
import urllib.request
import numpy as np
from cvlib.object_detection import draw_bbox
import concurrent.futures
 
url='http://192.168.10.162/cam-hi.jpg'
im=None
 
def run1():
    cv2.namedWindow("live transmission", cv2.WINDOW_AUTOSIZE)
    while True:
        img_resp=urllib.request.urlopen(url)
        imgnp=np.array(bytearray(img_resp.read()),dtype=np.uint8)
        im = cv2.imdecode(imgnp,-1)
 
        cv2.imshow('live transmission',im)
        key=cv2.waitKey(5)
        if key==ord('q'):
            break
            
    cv2.destroyAllWindows()
        
def run2():
    cv2.namedWindow("detection", cv2.WINDOW_AUTOSIZE)
    while True:
        img_resp=urllib.request.urlopen(url)
        imgnp=np.array(bytearray(img_resp.read()),dtype=np.uint8)
        im = cv2.imdecode(imgnp,-1)
 
        bbox, label, conf = cv.detect_common_objects(im)
        im = draw_bbox(im, bbox, label, conf)
 
        cv2.imshow('detection',im)
        key=cv2.waitKey(5)
        if key==ord('q'):
            break
            
    cv2.destroyAllWindows()
 
 
 
if __name__ == '__main__':
    print("started")
    with concurrent.futures.ProcessPoolExecutor() as executer:
            f1= executer.submit(run1)
            f2= executer.submit(run2)