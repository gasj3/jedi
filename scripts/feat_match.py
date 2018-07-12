#!/usr/bin/env python
"""OpenCV feature detectors with ros CompressedImage Topics in python.

This example subscribes to a ros topic containing sensor_msgs 
CompressedImage. It converts the CompressedImage into a numpy.ndarray, 
then detects and marks features in that image. It finally displays 
and publishes the new image - again as CompressedImage topic.
"""
__author__ = 'Simon Haller <simon.haller at uibk.ac.at>'
__version__ = '0.1'
__license__ = 'BSD'

# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros messages
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

VERBOSE=False

class ImageFeature:

	def __init__(self):
		'''Initialize ros publisher, ros subscriber'''
		# topic where we publish
		self.image_pub = rospy.Publisher("/vision_coca",
			String, queue_size = 10)
		# self.bridge = CvBridge()

		self.coca_img = cv2.imread("cocacola.jpg")

		# subscribed Topic
		self.subscriber = rospy.Subscriber("/xtion/rgb/image_raw/compressed",
			CompressedImage, self.callback, queue_size = 1)
		if VERBOSE:
			print "subscribed to /camera/image/compressed"


	def callback(self, ros_data):
		'''Callback function of subscribed topic.
		Here images get converted and features detected'''
		if VERBOSE:
			print 'received image of type: "%s"' % ros_data.format

		#### direct conversion to CV2 ####
		np_arr = np.fromstring(ros_data.data, np.uint8)
		image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
		# image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0

		detector = cv2.FeatureDetector_create("SIFT")
		descriptor = cv2.DescriptorExtractor_create("SIFT")

		kp1 = detector.detect(self.coca_img)
		kp1, des1 = descriptor.compute(self.coca_img, kp1)
		
		kp2 = detector.detect(image_np)
		kp2, des2 = descriptor.compute(image_np, kp2)

		bf = cv2.BFMatcher()
		matches = bf.knnMatch(des1, des2, k = 2)
		matches = list(map(lambda a: [a[0]], filter(lambda a: a[0].distance < 0.75*a[1].distance, matches)))

		matches = sorted(matches, key=lambda val: val[0].distance)

		dist = np.mean(list(map(lambda x: x[0].distance, matches[:25])))

		if dist < 185:
			msg = "Seeing cocacola"
		else:
			msg = "Cocacola not found!"

		# msg = str(dist)
		# print(msg)
		
		# if dist < 170:


		# Publish new image
		self.image_pub.publish(msg)

		# self.subscriber.unregister()


def main(args):
	'''Initializes and cleanup ros node'''
	ic = ImageFeature()
	rospy.init_node('image_feature', anonymous=True)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down ROS image feature detector module"

	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)