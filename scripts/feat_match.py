import cv2
import sys
import os.path
import numpy as np

def drawMatches(img1, kp1, img2, kp2, matches):

	rows1 = img1.shape[0]
	cols1 = img1.shape[1]
	rows2 = img2.shape[0]
	cols2 = img2.shape[1]

	out = np.zeros((max([rows1,rows2]),cols1+cols2,3), dtype='uint8')
	out[:rows1,:cols1] = np.dstack([img1])
	out[:rows2,cols1:] = np.dstack([img2])
	for mat in matches:
		img1_idx = mat.queryIdx
		img2_idx = mat.trainIdx
		(x1,y1) = kp1[img1_idx].pt
		(x2,y2) = kp2[img2_idx].pt

		cv2.circle(out, (int(x1),int(y1)), 4, (255, 0, 0, 1), 1)   
		cv2.circle(out, (int(x2)+cols1,int(y2)), 4, (255, 0, 0, 1), 1)
		cv2.line(out, (int(x1),int(y1)), (int(x2)+cols1,int(y2)), (255, 0, 0, 1), 1)

	return out

def compare(filename1, filename2):
	img1 = cv2.imread(filename1)          # queryImage
	img2 = cv2.imread(filename2)          # trainImage

	# Initiate SIFT detector
	# detector = cv2.FeatureDetector_create("SIFT")
	# descriptor = cv2.DescriptorExtractor_create("SIFT")
	
	sift = cv2.xfeatures2d.SIFT_create()

	# find the keypoints and descriptors with SIFT
	# kp1 = detector.detect(img1)
	# kp1, des1 = descriptor.compute(img1, kp1)
	
	# kp2 = detector.detect(img2)
	# kp2, des2 = descriptor.compute(img2, kp2)

	kp1, des1 = sift.detectAndCompute(img1,None)
	kp2, des2 = sift.detectAndCompute(img2,None)

	# BFMatcher with default params
	bf = cv2.BFMatcher()
	# matches = bf.match(des1,des2)
	matches = bf.knnMatch(des1, des2, k = 2)
	matches = list(map(lambda a: [a[0]], filter(lambda a: a[0].distance < 0.75*a[1].distance, matches)))
	
	# matches = sorted(matches, key=lambda val: val.distance)
	matches = sorted(matches, key=lambda val: val[0].distance)

	# img3 = drawMatches(img1,kp1,img2,kp2,matches[:25])
	img = np.zeros([max(img1.shape[0],img2.shape[0]),img1.shape[1]+img2.shape[1],3])
	img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,matches,img,flags=2)

	# Show the image
	cv2.imshow('Matched Features', img3)
	cv2.waitKey(0)
	# cv2.destroyWindow('Matched Features')

if len(sys.argv) != 3:
	sys.stderr.write("usage: compare.py <queryImageFile> <sourceImageFile>\n")
	sys.exit(-1)
	
# compare(sys.argv[1], sys.argv[2])