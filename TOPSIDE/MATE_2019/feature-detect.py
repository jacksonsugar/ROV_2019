#!/usr/bin/env python

import numpy as np
import cv2
import sys
import imutils
from skimage import exposure
import time

font = cv2.FONT_HERSHEY_COMPLEX

pipeline_string = "udpsrc port=5000 ! application/x-rtp,encoding-name=JPEG,payload=13 ! rtpjpegdepay ! jpegdec ! videoconvert ! appsink "

video_capture = cv2.VideoCapture(pipeline_string, cv2.CAP_GSTREAMER)

numtri = 0
numsqr = 0
numcir = 0
numlin = 0

while True:

	ret, img = video_capture.read()
	imCopy = img.copy()
	imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

	imgray = cv2.bilateralFilter(imgray, 11, 17, 17) #
	edged = cv2.Canny(imgray, 30, 200)             #

	cnts = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:20]
	screenCnt = None

	for cnt in cnts:
	    epsilon = 0.025*cv2.arcLength(cnt,True)
	    approx = cv2.approxPolyDP(cnt,epsilon,True)
	    cv2.drawContours(img, [approx], 0, (0), 5)
	    x = approx.ravel()[0]
	    y = approx.ravel()[1]

	    if len(approx) == 3:
	        cv2.putText(img, "Triangle", (x, y - 10), font, .5, (0,0,255))
	        numtri = numtri + 1
	    elif len(approx) == 4:
	        cv2.putText(img, "Rectangle", (x, y - 10), font, .5, (0,0,255))
	        numsqr = numsqr + 1
	    elif len(approx) == 5:
	        cv2.putText(img, "Pentagon", (x, y - 10), font, .5, (0,0,255))
	    elif 6 < len(approx) < 15:
	        cv2.putText(img, "Circle", (x, y - 10), font, .5, (0,0,255))
	        numcir = numcir + 1
	    else:
	        cv2.putText(img, "Line", (x, y - 10), font, .5, (0,0,255))
	        numlin = numlin + 1


	numtri = (numtri) * (.5)
	numsqr = (numsqr) * (.5)
	numcir = (numcir) * (.5)
	numlin = (numlin) * (.5)

	totTri = "%s :Triangles" % numtri
	totSqr = "%s :Squares" % numsqr
	totCir = "%s :Circles" % numcir
	totLin = "%s :Lines" % numlin

	cv2.putText(img, totTri, (0, 15), font, .5, (0,0,255))
	cv2.putText(img, totSqr, (0, 30), font, .5, (0,0,255))
	cv2.putText(img, totCir, (0, 45), font, .5, (0,0,255))
	cv2.putText(img, totLin, (0, 60), font, .5, (0,0,255))

	#cv2.drawContours(imCopy,cnts,-1,(0,255,0))
	cv2.imshow("shapes", img)

	numtri = 0
	numsqr = 0
	numcir = 0
	numlin = 0

	if cv2.waitKey(1) & 0xFF == ord('q'):
	    break


video_capture.release()
cv2.destroyAllWindows()
