#!/usr/bin/env python

import cv2
import numpy as np
import argparse
import serial
import rospy
#import cv2.cv

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String


class RMShieldDetector:
    def __init__(self):
        # fields
        self.bridge = CvBridge()
        self.detected_shield_x = 0.0
        self.detected_shield_y = 0.0
        self.pitch_req = 0.0
        self.yaw_req = 0.0
        #self.serCoor = serial.Serial('/dev/ttyACM0', 9600)
        self.tx = [0] * 6
        self.tx[0] = 0xFA
        self.tx[1] = 0x00
        self.size = 10
        self.angleDiff = 5
        self.angleFlip = 60
        self.heightDiff = 50
        self.lengthDiff = 50
        self.ratioThresh = 4
        self.offsetThresh = 50
	self.detected_shield_x = 0
        self.detected_shield_y = 0

	# template matching
	#self.templateImg = imread('/home/joseph/Desktop/Box.png', 0)
	self.corner_x = 0
	self.corner_y = 0
	self.searchCorner_x = 0
	self.searchCorner_y = 0
	self.shield_width = 1
	self.cropImage = None
	self.searchArea = None
	self.cropImageNull = True


	# ros
        self.sub_image_raw = rospy.Subscriber('/usb_cam/image_raw', Image, self.handle_image_raw)
        rospy.init_node('rm_shield_detector', anonymous=True) #initializes this program as a node

        rospy.spin()

    def handle_image_raw(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)


        # 1) generate threshold image
        img_b, img_g, img_r = cv2.split(img)
        ret, img_thresh = cv2.threshold(img_b, 227, 255, cv2.THRESH_BINARY)

        # 2) morphological operations
        img_2 = cv2.dilate(img_thresh, None, iterations=2)
        img_2 = cv2.erode(img_2, None, iterations=2)

        # 3) contour detection
        contours = cv2.findContours(img_2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        contours = sorted(contours, key=cv2.contourArea, reverse=True)




        # draws contours
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.size:
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
		box = np.int0(box)

                #box = np.int0(cv2.cv.BoxPoints(rect))
		#box = np.int0(box)
                cv2.drawContours(img, [box], 0, (0, 0, 255), 2)

        rectsRaw = []
        rects = []
	# ** Template Matching
	if (self.cropImageNull is False):
   	    res = cv2.matchTemplate(img, self.searchArea, cv2.TM_CCOEFF_NORMED)
   	    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
	    top_left = max_loc
    	    bottom_right = (int(top_left[0] + self.shield_width + 25), int(top_left[1] + self.shield_width + 25))
	    top_left_int = (int(top_left[0]), int(top_left[1]))
	    cv2.rectangle(img,top_left_int, bottom_right, 255, 2)


        # detect contours
        if (len(contours) > 0):
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > self.size:
                    rect = cv2.minAreaRect(cnt)
                    ((rect_x, rect_y), (rect_w, rect_h), rect_angle) = rect
                    rectsRaw.append(Rectangle(rect_x, rect_y, rect_w, rect_h, rect_angle))

            rctCountFilter = 0
            for rct in rectsRaw:
                if (rctCountFilter + 1 <= len(rectsRaw)):
                    filterRatio1 = rectsRaw[rctCountFilter].h / rectsRaw[rctCountFilter].w
                    if (abs(rectsRaw[rctCountFilter].angle) > self.angleFlip):
                        filterRatio1 = rectsRaw[rctCountFilter].w / rectsRaw[rctCountFilter].h
                    if (filterRatio1 > self.ratioThresh): #or filterRatio2 > self.ratioThresh
                        rects.append(Rectangle(rectsRaw[rctCountFilter].x, rectsRaw[rctCountFilter].y, rectsRaw[rctCountFilter].w, rectsRaw[rctCountFilter].h, rectsRaw[rctCountFilter].angle))
                        cv2.circle(img,(int(rectsRaw[rctCountFilter].x), int(rectsRaw[rctCountFilter].y)), 5, (255,0,0), -1)

                    rctCountFilter = rctCountFilter + 1

	    rctCountFilter = 0
            rects = sorted(rects, key=Rectangle.getKey)
            rctCount = 0



	    # threshold debugging
	    for rct in rects:
		if (rctCount +1 < len(rects)):
		    angle1 = rects[rctCount].angle
                    angle2 = rects[rctCount + 1].angle
		    print("angle1 : ", angle1, "deg")
		    print("angle2 : ", angle2, "deg")

                    if (abs(rects[rctCount].angle) > self.angleFlip):
                        angle1 = 90 - abs(rects[rctCount].angle)
                    if (abs(rects[rctCount + 1].angle) > self.angleFlip):
                        angle2 = 90 - abs(rects[rctCount + 1].angle)

                    angle_diff = angle1 - angle2
                    angle_diff = abs(angle_diff)
		    print("angle diff:", angle_diff)
		    if (angle_diff <self.angleDiff):
		        cv2.circle(img, (40, 440), 10, (255,0,0), -1)

    		    ratio1 = rects[rctCount].h / rects[rctCount].w
	       	    ratio2 = rects[rctCount + 1].h / rects[rctCount + 1].w
		    length1 = rects[rctCount].h
		    length2 = rects[rctCount + 1].h
		    length3 = rects[rctCount - 1].h

		    x1 = rects[rctCount].w
		    x2 = rects[rctCount + 1].w
		    y1 = rects[rctCount].h
		    y2 = rects[rctCount + 1].h

		    if (abs(rects[rctCount].angle) > self.angleFlip):
	     	        ratio1 = rects[rctCount].w / rects[rctCount].h
		        length1 = rects[rctCount].w
		        w1 = rects[rctCount].h
		        h1 = rects[rctCount].w
		    if (abs(rects[rctCount + 1].angle) > self.angleFlip):
		        ratio2 = rects[rctCount + 1].w / rects[rctCount + 1].h
		        length2 = rects[rctCount + 1].w
		        w2 = rects[rctCount + 1].h
		        h2 = rects[rctCount + 1].w

		    heightDiff = rects[rctCount].y - rects[rctCount + 1].y # height difference between the two rects
		    if (abs(heightDiff) < self.heightDiff):
		        cv2.circle(img, (80, 440), 10, (0,255,0), -1)

		    lengthDiff = length1 - length2 # length difference between the two rects

		    if (abs(lengthDiff) < self.lengthDiff):
		        cv2.circle(img, (120, 440), 10, (0,0,255), -1)

		    offset = 135 / 50 * length1
		    offsetDiff = rects[rctCount + 1].x - (rects[rctCount].x + offset)
		    if (abs(offsetDiff) < self.offsetThresh):
		        cv2.circle(img, (160, 440), 10, (0,100,100), -1)


		rctCount = rctCount + 1

	    # shield detection
	    rctCount1 = 0
            for rct in rects:
                if (rctCount1 + 1 < len(rects)):
		    print("STAGE 0 | rctCount1 : PASS")
                    #print(mousedata)
                    angle1 = rects[rctCount1].angle
                    angle2 = rects[rctCount1 + 1].angle
		    print("angle1 : ", angle1, "deg")
		    print("angle2 : ", angle2, "deg")

                    if (abs(rects[rctCount1].angle) > self.angleFlip):
                        angle1 = 90 - abs(rects[rctCount1].angle)
                    if (abs(rects[rctCount1 + 1].angle) > self.angleFlip):
                        angle2 = 90 - abs(rects[rctCount1 + 1].angle)

                    angle_diff = angle1 - angle2
                    angle_diff = abs(angle_diff)
		    print("angle diff:", angle_diff)

                    if (angle_diff < self.angleDiff):
                        print("STAGE  1 | angle_diff : PASS")
                        # if angle is in range between 60~90 swift width and height.
                        ratio1 = rects[rctCount1].h / rects[rctCount1].w
                        ratio2 = rects[rctCount1 + 1].h / rects[rctCount1 + 1].w
                        print(ratio1, " ", ratio2)

                        #flip the ratios (height and width changes when angle of box < 90)
                        length1 = rects[rctCount1].h
                        length2 = rects[rctCount1 + 1].h

                        x1 = rects[rctCount1].w
                        x2 = rects[rctCount1 + 1].w
                        y1 = rects[rctCount1].h
                        y2 = rects[rctCount1 + 1].h

                        if (abs(rects[rctCount1].angle) > self.angleFlip):
                            ratio1 = rects[rctCount1].w / rects[rctCount1].h
                            length1 = rects[rctCount1].w
                            w1 = rects[rctCount1].h
                            h1 = rects[rctCount1].w
                        if (abs(rects[rctCount1 + 1].angle) > self.angleFlip):
                            ratio2 = rects[rctCount1 + 1].w / rects[rctCount1 + 1].h
                            length2 = rects[rctCount1 + 1].w
                            w2 = rects[rctCount1 + 1].h
                            h2 = rects[rctCount1 + 1].w

                        heightDiff = rects[rctCount1].y - rects[rctCount1 + 1].y # height difference between the two rects
                        print("height difference: ", heightDiff)

                        lengthDiff = length1 - length2 # length difference between the two rects
                        print("length diference : ", lengthDiff)

                        offset = 135 / 50 * length1
                        offsetDiff = rects[rctCount1 + 1].x - (rects[rctCount1].x + offset)

                        if (abs(offsetDiff) < self.offsetThresh):
                            print("STAGE  3 | offsetDiff : PASS")
                            if (abs(lengthDiff) < self.lengthDiff):
                                if (abs(heightDiff) < self.heightDiff):
                                    print("STAGE  4 | heightDiff : PASS || lengthDiff : PASS")
                                    coordinate_x = abs(rects[rctCount1].x - rects[rctCount1 + 1].x) / 2 + rects[rctCount1].x
                                    coordinate_y = abs(rects[rctCount1].y - rects[rctCount1 + 1].y) / 2 + min(rects[rctCount1].y, rects[rctCount1 + 1].y)

                                    self.detected_shield_x = coordinate_x
                                    self.detected_shield_y = coordinate_y
                                    cv2.circle(img,(int(coordinate_x), int(coordinate_y)), 5, (0,255,0), -1)

				    # image for template matching
				    self.shield_width = abs(rects[rctCount1].x - rects[rctCount1 + 1].x)

				    self.corner_x = rects[rctCount1].x - 10
				    self.corner_y = rects[rctCount1].y - (self.shield_width / 2) - 10 # goes to negative
				    if (self.corner_x < 0):
					self.corner_x = 0
				    if (self.corner_y < 0):
					self.corner_y = 0
				    self.cropImage = img[int(self.corner_y):int(self.corner_y + self.shield_width + 25), int(self.corner_x):int(self.corner_x + self.shield_width + 25)]

				    self.searchCorner_x = self.corner_x - 100
				    self.searchCorner_y = self.corner_y - 100
				    if (self.searchCorner_x < 0):
					self.searchCorner_x = 0
				    if (self.searchCorner_y < 0):
					self.searchCorner_y = 0
				    self.searchArea = img[int(self.searchCorner_y):int(self.corner_y + self.shield_width + 100), int(self.searchCorner_x):int(self.corner_x + self.shield_width + 100)]

				    self.cropImageNull = False

        			    cv2.imshow('template', self.cropImage)
				    cv2.imshow('searchArea', self.searchArea)

                                    degX = (self.detected_shield_x - 320) / (320 /  75)
                                    degY = (self.detected_shield_y) / (240 / 75)
                                    intDegX = int(degX)
                                    intDegY = int(degY)

                                    print("int deg = ", intDegX, intDegY)

                                    self.tx[2] = (intDegX >> 8) & 255 # bitshift by 8 to right
                                    self.tx[3] = (intDegX) & 255
                                    self.tx[4] = (intDegY >> 8) &255
                                    self.tx[5] = (intDegY) & 255
                                    #self.serCoor.write(bytearray(self.tx))

		    rctCount1 = rctCount1 + 1

###############

        cv2.imshow('result image', img)
	cv2.waitKey(3)


class Rectangle:
    def __init__(self, x, y, w, h, angle):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.angle = angle

    def getKey(self):
        return self.x

if __name__ == '__main__':
    try:
        RMShieldDetector()
    except rospy.ROSInterruptException:
        pass




###

# Log 02/23/17
# Initialized template matching altorithm.
# Need variable threshold dependence on distance
# Method to find distance (sensors, lightstrip heights)
# Need initialization of search area for less process-heavy
# Move to TX1
