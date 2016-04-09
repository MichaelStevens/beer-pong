#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
import math
from tps import from_control_points
import IPython
from time import sleep
import serial

def to_romeo(motor_speed, angle):
    ser = serial.Serial('/dev/ttyACM0')  # open serial port
    print(ser.name)         # check which port was really used
    returner = str(motor_speed) + ' ' + str(angle)
    ser.write(returner)     # write a string
    ser.close()             # close port

class MovingAverage:
    def __init__(self, n):
        self.n = n
        self.values = []

    def clear(self):
        self.values = []

    def update(self, x):
        if len(self.values) < self.n:
            self.values.append(float(x))
        else:
            self.values = self.values[1:] + [x]

    def value(self):
        if len(self.values) == 0: return None
        return sum(self.values) / len(self.values)


class CupDetector:

    def nothing(self, x):
        pass

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.camera_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        self.camera_image = None
        self.depth_image = None
        self.interpolate = None
        cv2.namedWindow('blur')
        cv2.namedWindow('edges')
        cv2.namedWindow('image')
        cv2.createTrackbar('Thresh1', 'edges', 0, 255, self.nothing)
        cv2.createTrackbar('Thresh2', 'edges', 0, 255, self.nothing)
        cv2.createTrackbar('Blur', 'blur', 0, 10, self.nothing)
        cv2.createTrackbar('Ellipse Thresh', 'image', 0, 100, self.nothing)
        self.detections = []


    def distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0].value())**2 + (p1[1] - p2[1].value())**2)
    def get_detections(self):
        self.detections = []
        sleep(3)
        dets = [(int(x.value() + 0.5), int(y.value() + 0.5), z.value()) for (x, y, z) in self.detections]
        # for cup in dets:
        #     cv2.circle(self.camera_image, cup[0:2], 3, (0, 0, 255))


        return dets

    def camera_callback(self, data):
        try:
            self.camera_image = self.bridge.imgmsg_to_cv2(data)
            self.camera_image = cv2.cvtColor(self.camera_image, cv2.COLOR_RGB2BGR)
        except CvBridgeError, e:
            print e

        if self.depth_image != None:
            thresh1 = cv2.getTrackbarPos('Thresh1', 'edges')
            if thresh1 == 0: thresh1 = 100
            thresh2 = cv2.getTrackbarPos('Thresh2', 'edges')
            if thresh2 == 0: thresh2 = 255
            blur = cv2.getTrackbarPos('Blur', 'blur')
            ellipse_thresh = (100 - cv2.getTrackbarPos('Ellipse Thresh', 'image')) / 100.0
            if ellipse_thresh == 1: ellipse_thresh = 0.1

            image_grey = cv2.cvtColor(self.camera_image, cv2.COLOR_RGB2GRAY)

            image_blur = cv2.GaussianBlur(image_grey, (5, 5), 1)
            image_blur = cv2.medianBlur(image_blur, 2**(blur+1) + 1)
            image_edges = cv2.Canny(image_blur, thresh1, thresh2)
            image_edges_copy = np.copy(image_edges)
            contours, hierarchy = cv2.findContours(image_edges_copy, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cunt in contours:
                if len(cunt) < 10: continue

                center, (w, h), angle = cv2.fitEllipse(cunt)
                center = tuple([int(x+0.5) for x in center])

                true_area = cv2.contourArea(cunt)
                if true_area < 100: continue
                pred_area = math.pi * w / 2.0 * h / 2.0
                if pred_area < 1: continue

                if abs(1 - true_area / pred_area) > ellipse_thresh: continue
                x = center[0]
                y = center[1]
                if self.depth_image[y][x] < 10: continue
                new_cup = (x, y, self.depth_image[y][x])

                new_detection = True
                for cup in self.detections:
                    if self.distance(new_cup, cup) < 10:
                        cup[0].update(new_cup[0])
                        cup[1].update(new_cup[1])
                        cup[2].update(new_cup[2])
                        new_detection = False
                        break
                if new_detection:
                    x = MovingAverage(13)
                    y = MovingAverage(13)
                    z = MovingAverage(13)
                    x.update(new_cup[0])
                    y.update(new_cup[1])
                    z.update(new_cup[2])
                    self.detections.append((x, y, z))

                cv2.drawContours(self.camera_image, [cunt], 0, (0, 0, 255), 3)

            for cup in self.detections:
                cv2.circle(self.camera_image, (int(cup[0].value()), int(cup[1].value())), 3, (0, 0, 255))

            cv2.imshow('blur', image_blur)
            cv2.imshow('edges', image_edges)
            cv2.imshow('image', self.camera_image)
            # #plt.imshow(self.depth_image,cmap = 'gray')
            cv2.waitKey(1)
            #attention_image = attention_map(self.camera_image)
            #p = np.argmax(attention_image)
            #(x, y, z) = self.index2point(p)

            #if not (x == 0 and y == 0 and z == 0) and y > 0:
            #    self.x_smooth.update(x)
            #    self.y_smooth.update(y)
            #    self.z_smooth.update(z)



    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError, e:
            print e

        if self.depth_image != None:
            pass
            #thresh1 = cv2.getTrackbarPos('Thresh1', 'image')
            #thresh2 = cv2.getTrackbarPos('Thresh2', 'image')
            #edges = cv2.Canny(self.depth_image, thresh1, thresh2)

            #cv2.imshow('image', edges)
            #plt.imshow(self.depth_image,cmap = 'gray')
            #cv2.waitKey(1)


    def index2point(self, index):
        Z = self.depth_image.item(index)
        x = index % self.depth_image.shape[1]
        y = int(index / self.depth_image.shape[1])
        X = (Z / self.fx) * (x - self.cx)
        Y = (Z / self.fy) * (y - self.cy)
        return (X / 1000., Y / 1000., Z / 1000.)

def main(args):
    rospy.init_node('int_point_pub', anonymous=True)
    cup_detector = CupDetector()
    IPython.embed()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"



if __name__ == '__main__':
    main(sys.argv)
