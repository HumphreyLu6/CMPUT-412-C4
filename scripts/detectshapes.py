#Referenced from : https://www.pyimagesearch.com/2016/02/08/opencv-shape-detection/

import imutils, numpy
import cv2, cv_bridge
from enum import Enum

class Contour(Enum):
    Unidentified = 0
    Triangle = 1
    Rectangle = 2
    Circle = 3

class ContourDetector():
    def __init__(self):
        pass

    def getContours(self, hsv, loc = 0, loc3count = 0):
        '''
        Return: green contours, red contours
        '''
        blurred_hsv = cv2.pyrMeanShiftFiltering(hsv, 15, 20)
        
        #green mask
        lower_green = numpy.array([40, 50, 50]) 
        upper_green = numpy.array([80, 255, 255])
        green_mask = cv2.inRange(blurred_hsv, lower_green, upper_green)
        
        red_mask = threshold_hsv_360(100, 50, 10, 255, 255, 340, blurred_hsv) #20, 320

        #filter out part of image by location info
        h, w, d = blurred_hsv.shape
        green_mask = mask_filter(green_mask, h, w, d, loc, loc3count)
        red_mask = mask_filter(red_mask, h, w, d, loc, loc3count)
        
        _,ori_green_countours,_ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        _,ori_red_countours,_ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        green_contours, green_items = contours_filter(ori_green_countours)
        red_contours, red_items = contours_filter(ori_red_countours)

        for item in green_items:
            cv2.drawContours(hsv, [item], -1, (255,0,0), 6)
        for item in red_items:
            #print numpy.mean(item.squeeze()[:,0]), numpy.min(item.squeeze()[:,0]), numpy.max(item.squeeze()[:,0]), w
            cv2.drawContours(hsv, [item], -1, (255,255,0), 6)

        cv2.imshow("blur_hsv", hsv)
        cv2.imshow("redmask", red_mask)
        #cv2.moveWindow(color + "mask", 710, 0)
        cv2.waitKey(4)

        return green_contours, red_contours
    
    def get_red_contour_range(self, hsv, shape = Contour.Unidentified):
        if shape == Contour.Unidentified:
            return numpy.nan,numpy.nan
        else:
            blurred_hsv = cv2.pyrMeanShiftFiltering(hsv, 15, 20)
            red_mask = threshold_hsv_360(100, 50, 10, 255, 255, 340, blurred_hsv) #20, 320
            _,ori_red_countours,_ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            red_contours, red_items = contours_filter(ori_red_countours)
            for contour, item in zip(red_contours, red_items):
                if contour == shape:
                    return numpy.min(item.squeeze()[:,0]), numpy.max(item.squeeze()[:,0])
            return numpy.nan,numpy.nan

def contours_filter(ori_contours):
    filtered_contours = []
    filtered_items = []
    for item in ori_contours:
        contour = Contour.Unidentified
        perimeter = cv2.arcLength(item, True)
        vertices = cv2.approxPolyDP(item, 0.04 * perimeter, True)

        if len(vertices) == 3:
            contour = Contour.Triangle
        elif len(vertices) == 4:
            contour = Contour.Rectangle
        else:
            contour = Contour.Circle
        if contour != Contour.Unidentified and cv2.contourArea(item) > 1000:
            #print contour, cv2.contourArea(item)
            filtered_contours.append(contour)
            filtered_items.append(item)
    return filtered_contours, filtered_items

def mask_filter(mask, h, w, d, loc = 0, loc3count = 0):
    search_top = 0
    search_bot = h
    search_left = 0
    search_right = w
    #print search_top, search_bot, search_left, search_right
    if loc == 1:
        #print "loc = 1"
        search_top = 2 * h / 3
        search_bot = h
    elif loc == 2:
        #print "loc = 2"
        search_top = 2 * h / 5
        search_bot = h
    elif loc == 3:
        #print "loc = 3"
        search_top = 1 * h / 2
        if loc3count == 3:
            search_left = 1 * w / 2
        else:
            search_left = 1 * w / 6 
            search_right = 5 * w / 6
    #print search_top, search_bot, search_left, search_right
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    mask[0:h, 0:search_left] = 0
    mask[0:h, search_right:w] = 0

    return mask

def threshold_hsv_360(s_min, v_min, h_max, s_max, v_max, h_min, hsv):
    lower_color_range_0 = numpy.array([0, s_min, v_min],dtype=float)
    upper_color_range_0 = numpy.array([h_max/2., s_max, v_max],dtype=float)
    lower_color_range_360 = numpy.array([h_min/2., s_min, v_min],dtype=float)
    upper_color_range_360 = numpy.array([360/2., s_max, v_max],dtype=float)
    mask0 = cv2.inRange(hsv, lower_color_range_0, upper_color_range_0)
    mask360 = cv2.inRange(hsv, lower_color_range_360, upper_color_range_360)
    mask = mask0 | mask360
    return mask


