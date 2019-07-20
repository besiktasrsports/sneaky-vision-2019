# -*- coding: utf-8 -*-
"""
Created on Mon Jun 24 22:59:43 2019

@author: emrec

This code is tested on Ubuntu 16.04 with Pyhton 2.7.12 and OpenCV 3.3.1-dev
"""

import cv2
import numpy as np
from networktables import NetworkTables
import math
from sneaky_lib.sneaky_vision import *
import config
import os
import platform


def nothing(x):
    pass

ip = config.ipAddress
NetworkTables.initialize(server = ip)
sd = NetworkTables.getTable(config.networkTableName)

if(config.imageType == "Video"):
    cap = cv2.VideoCapture(config.imageSource)
    # If the secondary webcam is inserted and we want to call the OS script
    # TODO: Test control for windows here (using import platform.system(), meanwhile you can set config.callOS to 0 from config.py file
    if(config.imageSource == 1 and platform.system() == "Linux" and config.callOS == 1):
        os.system(config.osScript)
else:
    image = cv2.imread(config.imageSource)

# If we are in trackbar mode create the trackbars
if config.CREATE_TRACKBARS:
    cv2.namedWindow('Colorbars')

    # If we are using HSV color space
    if config.camera["ColorSpace"] == "HSV":
        bh='Hue High'
        bl='Hue Low'
        gh='Saturation High'
        gl='Saturation Low'
        rh='Value High'
        rl='Value Low'
        wnd='Colorbars'
        # Begin Creating trackbars for each HSV value
        cv2.createTrackbar(bl, wnd, config.camera['H_low'],   255, nothing)
        cv2.createTrackbar(bh, wnd, config.camera['H_high'],  255, nothing)
        cv2.createTrackbar(gl, wnd, config.camera['S_low'],  255, nothing)
        cv2.createTrackbar(gh, wnd, config.camera['S_high'],  255, nothing)
        cv2.createTrackbar(rl, wnd, config.camera['V_low'],   255, nothing)
        cv2.createTrackbar(rh, wnd, config.camera['V_high'],  255, nothing)

    # If we are using BGR color space
    else:
        bh='Blue High'
        bl='Blue Low'
        gh='Green High'
        gl='Green Low'
        rh='Red High'
        rl='Red Low'
        wnd='Colorbars'
        # Begin Creating trackbars for each BGR value
        cv2.createTrackbar(bl, wnd, config.camera['B_low'],   255, nothing)
        cv2.createTrackbar(bh, wnd, config.camera['B_high'],  255, nothing)
        cv2.createTrackbar(gl, wnd, config.camera['G_low'],  255, nothing)
        cv2.createTrackbar(gh, wnd, config.camera['G_high'],  255, nothing)
        cv2.createTrackbar(rl, wnd, config.camera['R_low'],   255, nothing)
        cv2.createTrackbar(rh, wnd, config.camera['R_high'],  255, nothing)

else:
    if config.camera["ColorSpace"] == "HSV":
        lower_bound = np.array([config.camera['H_low'], config.camera['S_low'], config.camera['V_low']])
        upper_bound = np.array([config.camera['H_high'], config.camera['S_high'], config.camera['V_high']])
    else:
        lower_bound = np.array([config.camera['B_low'], config.camera['G_low'], config.camera['R_low']])
        upper_bound = np.array([config.camera['B_high'], config.camera['G_high'], config.camera['R_high']])

while True:
    
    if(config.imageType == "Video"):
        ret, image = cap.read()

    # If an image exists
    if len(image):
        
        # Congifure some camera parameters
        brightness = config.camera['Brightness']
        contrast = config.camera['Contrast']
        image = np.int16(image)
        image = image * (contrast/127+1) - contrast + brightness
        image = np.clip(image, 0, 255)
        image = np.uint8(image)


        if config.camera["ColorSpace"] == "HSV":
            image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        resizedImage = cv2.resize(image,(config.camera['Size'], config.camera['Size']))
        resizedImage = resizedImage[config.camera['CropYLow']:config.camera['CropYHigh'], config.camera['CropXLow']:config.camera['CropXHigh']]

        
        # If we are in trackbar mode read the BGR or HSV values from trackbars
        if config.CREATE_TRACKBARS:
            bLow  = cv2.getTrackbarPos(bl, wnd)
            bHigh = cv2.getTrackbarPos(bh, wnd)
            gLow  = cv2.getTrackbarPos(gl, wnd)
            gHigh = cv2.getTrackbarPos(gh, wnd)
            rLow  = cv2.getTrackbarPos(rl, wnd)
            rHigh = cv2.getTrackbarPos(rh, wnd)
        
            lower_bound = np.array([bLow,gLow,rLow])
            upper_bound = np.array([bHigh,gHigh,rHigh])

        # Filter the image according to given hsv or bgr filters
        filteredImage = cv2.inRange(resizedImage,lower_bound,upper_bound)

        if config.DISPLAY:
            cv2.imshow('Orginal', image)
            cv2.imshow('Resized Image', resizedImage)
            cv2.imshow('Filtered Image', filteredImage)

        # Find COntours
        contourImage, contours, hierarchy = cv2.findContours(filteredImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #cv2.CHAIN_APPROX_SIMPLE)
        contourIndexes =  []
        contourCounter = 0

        for contour in contours:
            area =  cv2.contourArea(contour)
            if(area > config.filter['MinArea'] and config.filter['MaxArea'] > area):
                contourIndexes.append(contourCounter)
            contourCounter += 1
        # If we have at least 2 contours
        if(len(contourIndexes) >= 2):
            # Get the first two contours
            cnt = contours[contourIndexes[-1]]
            cnt2 = contours[contourIndexes[-2]]
            
            # Get the vertical angle of the rectangles
            box1, angle1 = findContourAngle(cnt)
            box2, angle2 = findContourAngle(cnt2)
            # We are comparing angles with 90 off, so we need this
            angle2 = 90 - angle2
            # Compare the vertical distance of the rectangles if less than 5 it means we've found it
            if (abs(angle1-angle2) < 5):
                x,y,w,h = cv2.boundingRect(cnt) 
                x2,y2,w2,h2 = cv2.boundingRect(cnt2)
                # Middle point of two rectangles in X axis 
                midPointX = int(((x+x2+w2)/2))
                # Middle point of any rectangle in Y axis 
                midPointY = y+h/2
                # Draw contours around the rectangles
                cv2.drawContours(resizedImage,[box1],0,(255,0,0),4)
                cv2.drawContours(resizedImage,[box2],0,(255,0,0),4)
                # Mark the middle points
                cv2.line(resizedImage,(midPointX,0),(midPointX,300), (0,0,255))
                cv2.line(resizedImage,(0,midPointY),(300,midPointY), (0,0,255))
                # Calculate yaw angle to target
                yaw_diff =  getAngleToTarget(config.camera['HFOV'],
                                            midPointX,
                                            config.camera['Size'])
                # Calculate distance to target
                distance_diff = getDistanceToTarget(config.camera['HeightDiff'], 
                                config.camera['MountAngle'], 
                                config.camera['VFOV'],
                                midPointY,
                                config.camera['Size'])
                # Put a text indicating the angle
                cv2.putText(resizedImage,"Angle :", (x,y-25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255))
                cv2.putText(resizedImage,str(yaw_diff), (x,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255))

                if config.DEBUG:
                    print("Yaw Angle: " ,str(yaw_diff))
                    print("Distance: "  , str(distance_diff))
                # Write the output to networktables
                sd.putNumber('angle', yaw_diff)
                sd.putNumber('distance', distance_diff)
                
                # TODO: calculate distance v2 -- emre
                # https://wpilib.screenstepslive.com/s/currentCS/m/vision/l/288985-identifying-and-processing-the-targets    

        if config.DISPLAY:   
            cv2.imshow('Final Image', resizedImage)
    keyPressed = cv2.waitKey(1)
    if keyPressed == 27:
        break
cv2.destroyAllWindows() # Destroy the windows and close the program
