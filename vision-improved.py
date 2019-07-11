# -*- coding: utf-8 -*-
"""
Created on Mon Jun 24 22:59:43 2019

@author: emrec
"""

import cv2
import numpy as np
from networktables import NetworkTables
import math
import shapedetector

def nothing(x):
    pass

sdetect = shapedetector.ShapeDetector()

ip = "10.99.99.2" 
cap = cv2.VideoCapture(1)# 'images/rocket/testvideo.mp4'

cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)

cv2.namedWindow('Colorbars')

# Assign strings for ease of coding"
bh='Blue High'
bl='Blue Low'
gh='Green High'
gl='Green Low'
rh='Red High'
rl='Red Low'
wnd='Colorbars'

# Begin Creating trackbars for each BGR value
cv2.createTrackbar(bl, wnd, 63,   255, nothing)
cv2.createTrackbar(bh, wnd, 137 ,   255, nothing)
cv2.createTrackbar(gl, wnd, 127,   255, nothing)
cv2.createTrackbar(gh, wnd, 255,   255, nothing)
cv2.createTrackbar(rl, wnd, 77,   255, nothing)
cv2.createTrackbar(rh, wnd, 255,   255, nothing)


NetworkTables.initialize(server = ip)
sd = NetworkTables.getTable("visiontable")

"""
def getShape(cnt):
    cnt = imutils.grab_contours(cnt)
    return sd.detect(cnt)
"""
    
def findContourAngle(cnt):
    rect = cv2.minAreaRect(cnt)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    
    
    angleX1 = box[2][0]
    angleX2 = box[3][0]
    angleY1 = box[2][1]
    angleY2 = box[3][1]
    
    angle = math.atan2(float((angleY2 - angleY1)),float((angleX2 - angleX1)))
    angle = math.degrees(angle)
    return box, angle

def getYawAngleOfTarget(_moc,fov,midPoint):
    cameraHorizAngle = fov
    pixelToAngle = float(300/cameraHorizAngle)
    
    moc = 150 # moc stands for middle of the camera
    # print(midPoint)
    angleDiff = 0
    if(moc-midPoint >= 0):
        angleDiff = moc-midPoint
        pixelToAngle = -pixelToAngle
    else:
        angleDiff = midPoint-moc
    angleToTurn = float(angleDiff/pixelToAngle)
    if angleToTurn == 0:
        return 0.0
    
    return angleToTurn

def getDistanceToTarget(_moc,fov,midPointY):
    cameraVeritcalAngle = fov # 45.6  
    print("MidPoint: " + str(midPointY))
    pixelToDistance = float(300.0/cameraVeritcalAngle)
    print("Pixel To Distance: " + str(pixelToDistance))
    moc = _moc # moc stands for middle of the camera 150
    distanceAngleDiff = 0
    if(moc-midPointY >= 0):
        distanceAngleDiff = moc-midPointY
        pixelToDistance = -pixelToDistance
    else:
        distanceAngleDiff = midPointY-moc
      
    distanceAngleDiff = float(distanceAngleDiff/pixelToDistance)
    print("Angle: " + str(distanceAngleDiff))
    h = 15 # to be changed
    distanceAngleDiff = math.radians(distanceAngleDiff)
    if math.tan(distanceAngleDiff) == 0:
        distanceToGo = 0
    else:
        # Add mounting angle to this
        mountingAngle = math.radians(-5.0)
        a1a2 = abs(mountingAngle+distanceAngleDiff)
        print("Total Angle: " + str(math.degrees(a1a2)))
        distanceToGo = abs(h/math.tan(a1a2))
    
    return distanceToGo

while True:
    
    ret, frame = cap.read()
    
    
    lower_green = np.array([59,244,116]) # 55 97 177 HSV  121 135 5 RGB
    upper_green = np.array([74,255,255]) #255 255 255 HSV 181 255 95 RGB
    
    test_image = frame
    
    # ratio = test_image.shape[0] / float(resized.shape[0])

    #test_image =  cv2.imread('images/cargo/CargoStraightDark90in.jpg')
    if len(frame):
    
        
        brightness = 1
        contrast = 1000
        test_image = np.int16(test_image)
        test_image = test_image * (contrast/127+1) - contrast + brightness
        test_image = np.clip(test_image, 0, 255)
        test_image = np.uint8(test_image)
        
        cv2.imshow('Test', test_image)
        test_image = cv2.cvtColor(test_image, cv2.COLOR_BGR2HSV)
        
        resizedImage = cv2.resize(test_image,(300,300))
        resizedImage = resizedImage[125:300, 0:300]
        #resizedImage = resizedImage[0:150, 0:300]
        cv2.imshow('Resized Image', resizedImage)
        staticColorImage = cv2.inRange(resizedImage,lower_green,upper_green)
        cv2.imshow('Static Image', staticColorImage)

       
        
        bLow  = cv2.getTrackbarPos(bl, wnd)
        bHigh = cv2.getTrackbarPos(bh, wnd)
        gLow  = cv2.getTrackbarPos(gl, wnd)
        gHigh = cv2.getTrackbarPos(gh, wnd)
        rLow  = cv2.getTrackbarPos(rl, wnd)
        rHigh = cv2.getTrackbarPos(rh, wnd)
    
    
        rgbLow=np.array([bLow,gLow,rLow])
        rgbHigh=np.array([bHigh,gHigh,rHigh])
        
        maskedImage = cv2.inRange(resizedImage, rgbLow, rgbHigh)
        kernel = np.ones((3,3),np.uint8)
        openedImage = cv2.morphologyEx(staticColorImage, cv2.MORPH_OPEN, kernel)
       # kernel = np.ones((3,3),np.uint8)
       # openedImage = cv2.morphologyEx(openedImage, cv2.MORPH_CLOSE, kernel)
        cv2.imshow('Opened', openedImage)
        cv2.imshow('Masked', maskedImage)
        
        contourImage, contours, hierarchy = cv2.findContours(staticColorImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #cv2.CHAIN_APPROX_SIMPLE)
        contourIndexes =  []
        contourCounter = 0

            
        
        for contour in contours:
            area =  cv2.contourArea(contour)
            if(area > 50 and 300 > area):
                contourIndexes.append(contourCounter)
            contourCounter += 1
            # print(contourIndexes)
        if(len(contourIndexes) >= 2):
            cnt = contours[contourIndexes[-1]]
            cnt2 = contours[contourIndexes[-2]]
            # if len(cnt) == 2 or len(cnt) == 3:
                # print(contours)
                # cnts = imutils.grab_contours(cnt)
            shape = sdetect.detect(cnt)
            #print("1:" + shape)
            shape2 = sdetect.detect(cnt2)
            #print("2:" + shape2)
            # print("1 :" + getShape(cnt))
            # print("2: " + getShape(cnt2))
            cv2.drawContours(resizedImage,[cnt],0,(255,0,0),4)
            cv2.drawContours(resizedImage,[cnt2],0,(255,0,0),4)
            x,y,w,h = cv2.boundingRect(cnt) 
            x2,y2,w2,h2 = cv2.boundingRect(cnt2)
            midPoint = int(((x+x2+w2)/2))
            midPointY = y+h/2
            # print(midPointY)
            
            
            box1, angle1 = findContourAngle(cnt)
            box2, angle2 = findContourAngle(cnt2)
            angle2 = 90 - angle2
            
            # print(angle1, angle2)
          
            if (abs(angle1-angle2) < 3):
                cv2.drawContours(resizedImage,[box1],0,(255,0,0),2)
                cv2.drawContours(resizedImage,[box2],0,(255,0,0),2)
                cv2.line(resizedImage,(midPoint,0),(midPoint,300), (0,0,255))
                cv2.line(resizedImage,(0,midPointY),(300,midPointY), (0,0,255))
         
            
            # calculate yaw
            # print(getYawAngleOfTarget(150,80,midPoint))
            sd.putNumber('angle', getYawAngleOfTarget(150,80,midPoint))
            
            # calculate pitch -- emre
            
            print(getDistanceToTarget(150,64,midPointY+125))
           
            
            # calculate pitch v2 -- emre
            # https://wpilib.screenstepslive.com/s/currentCS/m/vision/l/288985-identifying-and-processing-the-targets    
            
        cv2.imshow('Last Image', resizedImage)

    
    keyPressed = cv2.waitKey(1)
    if keyPressed == 27:
        break
cv2.destroyAllWindows() # Destroy the windows and close the program
