import cv2
import numpy as np
from networktables import NetworkTables
import math


def nothing(x):
    pass


ip = "127.0.0.1" 
cap = cv2.VideoCapture('images/rocket/testvideo.mp4')


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

while True:
    
    ret, frame = cap.read()
    
    
    lower_green = np.array([55,97,177]) # 55 97 177 HSV  121 135 5 RGB
    upper_green = np.array([255,255,255]) #255 255 255 HSV 181 255 95 RGB
    
    test_image = frame
    #test_image =  cv2.imread('images/cargo/CargoStraightDark90in.jpg')
    if len(frame):
    
        
        brightness = 1
        contrast = 100
        test_image = np.int16(test_image)
        test_image = test_image * (contrast/127+1) - contrast + brightness
        test_image = np.clip(test_image, 0, 255)
        test_image = np.uint8(test_image)
        
        cv2.imshow('Test', test_image)
        test_image = cv2.cvtColor(test_image, cv2.COLOR_BGR2HSV)
        
        resizedImage = cv2.resize(test_image,(300,300))
        # resizedImage = resizedImage[125:300, 0:300]
        resizedImage = resizedImage[0:150, 0:300]
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
        openedImage = cv2.morphologyEx(maskedImage, cv2.MORPH_OPEN, kernel)
       # kernel = np.ones((3,3),np.uint8)
       # openedImage = cv2.morphologyEx(openedImage, cv2.MORPH_CLOSE, kernel)
        cv2.imshow('Opened', openedImage)
        cv2.imshow('Masked', maskedImage)
        
        contourImage, contours, hierarchy = cv2.findContours(maskedImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #cv2.CHAIN_APPROX_SIMPLE)
        contourIndexes =  []
        contourCounter = 0
        for contour in contours:
            area =  cv2.contourArea(contour)
            if(area > 50 and area < 200):
                contourIndexes.append(contourCounter)
            contourCounter += 1
            # print(contourIndexes)
        if(len(contourIndexes) >= 2):
            cnt = contours[contourIndexes[-1]]
            cnt2 = contours[contourIndexes[-2]]
            # cv2.drawContours(resizedImage,[cnt],0,(255,0,0),4)
            # cv2.drawContours(resizedImage,[cnt2],0,(255,0,0),4)
            x,y,w,h = cv2.boundingRect(cnt) 
            x2,y2,w2,h2 = cv2.boundingRect(cnt2)
            midPoint = int(((x+x2+w2)/2))
            midPointY = y+h/2
            # print(midPointY)
            
            
            box1, angle1 = findContourAngle(cnt)
            box2, angle2 = findContourAngle(cnt2)
            angle2 = 90 - angle2
            
            # print(angle1, angle2)
            
            if (abs(angle1-angle2) < 5):
                cv2.drawContours(resizedImage,[box1],0,(255,0,0),2)
                cv2.drawContours(resizedImage,[box2],0,(255,0,0),2)
                cv2.line(resizedImage,(midPoint,0),(midPoint,300), (0,0,255))
                cv2.line(resizedImage,(0,midPointY),(300,midPointY), (0,0,255))
            
            
            # calculate yaw
            cameraHorizAngle = 61.0
            pixelToAngle = float(300/cameraHorizAngle)
            
            moc = 150 # moc stands for middle of the camera
            angleDiff = 0
            if(moc-midPoint >= 0):
                angleDiff = moc-midPoint
                pixelToAngle = -pixelToAngle
            else:
                angleDiff = midPoint-moc
                
                 
            angleToTurn = float(angleDiff/pixelToAngle)
            sd.putNumber('angle', angleToTurn)

            # calculate pitch -- emre
            
            cameraVeritcalAngle = 45.6  
            pixelToDistance = float(300/cameraVeritcalAngle)
            moc = 150 # moc stands for middle of the camera
            distanceAngleDiff = 0
            if(moc-midPointY >= 0):
                distanceAngleDiff = moc-midPointY
                pixelToDistance = -pixelToDistance
            else:
                distanceAngleDiff = midPointY-moc
              
            distanceAngleDiff = float(distanceAngleDiff/pixelToDistance)     
            h = 26.2 # to be changed
            distanceAngleDiff = math.radians(distanceAngleDiff)
            distanceToGo = abs(h/math.tan(distanceAngleDiff))
            print(distanceToGo)
            
            # calculate pitch v2 -- emre
            # https://wpilib.screenstepslive.com/s/currentCS/m/vision/l/288985-identifying-and-processing-the-targets    
            
        cv2.imshow('Last Image', resizedImage)

    
    keyPressed = cv2.waitKey(1)
    if keyPressed == 27:
        break
cv2.destroyAllWindows() # Destroy the windows and close the program
