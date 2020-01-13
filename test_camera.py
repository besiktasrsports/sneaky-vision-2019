import cv2
import config
import platform
import os


if(config.imageType == "Video"):
    if(platform.system() == "Linux" and config.callOS == 1):
        print("Calling OS Script")
        os.system(config.osScript)
    cap = cv2.VideoCapture(config.imageSource)
    if config.imageSource == 0 or config.imageSource == 1:
        cap.set(cv2.CAP_PROP_FPS, 15)
        if config.camera["SetSize"]:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.camera['WidthSize'])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.camera['HeightSize'])
        print("FPS Set To: ", cap.get(cv2.CAP_PROP_FPS))
else:
    image = cv2.imread(config.imageSource)

while True:
    
    if(config.imageType == "Video"):
        ret, image = cap.read()

    if config.DISPLAY:
        cv2.imshow('Original', image)

    keyPressed = cv2.waitKey(1)
    if keyPressed == 27:
        break
cv2.destroyAllWindows() # Destroy the windows and close the program