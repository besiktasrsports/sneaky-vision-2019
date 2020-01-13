# Debug or not
DEBUG = 1
# Trackbar or not
CREATE_TRACKBARS = 1
# Display or not
DISPLAY = 1
# Image or Video, if "Video" is given as argument, program will use cv2.VideoCapture
# If "Image" argument is given the program will use cv2.imread
imageType = "Video"
# imageType = "Image"
# Image/Video source 0 or 1 for webcam or the file path of the video source such as 
# "images/rocket/RocketPanelStraightDark72in.jpg" or "images/rocket/testvideo.mp4"
imageSource = 0
# Ip address
ipAddress = "10.99.99.2"
# The script to make camera arrangements
osScript = "v4l2-ctl --device /dev/video0 -c auto_exposure=1 -c exposure_auto_priority=0 -c exposure_time_absolute=20 --set-fmt-video=width=160,height=120,pixelformat=MJPG -p 15 && v4l2-ctl -d1 --get-fmt-video"
# Call OS script or not, close this in WINDOWS
callOS = 1
# NetworkTable Name
networkTableName = "visiontable"
# Camera Properties
camera = { 'HFOV'        : 53.50,   # 80.0, Horizontal FOV of the camera, see camera datasheet
           'VFOV'        : 41.41,   # 64.0, Vertical FOV of the camera, see camera datasheet
           'Brightness'  : 1,      # Brightness of the image
           'Contrast'    : 1000,   # Contrast of the image
           'HeightDiff'  : 15,     # Height difference between camera and target
           'MountAngle'  : -5,     # Mounting angle of the camera need minus sign if pointing downwards
           'WidthSize'   : 320,    # Resized image width size in pixels (image becomes square)
           'HeightSize'  : 240,    # Resized image height size in pixels (image becomes square)
           'FPS'         : 15,     # FPS of the camera
           'AngleAcc'    : 360,    # 5 is normally used, you can use 360 to let the code ignore accuracy
           'SetSize'     : 0,      # Set size of the camera with cap prop
           'DoCrop'      : 0,      # Crop the image or don't
           'DoResize'    : 1,      # Resize the image or don't
           'CropXLow'    : 0,      # Lowest Point in X axis to be cropped
           'CropYLow'    : 125,    # Lowest Point in Y axis to be cropped
           'ColorSpace'  : 'HSV',  # Which color space to use  BGR, HSV or Gray
           'Gray_low'    : 127,    # Lower Gray value to be filtered
           'Gray_high'   : 255,    # Higher Gray value to be filtered
           'H_low'       : 13,     # Lower Hue value to be filtered, 55
           'H_high'      : 255,    # Higher Hue to be filtered
           'S_low'       : 25,     # Lower Saturation to be filtered, 97
           'S_high'      : 255,    # Higher Saturation to be filtered
           'V_low'       : 24,     # Lower Value to be filtered, 177
           'V_high'      : 255,    # Higher Value to be filtered
           'B_low'       : 5,      # Lower Blue value to be filtered
           'B_high'      : 95,     # Higher Blue value to be filtered
           'G_low'       : 135,    # Lower Green value to be filtered
           'G_high'      : 255,    # Higher Green value to be filtered   
           'R_low'       : 121,    # Lower Red value to be filtered
           'R_high'      : 181     # Higher Red value to be filtered
        }

filter = {  'MinArea'  : 200, # Minimum value of area filter in pixels
            'MaxArea'  : 5000 # Maximum value of area filter in pixels
        }
