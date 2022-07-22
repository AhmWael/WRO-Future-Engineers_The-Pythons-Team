# The Pythons' Openmv Code
# Latest_Algorithm - By: Ahmad Zaki, Ahmad Wael, Aslam Said

# Servo angles:
#   Neutral --> -30
#   Hard Right --> 0
#   Light Right -->  -15
#   Hard Left --> -50
#   Light Left --> -40

# Motor functions:
#   Fast Forward --> F
#   Slow Forward --> S
#   Backward --> B
#   Stop --> Z


# These are the libraries needed for the code to function
import sensor, image, time, math, pyb
from pyb import Servo
from pyb import UART

# Thresholds to track red and green colored objects
greenthresholds = (88, 98, -95, -28, 18, 119)
redthresholds = (51, 73, 58, 121, -29, 88)
orangethresholds = (28, 61, 20, 39, 2, 99)
bluethresholds = (4, 46, 3, 21, -87, -18)
Wall_THRESHOLD = [(0, 47, -128, 12, -62, 27)]
Black_ROI = [(5,  85, 40, 150 , 0.3 ), (280, 85, 315, 150, 0.5) ];

# Defining which pins are used for TX and RX and the baud rate to communicate with Arduino Nano
uart = UART(3, 19200)

# Defining which pin is used to control the servo motor used for steering
servo_1 = Servo(1)

led1 = pyb.LED(1)
led2 = pyb.LED(2)
led3 = pyb.LED(3)
# Starting the camera sensor
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False, -1) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_auto_exposure(False, 20000)
# sensor.set_brightness(3)
#exposure_time_scale = 0.5
#current_exposure_time_in_microsecond = sensor.get_exposure_us()
#sensor.set_auto_exposure(False, exposure_us = int(current_exposure_time_in_microsecond * exposure_time_scale))
#sensor.set_contrast(-3)

# Needed variables
status = 10
red_area = -1
green_area = -1
red_blob_pos = -1
green_blob_pos = -1
wall_left = 0
wall_right = 0
area_right=0
area_left=0

# This function tracks the coloured objects and return the area of the blob and the x-coordinate of its center
# The function that start with draw are responsible for drawing the outlines of the blob in the frame buffer
def find_the_blob(thresholds) :
    blob_area = -1
    blob_cx = -1
    for blob in img.find_blobs([thresholds], pixels_threshold=100, area_threshold=1500, merge=True):
        # These values depend on the blob not being circular - otherwise they will be shaky.
        if blob.elongation() > 0.5:
            img.draw_edges(blob.min_corners(), color=0)
            img.draw_line(blob.major_axis_line(), color=0)
            img.draw_line(blob.minor_axis_line(), color=0)
        # These values are stable all the time.
        img.draw_rectangle(blob.rect(), color=127)
        img.draw_cross(blob.cx(), blob.cy(), color=127)
        # Note - the blob rotation is unique to 0-180 only.
        img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=40, color=127)
        blob_area = blob.area()
        blob_cx = blob.cx()
    return blob_area, blob_cx

servo_1.angle(-30)
uart.write("F")

while(True):
    clock.tick()
    img = sensor.snapshot()
    #img = img.gaussian(3)
    img = img.gamma_corr(gamma = 0.5, contrast = 15.0, brightness = 0.05)
    #img.median(1, percentile=0.8)
    #led1.on()
    #led2.on()
    #led3.on()


    # Finding the areas and the x-coordinates of the red and green objects by calling the function
    # The parameter given to the function is the threshold for the required color
    red_area, red_blob_pos = find_the_blob(redthresholds)
    green_area, green_blob_pos = find_the_blob(greenthresholds)
    largest_blobW = -1

    # The if condition's purpose is to know which colored object is closer and takes appropriate action
    # The larger the area the closer the object is

    if ((red_area > green_area) and (red_blob_pos >= 20)):
        print("Red is found")
        uart.write("S")
        servo_1.angle(0)

    elif ((red_area < green_area) and (green_blob_pos <= 300)):
       print("Green is found")
       uart.write("S")
       servo_1.angle(-55)

    else :
        wall_right=0
        wall_left=0
        area_right=0
        area_left=0

        for k in Black_ROI:
            blobsW = img.find_blobs([Wall_THRESHOLD[0]],roi= k[0:4])
            if blobsW:

                #print("blobsW : ", blobsW)
                # Check if a right wall is detected
                if(k[4] == 0.5):
                    largest_blobW = max(blobsW, key=lambda bL: bL.area())
                    #print("blobsW 0.5 : ", blobsW[0].area())
                    #print("largest_blobW 0.5", largest_blobW.area())
                    print("Right area: ", largest_blobW.area())
                    if(largest_blobW.area() >= 700):
                        img.draw_rectangle(largest_blobW.rect(), (0, 0, 255), 3)
                        #print("There is wall RIGHT")
                        wall_right = 1
                        area_right=largest_blobW.area()

                #print("WallRight",wall_right," K: ",k[4])
                # Check if a left wall is detected
                if(k[4] == 0.3):
                    #print("blobsW 0.3 : ", blobsW[0].area())
                    #print("largest_blobW 0.5", largest_blobW.area())
                    largest_blobW = max(blobsW, key=lambda bL: bL.area())
                    print("Left area: ", largest_blobW.area())
                    if(largest_blobW.area() >= 700):
                        img.draw_rectangle(largest_blobW.rect(), (0, 255, 255), 3)
                       # print("There is wall LEFT")
                        wall_left = 1
                        area_left=largest_blobW.area()
                #print("Area: ", blobsW[0].area())
                #print("Area: ", blobsW[1].area())
                #print("Var Wall Right: ", wall_right)
                #print("Var Wall Left: ", wall_left)

        #Condition if both right & left walls are detected
        if(wall_right is 1 and wall_left is 1):
            if(area_left-area_right >= 800):
                print("right_w_F")
                servo_1.angle(0)
                uart.write("F")
            elif(area_left-area_right <= -800):
                print("left_w_F")
                servo_1.angle(-50)
                uart.write("F")
            else:
                print("forward_w")
                servo_1.angle(-30)
                uart.write("F")

        #Condition if only the right wall is detected
        if(wall_right is 1 and wall_left is 0):
            if(area_right>=1000):
                print("left_w_strong")
                servo_1.angle(-50)
                uart.write("F")
            else:
                print("left_w_weak")
                servo_1.angle(-40)
                uart.write("S")

        #Condition if only the left wall is detected
        if(wall_right is 0 and wall_left is 1):
            if(area_left>=1000):
                print("right_w_strong")
                servo_1.angle(0)
                uart.write("F")
            else:
                print("right_w_weak")
                servo_1.angle(-15)
                uart.write("S")


