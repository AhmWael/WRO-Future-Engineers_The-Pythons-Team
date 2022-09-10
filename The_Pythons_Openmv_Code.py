# The Pythons' Openmv Code
# Latest_Algorithm - By: Ahmad Zaki, Ahmad Wael, Aslam Said

# Servo angles:
#   Neutral --> -30
#   Hard Right --> 20
#   Light Right -->
#   Hard Left --> -70
#   Light Left -->

# Motor functions:
#   Motion(80, 0) --> Forward fast
#   Motion(70, 0) --> Forward slow
#   Motion(0, 70) --> Backward
#   Motion(0, 0) ---> Stop

# These are the libraries needed for the code to function
import sensor, image, time, math, pyb
from pyb import Servo, Pin, Timer

# Thresholds to track red, green, blue, orange and black colored objects
greenthresholds = (23, 74, -89, -28, 30, 127)
redthresholds = (26, 70, 49, 127, 39, 126)
bluethresholds = (6, 40, 15, 127, -128, 0)
orangethresholds = (25, 100, 18, 62, 45, 127)
Wall_THRESHOLD = [(0, 13, -128, 27, -11, 127)]
Black_ROI = [(2,  25, 18, 40 , 0.3 ), (140, 25, 18, 40, 0.5) ];#(5,  20, 40, 80 , 0.3 ), (280, 20, 315, 80, 0.5)
Black_Center_ROI = [(50, 0, 60, 60, 0.3 ) ];

# Defining which pin is used to control the servo motor used for steering
servo_1 = Servo(1)

led1 = pyb.LED(1)
led2 = pyb.LED(2)
led3 = pyb.LED(3)

#led1.on()
#led2.on()
#led3.on()

# Starting the camera sensor
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
#sensor.set_brightness(2)
clock = time.clock()
#sensor.skip_frames(time = 2000)
#sensor.set_auto_gain(False) # must be turned off for color tracking
#sensor.set_auto_whitebal(False) # must be turned off for color tracking
#sensor.set_vflip(True)
#sensor.set_hmirror(True)
#sensor.set_auto_exposure(False, 15000)
sensor.skip_frames(time = 500)
# sensor.set_brightness(3)
#exposure_time_scale = 0.5
#current_exposure_time_in_microsecond = sensor.get_exposure_us()
#sensor.set_auto_exposure(False, exposure_us = int(current_exposure_time_in_microsecond * exposure_time_scale))
#sensor.set_contrast(-3)

#led1.off()
#led2.off()
#led3.off()


# Debugging section:

# Debug All:
debug_all = True
# Debug Blocks:
debug_blocks = True
# Debug Walls:
debug_walls = False
# Debug Lines
debug_lines = True


# Needed variables
status = 0 # Status: 0 --> neutral, 1 --> right, -1 --> left
red_area = -1
green_area = -1
red_blob_pos = -1
green_blob_pos = -1

left_angle_strong = -70
left_angle_weak = -70
neutral_angle = -30
right_angle_strong = 20
right_angle_weak = 20

turn_flag = 0
wall_left = 0
wall_right = 0
wall_center = 0
area_right = 0
area_left = 0

corner_counter = 0
first_line = "X"

# Defining which pins are used to control the motor driver
tim = Timer(2, freq=1000) # Frequency in Hz
ch2 = tim.channel(3, Timer.PWM, pin=Pin("P4"))
ch3 = tim.channel(4, Timer.PWM, pin=Pin("P5"))

pin6 = Pin('P6', Pin.IN, Pin.PULL_DOWN)


# Functions:

# This function tracks the coloured objects and return the area of the blob and the x-coordinate of its center
# The function that start with draw are responsible for drawing the outlines of the blob in the frame buffer
def find_the_blob(thresholds) :
    blob_area = -1
    blob_cx = -1
    for blob in img.find_blobs([thresholds], pixels_threshold=100, area_threshold=150, merge=True):
        # These values depend on the blob not being circular - otherwise they will be shaky.
        if debug_all or debug_blocks:
            print("area = ", blob.area())
            print("cy = ", blob.cy())
            print("cx = ", blob.cx())
        if ((blob.cy() < 60)):
            #img.draw_edges(blob.min_corners(), color=0)
            #img.draw_line(blob.major_axis_line(), color=0)
            #img.draw_line(blob.minor_axis_line(), color=255)
            # These values are stable all the time.
            img.draw_rectangle(blob.rect(), color=(255,255,255))
            img.draw_cross(blob.cx(), blob.cy(), color=(255,255,255))
            # Note - the blob rotation is unique to 0-180 only.
            img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=10, color=(255,255,255))
            blob_area = blob.area()
            blob_cx = blob.cx()
    return blob_area, blob_cx

# This function determines what angle should the servo be on when the robot is moving backwards
# The angle is opposite to the last angle the servo has been adjusted to
def what_angle (state):
    if (state == 1):
        servo_1.angle(left_angle_strong)
    elif (state == -1):
        servo_1.angle(right_angle_strong)

# Function explaination
def lines(line_color):
    global img, corner_counter, status
    start_time = pyb.millis()
    corner_counter += 1
    while (pyb.elapsed_millis(start_time) <= 3000) :
        img = img.gamma_corr(gamma = 0.3, contrast = 20.0, brightness = 0.08)
        largest_blobC = -1
        wall_center = 0
        red_area, red_blob_pos = find_the_blob(redthresholds)
        green_area, green_blob_pos = find_the_blob(greenthresholds)
        if ((red_area > green_area) and (red_blob_pos >= 0)) :
            status = 1
            if (red_area > 2500 and red_blob_pos > 30):
                Motion(0, 85)
                what_angle(status)
                time.sleep(0.4)
                Motion(80, 0)
                what_angle(status * -1)
                time.sleep(0.1)
                img = sensor.snapshot()
            else :
                Motion(80, 0)
                #Motion(80, 0)
                servo_1.angle(right_angle_strong)
                turn_flag = 1
            break
        if ((red_area < green_area) and (green_blob_pos <= 160)) :
            status = -1
            if (green_area > 2500 and green_blob_pos < 130):
                Motion(0, 85)
                what_angle(status)
                time.sleep(0.4)
                Motion(80, 0)
                what_angle(status * -1)
                time.sleep(0.1)
                img = sensor.snapshot()
            else :
                Motion(80, 0)
                #Motion(80, 0)
                servo_1.angle(left_angle_strong)
                turn_flag = 1
            break
        if (line_color == "Blue") :
            servo_1.angle(left_angle_strong)
            Motion(90, 0)
            status = -1
            if debug_all or debug_lines:print("Blue")
        else :
            servo_1.angle(right_angle_strong)
            Motion(90, 0)
            status = 1
            if debug_all or debug_lines:print("Orange")
        for x in Black_Center_ROI:
            blobsC = img.find_blobs([Wall_THRESHOLD[0]],roi= x[0:4])
            if blobsC:
                #print("blobsC : ", blobsC)
                largest_blobC = max(blobsC, key=lambda bL: bL.area())
                #print("blobsC 0.5 : ", blobsC[0].area())

                #print("largest_blobC 0.5", largest_blobC.area())
                if debug_all or debug_lines:print("Center area: ", largest_blobC.area())
                if(largest_blobC.area() >= 3200):
                    wall_center = 1
                    if debug_all or debug_lines:print("backwards")
                elif (largest_blobC.area() <= 420):
                    wall_center = -1
        if debug_all or debug_lines:print("wall center = ", wall_center)
        if (wall_center == 1) :
            Motion(0, 80)
            what_angle(status)
            time.sleep(0.2)
            Motion(80, 0)
            what_angle(status* -1)
            time.sleep(0.2)
            img = sensor.snapshot()
            if debug_all or debug_lines:print("here 1")
        img = sensor.snapshot()

# Function explaination
def Motion(speed1, speed2) :
    # Motion(80, 0) --> Forward fast
    # Motion(70, 0) --> Forward slow
    # Motion(0, 70) --> Backward
    # Motion(0, 0) ---> Stop
    ch2.pulse_width_percent(speed1)
    ch3.pulse_width_percent(speed2)

#led3.on()

servo_1.angle(neutral_angle)
Motion(80, 0)

while(True):
    clock.tick()
    img = sensor.snapshot()
    #img = img.gaussian(3)
    img = img.gamma_corr(gamma = 0.3, contrast = 20.0, brightness = 0.09)
    #img.median(1, percentile=0.8)
    #led1.on()
    #led2.on()
    #led3.on()

    # Finding the areas and the x-coordinates of the red and green objects by calling the function
    # The parameter given to the function is the threshold for the required color
    red_area, red_blob_pos = find_the_blob(redthresholds)
    green_area, green_blob_pos = find_the_blob(greenthresholds)
    print("Red Area",red_area)
    largest_blobW = -1
    largest_blobC = -1
    #img.draw_rectangle((2,  10, 18, 40))
    #img.draw_rectangle((140, 10, 18, 40), color=(0,0,0))
    #img.draw_rectangle((50, 0, 60, 60))

    blue = img.find_blobs([bluethresholds] , pixels_threshold=100, area_threshold=300)
    orange = img.find_blobs([orangethresholds] , pixels_threshold = 100 , area_threshold = 300)

    if (blue and orange) :
        largest_blue_blob = max(blue, key = lambda bL : bL.area())
        largest_orange_blob = max(orange, key = lambda bL : bL.area())
        if debug_all or debug_lines:print("blue blob", largest_blue_blob.area())
        if ((largest_orange_blob.elongation() > 0.9) and (largest_orange_blob.cy() > largest_blue_blob.cy())
             and (first_line == "Orange" or first_line == "X")):
            if (first_line == "X") : first_line = "Orange"
            lines("Orange")
        elif ((largest_blue_blob.elongation() > 0.9) and (largest_orange_blob.cy() < largest_blue_blob.cy())
             and (first_line == "Blue" or first_line == "X")):
            if (first_line == "X") : first_line = "Blue"
            lines("Blue")
    elif (blue and (first_line == "Blue" or first_line == "X")):
        largest_blue_blob = max(blue, key = lambda bL : bL.area())
        if debug_all or debug_lines:print("blue blobs", largest_blue_blob.area())
        if (largest_blue_blob.elongation() > 0.9) :
            if (first_line == "X") : first_line = "Blue"
            lines("Blue")
    elif (orange and (first_line == "Orange" or first_line == "X")):
        largest_orange_blob = max(orange, key = lambda bL : bL.area())
        if (largest_orange_blob.elongation() > 0.9) :
            if (first_line == "X") : first_line = "Orange"
            lines("Orange")

    # The if condition's purpose is to know which colored object is closer and takes appropriate action
    # The larger the area the closer the object is

    elif ((red_area > green_area) and (red_blob_pos >= 0)):
        #led3.off()
        if debug_all or debug_blocks:print("Red is found")
        if debug_all or debug_blocks:print(red_area)
        status = 1
        if (red_area > 2600 and red_blob_pos >20):
            if debug_all or debug_blocks:print("BACK_RED")
            Motion(0, 90)
            what_angle(status)
            time.sleep(0.4)
            Motion(80, 0)
            what_angle(status* -1)
            time.sleep(0.1)
            img = sensor.snapshot()
        else:
            if debug_all or debug_blocks:print("Right_RED")
            Motion(80, 0)
            #Motion(80, 0)
            servo_1.angle(right_angle_strong)
            turn_flag = 1

    elif ((red_area < green_area) and (green_blob_pos <= 160)):
        #led3.off()
        if debug_all or debug_blocks:print("Green is found")
        if debug_all or debug_blocks:print(green_area)
        status = -1
        if (green_area > 2600 and green_blob_pos<125):
            if debug_all or debug_blocks:print("BACK_GREEN")
            Motion(0, 85)
            what_angle(status)
            time.sleep(0.4)
            Motion(80, 0)
            what_angle(status* -1)
            time.sleep(0.1)
            img = sensor.snapshot()
        else:
            if debug_all or debug_blocks:print("Left_GREEN")
            Motion(80, 0)
            #Motion(80, 0)
            servo_1.angle(left_angle_strong)
            turn_flag = 1

    else :
        wall_right = 0
        wall_left = 0
        wall_center = 0
        area_right = 0
        area_left = 0
        largest_blue_blob = 0
        largest_orange_blob = 0

        if (turn_flag == 1) :
            Motion(80, 0)
            servo_1.angle(neutral_angle)
            time.sleep(0.1)
            what_angle(status)
            time.sleep(0.3)
            img = sensor.snapshot()
            status = status * -1
            turn_flag = 0


        for x in Black_Center_ROI:
            blobsC = img.find_blobs([Wall_THRESHOLD[0]],roi= x[0:4])
            if blobsC:
                #print("blobsC : ", blobsC)
                largest_blobC = max(blobsC, key=lambda bL: bL.area())
                #print("blobsC 0.5 : ", blobsC[0].area())

                #print("largest_blobC 0.5", largest_blobC.area())
                if debug_all or debug_walls:print("Center area: ", largest_blobC.area())
                #img.draw_rectangle(largest_blobC.rect())
                if(largest_blobC.area() >= 2250):#3200
                    wall_center = 1
                    if debug_all or debug_walls:print("backwards")
                elif (largest_blobC.area() <= 900):
                    wall_center = -1

        for k in Black_ROI:
            blobsW = img.find_blobs([Wall_THRESHOLD[0]],roi= k[0:4])
            if blobsW:
                #print("blobsW : ", blobsW)

                # Check if a right wall is detected
                if(k[4] == 0.5):
                    # Find the blob with the most pixels.
                    largest_blobW = max(blobsW, key=lambda bL: bL.area())
                    #print("blobsW 0.5 : ", blobsW[0].area())
                    #print("largest_blobW 0.5", largest_blobW.area())
                    if debug_all or debug_walls:print("Right area: ", largest_blobW.area())
                    if(largest_blobW.area() >= 120):#700
                        img.draw_rectangle(largest_blobW.rect(), (255, 255, 255), 1)
                        #print("There is wall RIGHT")
                        wall_right = 1
                        area_right=largest_blobW.area()

                #print("WallRight",wall_right," K: ",k[4])

                # Check if a left wall is detected
                if(k[4] == 0.3):
                    #print("blobsW 0.3 : ", blobsW[0].area())
                    #print("largest_blobW 0.5", largest_blobW.area())
                    largest_blobW = max(blobsW, key=lambda bL: bL.area())
                    if debug_all or debug_walls:print("Left area: ", largest_blobW.area())
                    if(largest_blobW.area() >= 120):#700
                        img.draw_rectangle(largest_blobW.rect(), (255, 255, 255), 1)
                       # print("There is wall LEFT")
                        wall_left = 1
                        area_left=largest_blobW.area()
                #print("Area: ", blobsW[0].area())
                #print("Area: ", blobsW[1].area())
                #print("Var Wall Right: ", wall_right)
                #print("Var Wall Left: ", wall_left)

        #Condition if both right & left walls are detected
        if(wall_right is 1 and wall_left is 1):
            if debug_all or debug_walls:print("Area right = ", area_right)
            if debug_all or debug_walls:print("Area left = ", area_left)
            if(wall_center == 1):
                    Motion(0, 80)
                    what_angle(status)
                    time.sleep(0.2)
                    Motion(80, 0)
                    what_angle(status* -1)
                    time.sleep(0.2)
                    continue
            elif(area_left-area_right >= 100):
                if debug_all or debug_walls:print("right_w_F")
                status = 1
                #if(wall_center == 1):
                    #Motion(0, 80)
                    #what_angle(status)
                    #time.sleep(0.2)
                    #Motion(80, 0)
                    #what_angle(status* -1)
                    #time.sleep(0.2)
                    #continue
                servo_1.angle(right_angle_strong)
                # Motion(70, 0)
                Motion(80, 0)
            elif(area_left-area_right <= -100):
                if debug_all or debug_walls:print("left_w_F")
                status = -1
                #if(wall_center == 1):
                    #Motion(0, 80)
                    #what_angle(status)
                    #time.sleep(0.2)
                    #Motion(80, 0)
                    #what_angle(status * -1)
                    #time.sleep(0.2)
                    #continue
                servo_1.angle(left_angle_strong)
                # Motion(70, 0)
                Motion(80, 0)
            else:
                if debug_all or debug_walls:print("forward_w")
                if (status == -1) :
                    servo_1.angle(neutral_angle - 3)
                else :
                    servo_1.angle(neutral_angle)
                Motion(80, 0)

        #Condition if only the right wall is detected
        if(wall_right is 1 and wall_left is 0):
            status = -1
            if(wall_center == 1):
                Motion(0, 80)
                what_angle(status)
                time.sleep(0.2)
                Motion(80, 0)
                what_angle(status * -1)
                time.sleep(0.2)
                continue
            if(area_right>=600):
                if debug_all or debug_walls:print("left_w_strong")
                servo_1.angle(left_angle_strong)
                # Motion(70, 0)
                Motion(80, 0)
            else:
                if debug_all or debug_walls:print("left_w_weak")
                servo_1.angle(left_angle_weak)
                # Motion(70, 0)
                Motion(80, 0)

        #Condition if only the left wall is detected
        if(wall_right is 0 and wall_left is 1):
            status = 1
            if(wall_center == 1):
                Motion(0, 80)
                what_angle(status)
                time.sleep(0.2)
                Motion(80, 0)
                what_angle(status * -1)
                time.sleep(0.2)
                continue
            if(area_left>=600):
                if debug_all or debug_walls:print("right_w_strong")
                servo_1.angle(right_angle_strong)
                # Motion(70, 0)
                Motion(80, 0)
            else:
                if debug_all or debug_walls:print("right_w_weak")
                servo_1.angle(right_angle_weak)
                # Motion(70, 0)
                Motion(80, 0)

    #print(clock.fps())

