# The Pythons' Openmv Code
# Latest_Algorithm - By: Ahmad Zaki

# These are the libraries needed for the code to function
import sensor, image, time, math, pyb
from pyb import Servo
from pyb import UART

# Thresholds to track red and green colored objects
greenthresholds = (59, 100, -128, -22, -128, 127)
redthresholds = (0, 100, 27, 127, -128, 127)

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
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()
# sensor.set_vflip(True)
# sensor.set_hmirror(True)

# Needed variables
status = 10
red_area = -1
green_area = -1
red_blob_pos = -1
green_blob_pos = -1

# This function tracks the coloured objects and return the area of the blob and the x-coordinate of its center
# The function that start with draw are responsible for drawing the outlines of the blob in the frame buffer
def find_the_blob(thresholds) :
    blob_area = -1
    blob_cx = -1
    for blob in img.find_blobs([thresholds], pixels_threshold=100, area_threshold=100, merge=True):
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


while(True):
    clock.tick()
    img = sensor.snapshot()
    #led1.on()
    #led2.on()
    #led3.on()


    # Sending the character 'F' to the Arduino Nano
    # The character 'F' represents the fast speed while the character 'S' reperesents the slow speed
    uart.write("F")

    # Setting the servo to the neurtal angle
    servo_1.angle(-10)
    print("Move Forward")

    # Finding the areas and the x-coordinates of the red and green objects by calling the function
    # The parameter given to the function is the threshold for the required color
    red_area, red_blob_pos = find_the_blob(redthresholds)
    green_area, green_blob_pos = find_the_blob(greenthresholds)

    # The if condition's purpose is to know which colored object is closer and takes appropriate action
    # The larger the area the closer the object is
    if (red_area > green_area):
        # Red is closer
        print("Red is found")
        # Since the target is to be to the right of the red traffic light, the robot should keep turning till
        # the traffic light is on the left
        while ((red_blob_pos >= 0 and red_blob_pos < 20) == 0):
            uart.write("S")
            img = sensor.snapshot()
            servo_1.angle(20)
            print("Turn right")
            print(red_blob_pos)
            red_area, red_blob_pos = find_the_blob(redthresholds)
            green_area, green_blob_pos = find_the_blob(greenthresholds)
            # This if condition handles the test case of a closer green object that may appear from a blindspot
            if (red_area < green_area):
                servo_1.angle(-35)
                break
            # This if condition handles the test case of incorrectly identifying a red object for a short time
            if (red_area == -1):
                servo_1.angle(-10)
                break

    elif (red_area < green_area):
       # Green is closer
       print("Green is found")
       # Since the target is to be to the left of the green traffic light, the robot should keep turning till
       # the traffic light is on the right
       while ((green_blob_pos > 300 and green_blob_pos <= 320) == 0):
           uart.write("S")
           img = sensor.snapshot()
           servo_1.angle(-35)
           print("Turn left")
           print(green_blob_pos)
           red_area, red_blob_pos = find_the_blob(redthresholds)
           green_area, green_blob_pos = find_the_blob(greenthresholds)
           # This if condition handles the test case of a closer red object that may appear from a blindspot
           if (red_area > green_area):
               servo_1.angle(20)
               break
           # This if condition handles the test case of incorrectly identifying a green object for a short time
           if (green_area == -1):
               servo_1.angle(-10)
               break

    else :
        print("Nothing is found")
