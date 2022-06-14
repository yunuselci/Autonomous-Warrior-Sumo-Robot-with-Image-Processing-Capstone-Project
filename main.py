import time
import cv2
import numpy as np
import RPi.GPIO as GPIO

global detected_area
global sure

sure = 0
detected_area = 0
generate_frame_width = 640
generate_frame_center_width = 320
generate_frame_height = 480
cap = cv2.VideoCapture(0)
cap.set(3, generate_frame_width)
cap.set(4, generate_frame_height)

##Input Defines Here
in1 = 16
in2 = 18
in3 = 38
in4 = 40
en = 11
en2 = 36
GPIO.setmode(GPIO.BOARD)
# First motor
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)
# Second motor
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.output(in3, GPIO.LOW)
GPIO.output(in4, GPIO.LOW)
# Speed control defined here
GPIO.setup(en, GPIO.OUT)
GPIO.setup(en2, GPIO.OUT)

p = GPIO.PWM(en, 1000)
p2 = GPIO.PWM(en2, 1000)
p.start(75)
p2.start(75)
global inumber
inumber = 0
global renk1
renk1 = 0
global renk2
renk2 = 0
global renk3
renk3 = 0
global state
state = "bos"


def GenerateFrames(scope, pictures):
    rows = len(pictures)
    cols = len(pictures[0])
    generateable_rows = isinstance(pictures[0], list)
    width = pictures[0][0].shape[1]
    height = pictures[0][0].shape[0]
    if generateable_rows:
        for x in range(0, rows):
            for y in range(0, cols):
                if pictures[x][y].shape[:2] == pictures[0][0].shape[:2]:
                    pictures[x][y] = cv2.resize(pictures[x][y], (0, 0), None, scope, scope)
                else:
                    pictures[x][y] = cv2.resize(pictures[x][y], (pictures[0][0].shape[1], pictures[0][0].shape[0]),
                                                None, scope, scope)
                    if len(pictures[x][y].shape) == 2: pictures[x][y] = cv2.cvtColor(pictures[x][y], cv2.COLOR_GRAY2BGR)
        b = np.zeros((height, width, 3), np.uint8)
        h = [b] * rows
        for x in range(0, rows):
            h[x] = np.hstack(pictures[x])
        ver = np.vstack(h)
    else:
        for x in range(0, rows):
            if pictures[x].shape[:2] == pictures[0].shape[:2]:
                pictures[x] = cv2.resize(pictures[x], (0, 0), None, scope, scope)
            else:
                pictures[x] = cv2.resize(pictures[x], (pictures[0].shape[1], pictures[0].shape[0]), None, scope, scope)
                if len(pictures[x].shape) == 2: pictures[x] = cv2.cvtColor(pictures[x], cv2.COLOR_GRAY2BGR)
        hor = np.hstack(pictures)
        ver = hor
    return ver


def move():
    global state
    global sure
    global detected_area

    if state == "ileri":
        print("ileri")
        sure = time.time() - 2
        p.ChangeDutyCycle(50)
        p2.ChangeDutyCycle(50)

        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
        GPIO.output(in3, GPIO.LOW)
        GPIO.output(in4, GPIO.HIGH)
        p.ChangeDutyCycle(37)
        p2.ChangeDutyCycle(37)
        state = "z"

    elif state == "sol":
        sure = time.time()
        print("sol")
        p.ChangeDutyCycle(45)
        p2.ChangeDutyCycle(45)

        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
        GPIO.output(in3, GPIO.LOW)
        GPIO.output(in4, GPIO.LOW)
        p.ChangeDutyCycle(37)
        p2.ChangeDutyCycle(37)
        state = "z"

    elif state == "sag":
        sure = time.time()
        print("sag")
        p.ChangeDutyCycle(42)
        p2.ChangeDutyCycle(42)
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        GPIO.output(in3, GPIO.LOW)
        GPIO.output(in4, GPIO.HIGH)
        p.ChangeDutyCycle(37)
        p2.ChangeDutyCycle(37)
        state = "z"
    elif state == "scan":

        if time.time() > (sure + 4):
            print("scan")
            p.ChangeDutyCycle(45)
            p2.ChangeDutyCycle(45)
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
            GPIO.output(in3, GPIO.LOW)
            GPIO.output(in4, GPIO.HIGH)
            p.ChangeDutyCycle(37)
            p2.ChangeDutyCycle(37)
        state = "z"
    if state == "stop":
        print("stop")
        p.ChangeDutyCycle(20)
        p2.ChangeDutyCycle(20)
        state = "z"


def FunctionToPass(a):
    pass


cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters", 640, 240)
cv2.createTrackbar("Threshold1", "Parameters", 23, 255, FunctionToPass)
cv2.createTrackbar("Threshold2", "Parameters", 20, 255, FunctionToPass)
cv2.createTrackbar("Area", "Parameters", 5000, 3000, FunctionToPass)


def GetFigures(image, image_figure, color):
    global state
    global detected_area
    figures, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in figures:
        area = cv2.contourArea(cnt)
        minimum_area = 1700

        if area > minimum_area:

            center_of_object = 0
            figure_perimeter = cv2.arcLength(cnt, True)
            approximate = cv2.approxPolyDP(cnt, 0.02 * figure_perimeter, True)

            x, y, w, h = cv2.boundingRect(approximate)
            if (len(approximate) == 3) & (color == red):

                cv2.drawContours(image_figure, cnt, -1, (0, 0, 0), 7)
                cv2.rectangle(image_figure, (x, y), (x + w, y + h), color, 5)
                center_of_object = (x + (w / 2))
                sens = area / 140
                if center_of_object > generate_frame_center_width + sens:  # soldasya
                    state = "sag"
                    # move()
                elif center_of_object < generate_frame_center_width - sens:  # sağdaysa
                    state = "sol"
                    # move()
                else:
                    state = "ileri"
                    # move()
                cv2.putText(image_figure, "triangle:" + str(len(approximate)), (x + w + 20, y + 20), cv2.FONT_HERSHEY_COMPLEX,
                            .7, color, 2)
                cv2.putText(image_figure, "Area:" + str(int(area)), (x + w + 20, y + 45), cv2.FONT_HERSHEY_COMPLEX, .7,
                            color, 2)

            if (len(approximate) == 4) & (color == green):

                cv2.drawContours(image_figure, cnt, -1, (0, 0, 0), 7)
                cv2.rectangle(image_figure, (x, y), (x + w, y + h), color, 5)
                center_of_object = (x + (w / 2))
                sens = area / 42
                if center_of_object > generate_frame_center_width + sens:  # soldasya
                    state = "sag"
                    # move()
                elif center_of_object < generate_frame_center_width - sens:  # sağdaysa
                    state = "sol"
                    # move()
                else:
                    state = "ileri"
                    # move()
                cv2.putText(image_figure, "rectangle" + str(len(approximate)), (x + w + 20, y + 20), cv2.FONT_HERSHEY_COMPLEX,
                            .7, color, 2)
                cv2.putText(image_figure, "Area:" + str(int(area)), (x + w + 20, y + 45), cv2.FONT_HERSHEY_COMPLEX, .7,
                            color, 2)

            if (len(approximate) == 3) & (color == blue):
                detected_area = area
                cv2.drawContours(image_figure, cnt, -1, (0, 0, 0), 7)
                cv2.rectangle(image_figure, (x, y), (x + w, y + h), color, 5)
                center_of_object = (x + (w / 2))
                sens = area / 250
                if center_of_object > generate_frame_center_width + sens:  # soldasya
                    state = "sag"
                    # move()
                elif center_of_object < generate_frame_center_width - sens:  # sağdaysa
                    state = "sol"
                    # move()
                else:
                    state = "ileri"
                    # move()
                cv2.putText(image_figure, "circle" + str(len(approximate)), (x + w + 20, y + 20), cv2.FONT_HERSHEY_COMPLEX, .7,
                            color, 2)
                cv2.putText(image_figure, "Area:" + str(int(area)), (x + w + 20, y + 45), cv2.FONT_HERSHEY_COMPLEX, .7,
                            color, 2)


while True:
    success, img = cap.read()
    image_figure = img.copy()
    blur = cv2.GaussianBlur(img, (7, 7), 1)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    first_threshold = cv2.getTrackbarPos("First Threshold", "Parameters")
    second_threshold = cv2.getTrackbarPos("Second Threshold", "Parameters")
    image_canny_algorithm = cv2.Canny(gray, first_threshold, second_threshold)
    kernel = np.ones((5, 5))
    image_dilate = cv2.dilate(image_canny_algorithm, kernel, iterations=1)
    red = (0, 0, 255)
    blue = (255, 0, 0)
    green = (0, 255, 0)
    black = (0, 0, 0)
    red_lower = np.array([170, 20, 80], np.uint8)
    red_upper = np.array([190, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsv, red_lower, red_upper)
    green_lower = np.array([25, 52, 72], np.uint8)
    green_upper = np.array([102, 255, 255], np.uint8)
    green_mask = cv2.inRange(hsv, green_lower, green_upper)
    blue_lower = np.array([94, 80, 2], np.uint8)
    blue_upper = np.array([120, 255, 255], np.uint8)
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
    red_mask = cv2.dilate(red_mask, kernel)
    res_red = cv2.bitwise_and(img, img, mask=red_mask)
    green_mask = cv2.dilate(green_mask, kernel)
    res_green = cv2.bitwise_and(img, img, mask=green_mask)
    blue_mask = cv2.dilate(blue_mask, kernel)
    res_blue = cv2.bitwise_and(img, img, mask=blue_mask)
    black_lower = np.array([0, 0, 0], np.uint8)
    black_upper = np.array([255, 255, 50], np.uint8)
    black_mask = cv2.inRange(hsv, black_lower, black_upper)
    black_mask = cv2.dilate(black_mask, kernel)
    res_black = cv2.bitwise_and(img, img, mask=black_mask)
    GetFigures(red_mask, image_figure, red)
    GetFigures(green_mask, image_figure, green)
    GetFigures(blue_mask, image_figure, blue)
    if (state != "ileri") & (state != "sol") & (state != "sag"):
        state = "scan"
        move()
    else:
        move()
    if cv2.waitKey(1) & 0xFF == ord("q"):
        GPIO.cleanup()
        break
