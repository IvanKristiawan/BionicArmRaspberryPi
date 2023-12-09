import cvzone
import cv2
from picamera2 import Picamera2
import mediapipe as mp
import math
import RPi.GPIO as GPIO
import time

#Library HandTrackingModule
class HandDetector:
    """
    Finds Hands using the mediapipe library. Exports the landmarks
    in pixel format. Adds extra functionalities like finding how
    many fingers are up or the distance between two fingers. Also
    provides bounding box info of the hand found.
    """

    def __init__(self, mode=False, maxHands=1, modelComplexity=1, detectionCon=0.5, trackCon=0.5):
        """
        :param mode: In static mode, detection is done on each image: slower
        :param maxHands: Maximum number of hands to detect
        :param detectionCon: Minimum Detection Confidence Threshold
        :param minTrackCon: Minimum Tracking Confidence Threshold
        """
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.modelComplex = 1
        self.trackCon = trackCon
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.modelComplex, self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils
        self.tipIds = [4, 8, 12, 16, 20]

    def findHands(self, img, draw=True):
        """
        Finds hands in a BGR image.
        :param img: Image to find the hands in.
        :param draw: Flag to draw the output on the image.
        :return: Image with or without drawings
        """
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms,
                                               self.mpHands.HAND_CONNECTIONS)
        return img

    def findPosition(self, img, handNo=0, draw=True):
        """
        Finds landmarks of a single hand and puts them in a list
        in pixel format. Also finds the bounding box around the hand.

        :param img: main image to find hand in
        :param handNo: hand id if more than one hand detected
        :param draw: Flag to draw the output on the image.
        :return: list of landmarks in pixel format; bounding box
        """

        xList = []
        yList = []
        bbox = []
        bboxInfo =[]
        self.lmList = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                h, w, c = img.shape
                px, py = int(lm.x * w), int(lm.y * h)
                xList.append(px)
                yList.append(py)
                self.lmList.append([px, py])
                if draw:
                    cv2.circle(img, (px, py), 5, (255, 0, 255), cv2.FILLED)
            xmin, xmax = min(xList), max(xList)
            ymin, ymax = min(yList), max(yList)
            boxW, boxH = xmax - xmin, ymax - ymin
            bbox = xmin, ymin, boxW, boxH
            cx, cy = bbox[0] + (bbox[2] // 2), \
                     bbox[1] + (bbox[3] // 2)
            bboxInfo = {"id": id, "bbox": bbox,"center": (cx, cy)}

            if draw:
                cv2.rectangle(img, (bbox[0] - 20, bbox[1] - 20),
                              (bbox[0] + bbox[2] + 20, bbox[1] + bbox[3] + 20),
                              (0, 255, 0), 2)

        return self.lmList, bboxInfo

    def fingersUp(self):
        """
        Finds how many fingers are open and returns in a list.
        Considers left and right hands separately
        :return: List of which fingers are up
        """
        if self.results.multi_hand_landmarks:
            myHandType = self.handType()
            fingers = []
            # Thumb
            if myHandType == "Right":
                if self.lmList[self.tipIds[0]][0] > self.lmList[self.tipIds[0] - 1][0]:
                    fingers.append(1)
                else:
                    fingers.append(0)
            else:
                if self.lmList[self.tipIds[0]][0] < self.lmList[self.tipIds[0] - 1][0]:
                    fingers.append(1)
                else:
                    fingers.append(0)

            # 4 Fingers
            for id in range(1, 5):
                if self.lmList[self.tipIds[id]][1] < self.lmList[self.tipIds[id] - 2][1]:
                    fingers.append(1)
                else:
                    fingers.append(0)
        return fingers

    def findDistance(self, p1, p2, img, draw=True):
        """
        Find the distance between two landmarks based on their
        index numbers.
        :param p1: Point1 - Index of Landmark 1.
        :param p2: Point2 - Index of Landmark 2.
        :param img: Image to draw on.
        :param draw: Flag to draw the output on the image.
        :return: Distance between the points
                 Image with output drawn
                 Line information
        """

        if self.results.multi_hand_landmarks:
            x1, y1 = self.lmList[p1][0], self.lmList[p1][1]
            x2, y2 = self.lmList[p2][0], self.lmList[p2][1]
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

            if draw:
                cv2.circle(img, (x1, y1), 15, (255, 0, 255), cv2.FILLED)
                cv2.circle(img, (x2, y2), 15, (255, 0, 255), cv2.FILLED)
                cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
                cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)

            length = math.hypot(x2 - x1, y2 - y1)
            return length, img, [x1, y1, x2, y2, cx, cy]

    def handType(self):
        """
        Checks if the hand is left or right
        :return: "Right" or "Left"
        """
        if self.results.multi_hand_landmarks:
            if self.lmList[17][0] < self.lmList[5][0]:
                return "Right"
            else:
                return "Left"

# Initialize Camera
piCam = Picamera2()
piCam.preview_configuration.main.size=(1280, 720)
piCam.preview_configuration.main.format="RGB888"
piCam.preview_configuration.align()
piCam.configure("preview")
piCam.start()

# Initialize Hand Detector
detector = HandDetector(maxHands=1, detectionCon=0.7)

# Initialize Servo
GPIO.setmode(GPIO.BOARD)

servoThumb = 11
servoIndex= 13
servoMiddle= 15
servoRing = 16
servoPinky = 18

GPIO.setup(servoThumb, GPIO.OUT)
GPIO.setup(servoIndex, GPIO.OUT)
GPIO.setup(servoMiddle, GPIO.OUT)
GPIO.setup(servoRing, GPIO.OUT)
GPIO.setup(servoPinky, GPIO.OUT)

# 0 degree = 2 | 180 degree = 12
pwmThumb = GPIO.PWM(servoThumb, 50)
pwmThumb.start(2)
pwmIndex = GPIO.PWM(servoIndex, 50)
pwmIndex.start(2)
pwmMiddle = GPIO.PWM(servoMiddle, 50)
pwmMiddle.start(2)
pwmRing = GPIO.PWM(servoRing, 50)
pwmRing.start(2)
pwmPinky = GPIO.PWM(servoPinky, 50)
pwmPinky.start(2)

lastValsThumb = 0
lastValsIndex = 0
lastValsMiddle = 0
lastValsRing = 0
lastValsPinky = 0

while True:
    img = piCam.capture_array()
    img = detector.findHands(img)
    lmList, bbox = detector.findPosition(img)
    if lmList:
        fingers = detector.fingersUp()
        print(fingers)

        if (fingers[0] == 0 and lastValsThumb != 1):
            print("No")
            for i in range(0, 180):
                DC=1./18.*(i)+2
                pwmThumb.ChangeDutyCycle(DC)
                time.sleep(.001)
            lastValsThumb = 1
        elif (fingers[0] == 1 and lastValsThumb != 0):
            print("Yes")
            for i in range(180,0,-1):
                DC=1./18.*(i)+2
                pwmThumb.ChangeDutyCycle(DC)
                time.sleep(.001)
            lastValsThumb = 0

        elif (fingers[1] == 0 and lastValsIndex != 1):
            print("No")
            for i in range(0, 180):
                DC=1./18.*(i)+2
                pwmIndex.ChangeDutyCycle(DC)
                time.sleep(.001)
            lastValsIndex = 1
        elif (fingers[1] == 1 and lastValsIndex != 0):
            print("Yes")
            for i in range(180,0,-1):
                DC=1./18.*(i)+2
                pwmIndex.ChangeDutyCycle(DC)
                time.sleep(.001)
            lastValsIndex = 0

        if (fingers[2] == 0 and lastValsMiddle != 1):
            print("No")
            for i in range(0, 180):
                DC=1./18.*(i)+2
                pwmMiddle.ChangeDutyCycle(DC)
                time.sleep(.001)
            lastValsMiddle = 1
        elif (fingers[2] == 1 and lastValsMiddle != 0):
            print("Yes")
            for i in range(180,0,-1):
                DC=1./18.*(i)+2
                pwmMiddle.ChangeDutyCycle(DC)
                time.sleep(.001)
            lastValsMiddle = 0

        if (fingers[3] == 0 and lastValsRing != 1):
            print("No")
            for i in range(0, 180):
                DC=1./18.*(i)+2
                pwmRing.ChangeDutyCycle(DC)
                time.sleep(.001)
            lastValsRing = 1
        elif (fingers[3] == 1 and lastValsRing != 0):
            print("Yes")
            for i in range(180,0,-1):
                DC=1./18.*(i)+2
                pwmRing.ChangeDutyCycle(DC)
                time.sleep(.001)
            lastValsRing = 0

        if (fingers[4] == 0 and lastValsPinky != 1):
            print("No")
            for i in range(0, 180):
                DC=1./18.*(i)+2
                pwmPinky.ChangeDutyCycle(DC)
                time.sleep(.001)
            lastValsPinky = 1
        elif (fingers[4] == 1 and lastValsPinky != 0):
            print("Yes")
            for i in range(180,0,-1):
                DC=1./18.*(i)+2
                pwmPinky.ChangeDutyCycle(DC)
                time.sleep(.001)
            lastValsPinky = 0

    cv2.imshow("Image", img)
    cv2.waitKey(1)

pwmThumb.stop()
GPIO.cleanup()