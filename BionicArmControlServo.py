import RPi.GPIO as GPIO
import time

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

# Take Input
#for i in range(0, 20):
    #desiredPosition = input("Where do you want the Servo? 0-180")
    #print(desiredPosition)
    #print(type(desiredPosition))
    #DC = 1.0/18.0*(float(desiredPosition))+2.0
    #pwmThumb.ChangeDutyCycle(DC)

# Hard Coded Input
for i in range(0, 180):
    DC=1./18.*(i)+2
    pwmThumb.ChangeDutyCycle(DC)
    pwmIndex.ChangeDutyCycle(DC)
    time.sleep(.05)

for i in range(180,0,-1):
    DC=1./18.*(i)+2
    pwmThumb.ChangeDutyCycle(DC)
    pwmIndex.ChangeDutyCycle(DC)
    time.sleep(.05)

pwmThumb.stop()
GPIO.cleanup()