import cv2
import math
import time
from threading import Thread
from time import sleep
import numpy as np
import os
import serial
import RPi.GPIO as GPIO


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(38,GPIO.OUT)
pwm=GPIO.PWM(38,100)
pwm.start(0)
pwm.ChangeDutyCycle(30)

GPIO.setup(5,GPIO.OUT)
servo1=GPIO.PWM(5,50)
servo1.start(0)


n=[i for i in range(0,200)]
ser = serial.Serial("/dev/ttyAMA0", 115200)
z=[i for i in range(100,150)]
temp=0
fk=0
count=10

def fun1():           #coverting live_video to frames
    print("fun1")
    vid = cv2.VideoCapture("https://192.168.206.203:8080/video")
    #vid = cv2.VideoCapture(0)
    fps = vid.get(cv2.CAP_PROP_FPS)
    fps = math.ceil(fps)
    fr=0
    fc = 1
    if (vid.isOpened()):
        while True:
            ret , frame = vid.read()
            if ret == True:
                frame = cv2.resize(frame,(640,360))
                cv2.imshow("video",frame)
                if (1*fr)%fps == 0:
                    
                    if (os.path.isdir("./frames")):
                        cv2.imwrite("frames/frame"+str(fc)+".jpg",frame)
                        fk=fun4()
                        print(fk)
                        if fk==1:#right_lane
                            t2 = Thread(target=fun2 ,args=(str(fc),))
                            t2.start()
                            fc+=1
                        else:
                            t3 = Thread(target=fun3 ,args=(str(fc),))
                            t3.start()
                            fc+=1 
                    else:
                        os.mkdir("frames")
                        cv2.imwrite("frames/frame"+str(fc)+".jpg",frame)
                        if fk==1:#right_lane
                            t2 = Thread(target=fun2 ,args=(str(fc),))
                            t2.start()
                            fc+=1
                        else:
                            t3 = Thread(target=fun3 ,args=(str(fc),))
                            t3.start()
                            fc+=1
                            
                fr+=1
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    else:
        print("***Camera Not Connected***")

    vid.release()
    cv2.destroyAllWindows()

def fun2(i):                  #using that frames for lane detection
    print("Right_lane_detected")
    img=cv2.imread("./frames/frame"+str(i)+".jpg")
    image=cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
    yl=np.array([61,100,100])
    yu=np.array([100,255,255])

    mask=cv2.inRange(image,yl,yu)
        
    yop=cv2.bitwise_and(image,image,mask=mask)
    image2=cv2.cvtColor(yop,cv2.COLOR_BGR2RGB)
    
    for x in range(350,600,1):
        px=image2[150,x]
        if (image2[150,x,0]!=0 and image2[150,x,1]!=0 and image2[150,x,2]!=0):
            #print("move")
            p1=x
            #print(x)
            r1=600-x
            print(i)
            if r1 in z:
                print("go ahead")
                #servo1.ChangeDutyCycle(9)
                break
            elif r1<136:
                print('left')
                servo1.ChangeDutyCycle(11)
                time.sleep(0.5)
                servo1.ChangeDutyCycle(9)
                break       
            else:
                print("right")
                servo1.ChangeDutyCycle(4)
                time.sleep(0.5)
                servo1.ChangeDutyCycle(9)
                break
                
        else:
        
            continue
    

        

def fun3(i):#left_lane_tracking
    print("left_lane_detected")
    img=cv2.imread("./frames/frame"+str(i)+".jpg")

    image=cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
    yl=np.array([61,100,100])
    yu=np.array([100,255,255])

    mask=cv2.inRange(image,yl,yu)
        
    yop=cv2.bitwise_and(image,image,mask=mask)
    image2=cv2.cvtColor(yop,cv2.COLOR_BGR2RGB)
    
    for x in range(350,0,-1):
        px=image2[150,x]
        if (image2[150,x,0]!=0 and image2[150,x,1]!=0 and image2[150,x,2]!=0):
            #print("move")
            p1=x
            #print(x)
            r1=350-x
            print(i)
            if r1 in z:
                print("go ahead")
                #servo1.ChangeDutyCycle(9)
                break
            elif r1<136:
                print('right')
                servo1.ChangeDutyCycle(4)
                time.sleep(0.5)
                servo1.ChangeDutyCycle(9)
                break       
            else:
                print("left")
                servo1.ChangeDutyCycle(11)
                time.sleep(0.5)
                servo1.ChangeDutyCycle(9)
                break
                
        else:
        
            continue

def fun4():              # lidar code for estimating the distance of the object
    global temp
    global fk
    count = ser.in_waiting
    time.sleep(0.0005)
    if count >8 :
        recv = ser.read(9)
        ser.reset_input_buffer()
        if recv[0] == 0x59 and recv[1] == 0x59 :
            distance=recv[2]+recv[3]*256
            print('distance',distance)
            if temp==1 and distance in n:
                print("right_lidar")
                servo1.ChangeDutyCycle(4)
                time.sleep(2)
                servo1.ChangeDutyCycle(9)
                
                temp=0
                fk=1
            elif temp==0 and distance in n:
                print("left_lidar")
                
                servo1.ChangeDutyCycle(11)
                time.sleep(2)
                servo1.ChangeDutyCycle(9)
                temp=1
                fk=0
            elif temp==1:
                fk=1
            else:
                fk=0
    return fk



    
t1 = Thread(target=fun1)
t1.start()
