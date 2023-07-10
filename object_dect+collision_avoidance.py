import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
net = cv2.dnn.readNet('yolov4-tiny-person_last.weights', 'yolov4-tiny-custom.cfg')


def object_dect():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(38,GPIO.OUT)
    pwm=GPIO.PWM(38,100)

    #pwm.start(0)
    #pwm.ChangeDutyCycle(30)

    GPIO.setup(5,GPIO.OUT)
    servo1=GPIO.PWM(5,50)
    servo1.start(0)
    classes = []
    with open("coco.names", "r") as f:
        classes = f.read().splitlines()

    cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
    font = cv2.FONT_HERSHEY_PLAIN
    colors = np.random.uniform(0, 255, size=(100, 3))

    while True:
        ret, img = cap.read()
        img=cv2.resize(img,(600,600))
        X1=0
        X2=300
        X3=600
        #l=cv2.rectangle(img,(0,0),(300,700),(100,255,100),2)
        #m=cv2.rectangle(img,(0,0),(300,600),(100,255,100),2)
        #r=cv2.rectangle(img,(300,0),(600,700),(100,255,100),2)
        height, width,cha = img.shape

        blob = cv2.dnn.blobFromImage(img, 1/255, (416, 416), (0,0,0), swapRB=True, crop=False)
        net.setInput(blob)
        output_layers_names = net.getUnconnectedOutLayersNames()
        layerOutputs = net.forward(output_layers_names)

        boxes = []
        confidences = []
        class_ids = []

        for output in layerOutputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.2:
                    center_x = int(detection[0]*width)
                    center_y = int(detection[1]*height)
                    w = int(detection[2]*width)
                    h = int(detection[3]*height)

                    x = int(center_x - w/2)
                    y = int(center_y - h/2)

                    boxes.append([x, y, w, h])
                    confidences.append((float(confidence)))
                    class_ids.append(class_id)
                    #cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
                    #cv2.imshow('img',img)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.2, 0.4)
        flag= 0
        if(len(boxes)>=2):            
            x1, y1, w1, h1 = boxes[0]
            mid_p1=((x1+w1)+x1)/2
            for i in range(1,len(boxes)):
                x2, y2, w2, h2 = boxes[i]
                mid_p2=((x2+w2)+x2)/2
                if((X1<mid_p1<X2 or X1<mid_p2<X2) and (X2<mid_p1<X3 or X2<mid_p2<X3)):
                    print("stop...")
                    pwm.stop(0)
                    #time.sleep(10)
                    flag = 1
                    break;
        if(flag!=1 or len(boxes)<2):

            if len(indexes)>0:
                for i in indexes.flatten():
                    x, y, w, h = boxes[i]
                    label = str(classes[class_ids[i]])
                    confidence = str(round(confidences[i],2))
                    color = colors[i]
                    rect=cv2.rectangle(img, (x,y), (x+w, y+h), color, 2)
                    cv2.putText(img, label + " " + confidence, (x, y+20), font, 2, (255,255,255), 2)
                    #print(x)
                    #print(x+w)
                    mid_p=((x+w)+x)/2
                    #print('mid_p',mid_p)
                    lb=len(boxes)
                    #print(lb)
                    #print("object detected")
                    if X1<mid_p<X2:
                        print("object in right")
                        servo1.ChangeDutyCycle(4)
                        time.sleep(0.5)
                        servo1.ChangeDutyCycle(9)
                        break
                        #time.sleep(5)
                    elif X2<mid_p<X3:
                        print("object in left")
                        servo1.ChangeDutyCycle(11)
                        time.sleep(0.5)
                        servo1.ChangeDutyCycle(9)
                        #time.sleep(5)
                    else:
                        print("no object")
                        pwm.start(0)
                        pwm.ChangeDutyCycle(30)
                            #motor_on()
                            #motor open
            else:
                print("No object detected")
                pwm.start(0)
                pwm.ChangeDutyCycle(30)
        cv2.rectangle(img, (x,y), (x+w, y+h), (0,255,0), 2)        
        cv2.imshow('Image', img)
        if cv2.waitKey(25) & 0xFF ==ord('v'):
            break
         

    cap.release()
    cv2.destroyAllWindows()

object_dect()
    

    

















