import cv2
import numpy as np
import time

net = cv2.dnn.readNet('yolov4-tiny-custom_last.weights', 'yolov4-tiny-custom.cfg')


def object_dect():
    classes = []
    with open("coco.names", "r") as f:
        classes = f.read().splitlines()
    font = cv2.FONT_HERSHEY_PLAIN
    colors = np.random.uniform(0, 255, size=(100, 3))
    img =cv2.imread('bott.jpg')
    img=cv2.resize(img,(600,400))
    X1=0
    X2=300
    X3=600
    
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
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.2, 0.4)
    flag= 0
    if(len(boxes)>=2):
        x1, y1, w1, h1 = boxes[0]
        mid_p1=((x1+w1)+x1)/2
        for i in range(1,len(boxes)):
            x2, y2, w2, h2 = boxes[i]
            mid_p2=((x2+w2)+x2)/2
            if((X1<mid_p1<X2 or X1<mid_p2<X2) and (X2<mid_p1<X3 or X2<mid_p2<X3)):
                print("Aagara... or stop")
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
                
                lb=len(boxes)
                mid_p=((x+w)+x)/2
                
                #print(mid_p)
                print("object detected")
                if X1<mid_p<X2:
                    print("object in right")
                    
                elif X2<mid_p<X3:
                    print("object in left")
                    
                else:
                    print("no object")
        else:
            print("No object detected")
            #motor_on()
        cv2.imshow('Image', img)
cv2.destroyAllWindows()
object_dect()
    
