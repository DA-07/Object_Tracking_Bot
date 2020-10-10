import cv2
import numpy as np

def callback(x):
    pass

cap = cv2.VideoCapture(0)#0-laptop webcam , 1-additional webcam
cap.set(3,480)
cap.set(4,320)

_,frame=cap.read()

rows,cols,_=frame.shape
x_middle=int(cols/2)
y_middle=int(rows/2)

#display text
x_center=x_middle
y_center=y_middle
org=(50, 50)
font=cv2.FONT_HERSHEY_SIMPLEX
fontScale = 1
color =(0,255,255)
thickness=2

area = 450.0
cv2.namedWindow('HSV_TRACKBAR')

#redcolor
ilowH = 51
ihighH = 81

ilowS = 31
ihighS = 255
ilowV = 117
ihighV = 255

# create trackbars for color change
cv2.createTrackbar('lowH','HSV_TRACKBAR',ilowH,179,callback)
cv2.createTrackbar('highH','HSV_TRACKBAR',ihighH,179,callback)

cv2.createTrackbar('lowS','HSV_TRACKBAR',ilowS,255,callback)
cv2.createTrackbar('highS','HSV_TRACKBAR',ihighS,255,callback)

cv2.createTrackbar('lowV','HSV_TRACKBAR',ilowV,255,callback)
cv2.createTrackbar('highV','HSV_TRACKBAR',ihighV,255,callback)

while True:
    _, frame = cap.read()
    
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    hL = cv2.getTrackbarPos('lowH','HSV_TRACKBAR')
    hH = cv2.getTrackbarPos('highH','HSV_TRACKBAR')
    sL = cv2.getTrackbarPos('lowS','HSV_TRACKBAR')
    sH = cv2.getTrackbarPos('highS','HSV_TRACKBAR')
    vL = cv2.getTrackbarPos('lowV','HSV_TRACKBAR')
    vH = cv2.getTrackbarPos('highV','HSV_TRACKBAR')
    
    #red color
    #low_red = np.array([0, 140, 136])
    #high_red = np.array([9,255,255])
   
    #green Color
    #low_green = np.array([51, 31, 117])
    #high_green = np.array([81,255,255])
   
    lower_hsv = np.array([hL,sL,vL],np.uint8)
    higher_hsv = np.array([hH,sH,vH],np.uint8)
    
    mask = cv2.inRange(hsv_frame, lower_hsv, higher_hsv)
    
    kernel= np.ones((5,5),np.uint8)
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask=opening
    
    #For openCV version 2 or 4 
    if cv2.getVersionMajor() in [2, 4]:
        contours,_ = cv2.findContours(mask,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours= sorted(contours, key=lambda x:cv2.contourArea(x) , reverse=True)
    else:
    #For OpenCV 3  
         _,contours,_ = cv2.findContours(mask,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
         contours= sorted(contours, key=lambda x:cv2.contourArea(x) , reverse=True)
    
    for cnt in contours: 
        (x,y,w,h) = cv2.boundingRect(cnt)
        area = cv2.contourArea(cnt)
        print(area)
        if area>450: #20000 to stop
            x_middle = int((x+x+w)/2)
            y_middle = int((y+y+h)/2)       
            cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,0), 2)
            break
        else:
            x_middle = x_center
            y_middle = y_center
        
    cv2.line( frame, (x_middle,0), (x_middle,rows), (0,0,255),1)
    cv2.line( frame, (0,y_middle), (cols,y_middle), (0,0,255),1)
   
    if area<14000 and area>450 :
         if x_middle<x_center-25 :
             frame=cv2.putText( frame, "Move Left",org,font,fontScale,color,thickness,cv2.LINE_AA)
         elif x_middle>x_center+25 :
             frame=cv2.putText( frame, "Move Right",org,font,fontScale,color,thickness, cv2.LINE_AA)
         elif x_middle>=x_center-25 or x_middle<=x_center+25  :
             frame=cv2.putText( frame, "Move Forward",org,font,fontScale,color,thickness, cv2.LINE_AA)
    elif area>=14000 :
        frame=cv2.putText( frame, "STOP!!!",org,font,fontScale,(0,0,255),thickness, cv2.LINE_AA)
   
    cv2.imshow("Frame", frame)
    cv2.imshow("mask", mask)
    
    
    key = cv2.waitKey(1)
    if key == 27:
        break
cap.release()
cv2.destroyAllWindows()
