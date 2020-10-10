import cv2
import numpy as np
import RPi.GPIO as GPIO #control motor board through GPIO pins
import time #set delay time to control moving distance

GPIO.setwarnings(False)
#If INL1=True and INL2=False left motor move forward, If INL1=False,INL2=True left motor move backward,in other cases left motor stop(M1)
INL1 = 13 #GPIO27 to IN1 Front-left wheel direction
INL2 = 15 #GPIO22 to IN2 Front-left wheel direction

#If INL3=True and INL4=False left motor move forward, If INL3=False,INL4=True left motor move backward,in other cases left motor stop(M2)

INL3 = 16 #GPIO23 to IN3 Rear-left wheel direction
INL4 = 18 #GPIO24 to IN4 Rear-left wheel direction

#ENA/ENB are PWM(analog) signal pin which control the speed of left motors through GPIO ChangeDutyCycle(speed) function
ENAL = 11 #GPIO17 to ENA PWM SPEED of M1 motor
ENBL = 22 #GPIO25 to ENB PWM SPEED of M2 motor

#If INR1=True and INR2=False right motor move forward, If INL1=False,INL2=True right motor move backward,in other cases right motor stop(M3)

INR1 = 21 #GPIO9 to IN1 Front-right wheel direction
INR2 = 23 #GPIO11 to IN2 Front-right wheel direction


#If INR3=True and INR4=False right motor move forward, If INR1=False,INR2=True right motor move backward,in other cases right motor stop(M4)
INR3 = 24 #GPIO8 to IN3 Rear-right wheel direction
INR4 = 26 #GPIO7 to IN4 Rear-right wheel direction

#ENA/ENB are PWM(analog) signal pin which control the speed of right motors through GPIO ChangeDutyCycle(speed) function
ENAR = 19 #GPIO10 to ENA PWM SPEED of M3 motor
ENBR = 32 #GPIO12 to ENB PWM SPEED of M4 motor

#ultrasonic sensor
TRIG = 29
ECHO = 31

#initialize GPIO pins, tell OS which pins will be used to control Model-Pi L298N board
GPIO.setmode(GPIO.BOARD)
GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)
GPIO.setup(INR1, GPIO.OUT)
GPIO.setup(INR2, GPIO.OUT)
GPIO.setup(INR3, GPIO.OUT)
GPIO.setup(INR4, GPIO.OUT)
GPIO.setup(ENAL, GPIO.OUT)
GPIO.setup(ENBL, GPIO.OUT)
GPIO.setup(ENAR, GPIO.OUT)
GPIO.setup(ENBR, GPIO.OUT)
GPIO.setup(INL1, GPIO.OUT)
GPIO.setup(INL2, GPIO.OUT)
GPIO.setup(INL3, GPIO.OUT)
GPIO.setup(INL4, GPIO.OUT)
GPIO.output(ENAL,True)
GPIO.output(ENBL,True)
GPIO.output(ENAR,True)
GPIO.output(ENBR,True)


#make front right motor moving forward
def fr_ahead(speed):
    GPIO.output(INR1,True)
    GPIO.output(INR2,False)

#make rear right motor moving forward    
def rr_ahead(speed):  
    GPIO.output(INR3,True)
    GPIO.output(INR4,False)
   
#make front right motor moving backward
def fr_back(speed):
    GPIO.output(INR2,True)
    GPIO.output(INR1,False)

#make rear right motor moving backward    
def rr_back(speed):  
    GPIO.output(INR4,True)
    GPIO.output(INR3,False)   
   
#make front left motor moving forward
def fl_ahead(speed):
    GPIO.output(INL1,True)
    GPIO.output(INL2,False)

#make rear left motor moving forward    
def rl_ahead(speed):  
    GPIO.output(INL3,True)
    GPIO.output(INL4,False)
 
   
#make Front left motor moving backward
def fl_back(speed):
    GPIO.output(INL2,True)
    GPIO.output(INL1,False)

#make rear left motor moving backward    
def rl_back(speed):  
    GPIO.output(INL4,True)
    GPIO.output(INL3,False)

   
def go_ahead(speed):
    rr_ahead(speed)
    fl_ahead(speed)
    fr_ahead(speed)
    rl_ahead(speed)
   
def go_back(speed):
    rr_back(speed)
    rl_back(speed)
    fr_back(speed)
    fl_back(speed)

#making right turn  
def turn_right(speed):
    rr_back(speed)
    fr_back(speed)
    fl_ahead(speed)
    rl_ahead(speed)
     
#make left turn
def turn_left(speed):
    fl_back(speed)
    rl_back(speed)
    fr_ahead(speed)
    rr_ahead(speed)

# parallel left shift
def shift_left(speed):
    fl_back(speed)
    rr_back(speed)
    rl_ahead(speed)
    fr_ahead(speed)

# parallel right shift
def shift_right(speed):
    fr_back(speed)
    fl_ahead(speed)
    rl_back(speed)
    rr_ahead(speed)

def upper_right(speed):
    fl_ahead(speed)
    rr_ahead(speed)

def lower_left(speed):
    rr_back(speed)
    fl_back(speed)
   
def upper_left(speed):
    fr_ahead(speed)
    rl_ahead(speed)

def lower_right(speed):
    fr_back(speed)
    rl_back(speed)

#make both motor stop
def stop_car():
    GPIO.output(INR1,False)
    GPIO.output(INR2,False)
    GPIO.output(INR3,False)
    GPIO.output(INR4,False)
    GPIO.output(INL1,False)
    GPIO.output(INL2,False)
    GPIO.output(INL3,False)
    GPIO.output(INL4,False)

def callback(x):
    pass

def ultra():
        GPIO.output(TRIG, False)
        #print ("Waiting For Sensor To Settle")
        time.sleep(0.05)
        
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)
        
        while GPIO.input(ECHO)==0:
            pulse_start = time.time()
        
        while GPIO.input(ECHO)==1:
            pulse_end = time.time()
        
        pulse_duration = pulse_end - pulse_start
        
        distance = pulse_duration * 17150
        
        distance = round(distance, 2)
        
        # print ("Distance: ",distance,"cm")
        return distance


# go_ahead(100)
# time.sleep(1)
# stop_car()
# 
# go_back(100)
# time.sleep(1)
# stop_car()
# 
# turn_left(100)
# time.sleep(1)
# stop_car()
# 
# turn_right(100)
# time.sleep(1)
# stop_car()
# 
# shift_right(100)
# time.sleep(1)
# stop_car()
# 
# shift_left(100)
# time.sleep(1)
# stop_car()
# 
# upper_left(100)
# time.sleep(1)
# stop_car()
# 
# lower_right(100)
# time.sleep(1)
# stop_car()
# 
# upper_right(100)
# time.sleep(1)
# stop_car()
# 
# lower_left(100)
# time.sleep(1)
# stop_car()
# 
# GPIO.cleanup()    
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
    
    ut=ultra()
    print("distance",ut)
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
            stop_car()
        
    cv2.line( frame, (x_middle,0), (x_middle,rows), (0,0,255),1)
    cv2.line( frame, (0,y_middle), (cols,y_middle), (0,0,255),1)
   
    if (ut > 20 and ut <300) and area<38000 and area>450:
        if x_middle < x_center-45:
            frame=cv2.putText( frame, "Move Left",org,font,fontScale,color,thickness,cv2.LINE_AA)
            time.sleep(0.09)
            shift_left(100)
            time.sleep(0.02)
            stop_car()
        elif x_middle>x_center+45 :
            frame=cv2.putText( frame, "Move Right",org,font,fontScale,color,thickness, cv2.LINE_AA)
            time.sleep(0.09)
            shift_right(100)
            time.sleep(0.02)
            stop_car()
        elif x_middle>=x_center-45 or x_middle<=x_center+45  :
            frame=cv2.putText( frame, "Move Forward",org,font,fontScale,color,thickness, cv2.LINE_AA)
            time.sleep(0.09)
            go_ahead(100)
            time.sleep(0.08)
            stop_car()
    elif ut<=20 or area>=38000:
        frame=cv2.putText( frame, "STOP!!!",org,font,fontScale,(0,0,255),thickness, cv2.LINE_AA)    
        stop_car()

    cv2.imshow("Frame", frame)
    cv2.imshow("mask", mask)
    
    
    key = cv2.waitKey(1)
    if key == 27:
        break
cap.release()
cv2.destroyAllWindows()
GPIO.cleanup()
