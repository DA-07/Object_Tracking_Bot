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

#initialize GPIO pins, tell OS which pins will be used to control Model-Pi L298N board
GPIO.setmode(GPIO.BOARD)
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

#Initialize Rear model X board ENA and ENB pins, tell OS that ENA,ENB will output analog PWM signal with 1000 frequency
#rightSpeed = GPIO.PWM(ENA,1000)
#leftSpeed = GPIO.PWM(ENB,1000)
#rightSpeed.start(0)
#leftSpeed.start(0)

#make front right motor moving forward
def fr_ahead(speed):
    GPIO.output(INR1,True)
    GPIO.output(INR2,False)

    #ChangeDutyCycle(speed) function can change the motor rotation speed
    #rightSpeed.ChangeDutyCycle(speed)

#make rear right motor moving forward    
def rr_ahead(speed):  
    GPIO.output(INR3,True)
    GPIO.output(INR4,False)
    #leftSpeed.ChangeDutyCycle(speed)
   
#make front right motor moving backward
def fr_back(speed):
    GPIO.output(INR2,True)
    GPIO.output(INR1,False)

    #ChangeDutyCycle(speed) function can change the motor rotation speed
    #rightSpeed.ChangeDutyCycle(speed)

#make rear right motor moving backward    
def rr_back(speed):  
    GPIO.output(INR4,True)
    GPIO.output(INR3,False)
    #leftSpeed.ChangeDutyCycle(speed)
   
   
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
    #leftSpeed.ChangeDutyCycle(0)
    #rightSpeed.ChangeDutyCycle(0)

go_ahead(100)
time.sleep(1)
stop_car()

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

GPIO.cleanup()    

