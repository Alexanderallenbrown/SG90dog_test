
#import keyboard
from smbus import SMBus
from PCA9685 import PWM
import time
from math import sin
from Walk import Walk
from Leg import Leg
from Leg3d import Leg3d
from numpy import *
from legpattern import LegPattern

state = 2

fPWM = 50
i2c_address = 0x40 # (standard) adapt to your module
channel = 0 # adapt to your wiring
a = 8.5 # adapt to your servo
b = 2  # adapt to your servo

def setup():
    global pwm
    bus = SMBus(3) # Raspberry Pi revision 2
    pwm = PWM(bus, i2c_address)
    pwm.setFreq(fPWM)

def zeroBot():
    duty = a/180*90+b
    for ch in range(0,16):
        pwm.setDuty(ch,duty)
    time.sleep(.001)

#zeroBot()

def setHips(angle):
    duty = a / 180 * angle + b
    for ch in range(8,12):
        pwm.setDuty(ch,duty)

def setLeg3d(femur,tibia,hip,ch,hipch):
    fduty = a/180 * femur + b
    tduty = a/180*tibia+b
    hduty = a/180*hip+b
    # print fduty,tduty,hduty
    pwm.setDuty(ch,fduty)
    pwm.setDuty(ch+1,tduty)
    pwm.setDuty(hipch,hduty)

def setLeg(femur,tibia,ch):
    fduty = a / 180 * femur + b
    tduty = a/180*tibia+b
    pwm.setDuty(ch,fduty)
    pwm.setDuty(ch+1,tduty)


setup()

zeroBot()

frLeg = Leg(side=1)
flLeg = Leg(side=2)
lrLeg = Leg(side=2)
rrLeg = Leg(side=1)

walker = LegPattern()

p = 0

while True:

    frLeg = Leg3d(side=1)   

    fr = walker.getPos(p)
    frfem,frtib,frhip = frLeg.servoAngles(fr[0],fr[1],fr[2])
    # print frfem,frtib,frhip
    setLeg3d(frfem,frtib,frhip,0,8)

    p-=1
        #print(p)
    #if state==2:
    #    zeroBot()
    time.sleep(.01)

