from smbus import SMBus
from PCA9685 import PWM
import time
from math import sin
from Walk import Walk
from Leg import Leg
from numpy import *

fPWM = 50
i2c_address = 0x40 # (standard) adapt to your module
channel = 0 # adapt to your wiring
a = 8.5 # adapt to your servo
b = 2  # adapt to your servo

def setup():
    global pwm
    bus = SMBus(1) # Raspberry Pi revision 2
    pwm = PWM(bus, i2c_address)
    pwm.setFreq(fPWM)

def zeroBot():
    duty = a/180*90+b
    for ch in range(0,16):
        pwm.setDuty(ch,duty)
    time.sleep(1)

#zeroBot()

def setHips(angle):
    duty = a / 180 * angle + b
    for ch in range(8,12):
        pwm.setDuty(ch,duty)

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

walker = Walk()

p = 0

while True:
    setHips(90)
    fr,fl,lr,rr = walker.getPos(p)
    flfem,fltib = flLeg.servoAngles(fl[0],fl[1])
    frfem,frtib = frLeg.servoAngles(fr[0],fr[1])
    lrfem,lrtib = lrLeg.servoAngles(lr[0],lr[1])
    rrfem,rrtib = rrLeg.servoAngles(rr[0],rr[1])
    #set each leg
    setLeg(frfem,frtib,0)
    setLeg(flfem,fltib,2)
    setLeg(lrfem,lrtib,4)
    setLeg(rrfem,rrtib,6)
    p+=1
    print(p)
    time.sleep(.001)

