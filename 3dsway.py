
#import keyboard
from smbus import SMBus
from PCA9685 import PWM
import time
from math import sin
from Walk3d import Walk3d
from Leg3d import Leg3d
from numpy import *

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

def setLeg(femur,tibia,ch):
    fduty = a / 180 * femur + b
    tduty = a/180*tibia+b
    pwm.setDuty(ch,fduty)
    pwm.setDuty(ch+1,tduty)

def setLeg3d(femur,tibia,hip,ch,hipch):
    fduty = a/180 * femur + b
    tduty = a/180*tibia+b
    hduty = a/180*hip+b
    # print fduty,tduty,hduty
    pwm.setDuty(ch,fduty)
    pwm.setDuty(ch+1,tduty)
    pwm.setDuty(hipch,hduty)


setup()

zeroBot()

frLeg = Leg3d(side=1)
flLeg = Leg3d(side=2)
lrLeg = Leg3d(side=2)
rrLeg = Leg3d(side=1)

# walker = Walk3d(stride_height=-0.01,stride_length =.02)

p = 0
freq = 2*pi
amp = 0.01
starttime = time.time()

while True:
    #setHips(90)
    t = time.time()-starttime
    x = amp*sin(freq*t)
    y = 0
    z = 0
    flfem,fltib,flhip = flLeg.servoAngles(x,y,z)
    frfem,frtib,frhip = frLeg.servoAngles(x,y,z)
    lrfem,lrtib,lrhip = lrLeg.servoAngles(x,y,z)
    rrfem,rrtib,rrhip = rrLeg.servoAngles(x,y,z)

    # setLeg(frfem,frtib,0)
    # setLeg(flfem,fltib,2)
    # setLeg(lrfem,lrtib,4)
    # setLeg(rrfem,rrtib,6)

    setLeg3d(frfem,frtib,frhip,0,8)
    setLeg3d(flfem,fltib,flhip,0,9)
    setLeg3d(lrfem,lrtib,lrhip,0,10)
    setLeg3d(rrfem,rrtib,rrhip,0,11)

    time.sleep(.01)

