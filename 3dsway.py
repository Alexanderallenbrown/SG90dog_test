
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
freq = 1.1*pi
amp = 0.015
starttime = time.time()

def doSway(freq,amp,t):
    xfl = 0#amp*sin(freq*t)
    yfl = amp*sin(freq*t)
    zfl = 0
    xfr = 0#amp*sin(freq*t)
    yfr = -amp*sin(freq*t)
    zfr = 0
    xlr = 0#amp*sin(freq*t)
    ylr = -amp*sin(freq*t)
    zlr = 0
    xrr = 0#amp*sin(freq*t)
    yrr = amp*sin(freq*t)
    zrr = 0
    return xfl,yfl,zfl,xfr,yfr,zfr,xlr,ylr,zlr,xrr,yrr,zrr

def doBump(freq,amp,t):
    xfl = 0#amp*sin(freq*t)
    yfl = 0
    zfl = amp*sin(freq*t)
    xfr = 0#amp*sin(freq*t)
    yfr = 0
    zfr = amp*sin(freq*t)
    xlr = 0#amp*sin(freq*t)
    ylr = 0
    zlr = amp*sin(freq*t)
    xrr = 0#amp*sin(freq*t)
    yrr = 0
    zrr = amp*sin(freq*t)
    return xfl,yfl,zfl,xfr,yfr,zfr,xlr,ylr,zlr,xrr,yrr,zrr

def doStompL(freq,amp,t):
    xfl = 0#amp*sin(freq*t)
    yfl = 0
    zfl = amp*sin(2*freq*t)
    xfr = 0#amp*sin(freq*t)
    yfr = -amp
    zfr = 0
    xlr = 0#amp*sin(freq*t)
    ylr = -amp
    zlr = 0
    xrr = 0#amp*sin(freq*t)
    yrr = amp
    zrr = 0
    return xfl,yfl,zfl,xfr,yfr,zfr,xlr,ylr,zlr,xrr,yrr,zrr

while True:
    #setHips(90)
    t = time.time()-starttime
    # print t
    
    xfl,yfl,zfl,xfr,yfr,zfr,xlr,ylr,zlr,xrr,yrr,zrr = doStompL(freq,amp,t)

    flfem,fltib,flhip = flLeg.servoAngles(xfl,yfl,zfl)
    frfem,frtib,frhip = frLeg.servoAngles(xfr,yfr,zfr)
    lrfem,lrtib,lrhip = lrLeg.servoAngles(xlr,ylr,zlr)
    rrfem,rrtib,rrhip = rrLeg.servoAngles(xrr,yrr,zrr)

    # setLeg(frfem,frtib,0)
    # setLeg(flfem,fltib,2)
    # setLeg(lrfem,lrtib,4)
    # setLeg(rrfem,rrtib,6)

    setLeg3d(frfem,frtib,frhip,0,8)
    setLeg3d(flfem,fltib,flhip,2,9)
    setLeg3d(lrfem,lrtib,lrhip,4,10)
    setLeg3d(rrfem,rrtib,rrhip,6,11)

    time.sleep(.01)

