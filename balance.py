
#import keyboard
from smbus import SMBus
from PCA9685 import PWM
import time
from math import sin,tan
from Walk import Walk
from Leg import Leg
from numpy import *

import BNO055



bno = BNO055.BNO055(rst=None)
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

# Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))


#now walk state stuff.
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
    if not isnan(femur):
        fduty = a / 180 * femur + b
        pwm.setDuty(ch,fduty)
    else:
        print "femur nan!"
    if not isnan(tibia):
        tduty = a/180*tibia+b
        pwm.setDuty(ch+1,tduty)
    else:
        print "tibia nan!"


setup()

zeroBot()

standheight = 4.5
# frLeg = Leg(side=1,zeroz=-.0254*standheight)
# flLeg = Leg(side=2,zeroz=-.0254*standheight)
# lrLeg = Leg(side=2,zeroz=-.0254*(standheight-.2))
# rrLeg = Leg(side=1,zeroz=-.0254*standheight)
frLeg = Leg(side=1)
flLeg = Leg(side=2)
lrLeg = Leg(side=2)
rrLeg = Leg(side=1)


walker = Walk(stride_length=0.0,stride_height=0.00)

p = 0

while True:
    setHips(90)

    #if keyboard.is_pressed('1'):
    #    state=1
    #    print("standing")
    #if keyboard.is_pressed('2'):
    #    state = 2
    #    print("walking!")
    if state==2:
        heading, roll, pitch = bno.read_euler()
        if not isnan(roll):
            roll = roll*3.14/180
        if not isnan(pitch):
            pitch = pitch*3.14/180
        #move right legts up and left down to make a positive angle
        #calculate pitch offsets
        #assume that half width of robot is 5cm
        poffset_left = 0#tan(pitch)*.045
        poffset_right = -poffset_left
        print pitch

        fr,fl,lr,rr = walker.getPos(p)
        flfem,fltib = flLeg.servoAngles(fl[0],fl[1]+poffset_left)
        frfem,frtib = frLeg.servoAngles(fr[0],fr[1]+poffset_right)
        lrfem,lrtib = lrLeg.servoAngles(lr[0],lr[1]+poffset_left)
        rrfem,rrtib = rrLeg.servoAngles(rr[0],rr[1]+poffset_right)
        #set each leg
        setLeg(frfem,frtib,0)
        setLeg(flfem,fltib,2)
        setLeg(lrfem,lrtib,4)
        setLeg(rrfem,rrtib,6)
        p-=3
        #print(p)
    #if state==2:
    #    zeroBot()
    time.sleep(.05)

