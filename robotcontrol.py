
import curses
stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)

stdscr.addstr(0,10,"Hit 'q' to quit")
stdscr.refresh()
stdscr.timeout(1)
key = ''

from smbus import SMBus
from PCA9685 import PWM
import time
from math import sin,tan
from Walk import Walk
from Leg import Leg
from numpy import *

import BNO055

ftau = 0.75;
fpitch = 0;
froll = 0;

told = time.time()-.1
tnow = time.time()
dt = tnow-told;

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


def doWalk(dir):
    global froll, fpitch,walker,p,ftau
    heading, roll, pitch = bno.read_euler()
    if not isnan(roll):
        roll = roll*3.14/180
        froll += dt/ftau*(roll-froll)
    if not isnan(pitch):
        pitch = pitch*3.14/180
        fpitch+=dt/ftau*(pitch-fpitch)
    #move right legts up and left down to make a positive angle
    #calculate pitch offsets
    #assume that half width of robot is 5cm
    poffset_left = tan(fpitch)*.045
    poffset_right = -poffset_left

    roffset_front= tan(-froll)*.08
    roffset_rear = -roffset_front
    #print pitch

    fr,fl,lr,rr = walker.getPos(p)
    flfem,fltib = flLeg.servoAngles(fl[0],-.01+fl[1]+poffset_left+roffset_front)
    frfem,frtib = frLeg.servoAngles(fr[0],-.01+fr[1]+poffset_right+roffset_front)
    lrfem,lrtib = lrLeg.servoAngles(lr[0],.01+lr[1]+poffset_left+roffset_rear)
    rrfem,rrtib = rrLeg.servoAngles(rr[0],.01+rr[1]+poffset_right+roffset_rear)
    #set each leg
    setLeg(frfem,frtib,0)
    setLeg(flfem,fltib,2)
    setLeg(lrfem,lrtib,4)
    setLeg(rrfem,rrtib,6)
    if(dir==1):
        p-=2
    elif(dir==-1):
        p+=2




##### MAIN PROGRAM #####


#now walk state stuff.
state = 1

fPWM = 50
i2c_address = 0x40 # (standard) adapt to your module
channel = 0 # adapt to your wiring
a = 8.5 # adapt to your servo
b = 2  # adapt to your servo

setup()

zeroBot()

standheight = 3.5
frLeg = Leg(side=1)
flLeg = Leg(side=2)
lrLeg = Leg(side=2)
rrLeg = Leg(side=1)
# frLeg = Leg(side=1)
# flLeg = Leg(side=2)
# lrLeg = Leg(side=2)
# rrLeg = Leg(side=1)


walker = Walk(stride_length=0.02,stride_height=-0.01)

p = 0

while key != ord('q'):
    setHips(90)
    key = stdscr.getch()
    stdscr.addch(20,25,key)
    stdscr.refresh()
    

    if key==ord('1'):
       state=1
       print("standing")
    if key==ord('2'):
       state = 2
       print("walking!")
    if key==ord('3'):
        state=3
        print("backwards!")

    # output of state machine
    if(state==1):
        zeroBot();
    if state==2:
        doWalk(1)
    if state==3:
        doWalk(-1)
        #print(p)
    #if state==2:
    #    zeroBot()
    time.sleep(.01)

curses.endwin()