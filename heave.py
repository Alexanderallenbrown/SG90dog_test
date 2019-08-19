# Servo2.py
# Two servo motors driven by PCA9685 chip

from smbus import SMBus
from PCA9685 import PWM
import time
from math import sin

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


def setHips(angle):
    duty = a / 180 * angle + b
    for ch in range(8,12):
        pwm.setDuty(ch,duty)

def setLegs(angle):
    duty = a / 180 * angle + b
    duty2 = a/180*(-angle+180)+b
    print (-angle+90)
    for ch in range(0,2):
        pwm.setDuty(ch,duty)
    for ch in range(2,6):
        pwm.setDuty(ch,duty2)
    for ch in range(6,8):
        pwm.setDuty(ch,duty)

def setDirection(direction):
    duty = a / 180 * direction + b
    pwm.setDuty(channel, duty)
    print "direction =", direction, "-> duty =", duty
    time.sleep(1) # allow to settle
   
print "starting"
setup()

tstart = time.time()

while True:
    t = time.time()-tstart
    alegs = 90+15*sin(t*2*3.14)
    setLegs(alegs)
    setHips(90)
    time.sleep(0.01)


