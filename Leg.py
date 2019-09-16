from numpy import *
from matplotlib.pyplot import *


class Leg:
    def __init__(self,lf=.044,lt=.071,tht_offset = (76.8)*pi/180.0,thf_offset =(135)*pi/180.0,zerox=-.0254*1,zeroz=-.0254*3.58,side=1,servozero_f=90,servozero_t=90):
        self.servozero_f = servozero_f*pi/180#degrees to rads. servos meant to rest at mid position.
        self.servozero_t = servozero_t*pi/180
        self.lf,self.lt,self.tht_offset,self.thf_offset,self.zerox,self.zeroz,self.side=lf,lt,tht_offset,thf_offset,zerox,zeroz,side
        self.thf_raw = pi/4
        self.tht_raw = 76.8*pi/180.
        self.thf = self.thf_raw+self.thf_offset
        self.tht = self.tht_raw+self.thf_offset
    
    def servoAngles(self,xrel,zrel):
        self.rawAngles(xrel,zrel)
        #for right femur, 90 degrees represents thetaf of 135, zero represents thetaf of 180
        #therefore, effective offsets can be calculated as
        off_femur = -(self.servozero_f+self.thf_offset)
        off_tibia = self.servozero_t-self.tht_offset
        self.thf = -(self.thf_raw + off_femur)-pi
        self.tht = self.tht_raw + off_tibia
        if self.side==2:
            self.thf = pi-self.thf
            self.tht = self.tht
        elif self.side==1:
            self.tht = pi-self.tht
            

        
         
        return self.thf*180/pi,self.tht*180/pi
    def rawAngles(self,xrel,zrel):
        x = xrel+self.zerox
        z = zrel+self.zeroz
        d = sqrt(z**2+x**2)
        thleg = arctan2(z,x)
        opthlt = (d**2+self.lf**2-self.lt**2)/(2*d*self.lf)
        if(abs(opthlt)>=1):
            opthlt=1*opthlt/abs(opthlt)
            print "warning...thlt out of bounds for xrel: "+str(xrel)+" and zrel: "+str(zrel)
        thlt = arccos(opthlt)
        self.thf_raw = thleg+thlt
        opthd = (self.lt**2+self.lf**2-d**2)/(2*self.lt*self.lf)
        if(abs(opthd)>=1):
            opthd=1*opthd/abs(opthd)
            print "warning... thd out of bounds for xrel: "+str(xrel)+" and zrel: "+str(zrel)
        thd = arccos(opthd)
        self.tht_raw = pi-thd
        return self.thf_raw,self.tht_raw


def main():
    leg = Leg()

    dp = 1.0
    p = arange(0,100,1)

    L_s = .03
    H_s = .01

    P = array([0,25,50,75,100])
    dx = array([0,-L_s,-L_s,0,0])
    dy= array([0,0,H_s,H_s,0])

    dxp = interp(p,P,dx)
    dyp = interp(p,P,dy)

    thf_raw = zeros(len(p))
    tht_raw = zeros(len(p))
    thf_servo = zeros(len(p))
    tht_servo = zeros(len(p))

    for k in range(0,len(p)):
        thf_raw[k],tht_raw[k] = leg.rawAngles(dxp[k],dyp[k])
        thf_servo[k],tht_servo[k] = leg.servoAngles(dxp[k],dyp[k])

    figure()
    plot(dxp,dyp,'k.')
    plot(dx,dy,'ro')
    xlabel('X relative position (m)')
    ylabel('Y relative position (m)')
    axis('equal')

    figure()
    subplot(2,1,1)
    plot(p,thf_raw*180/pi,'k')
    xlabel('percent of stride')
    ylabel('femur angle raw (deg)')
    subplot(2,1,2)
    plot(p,tht_raw*180/pi,'k')
    xlabel('percent of stride')
    ylabel('tibia angle raw (deg)')

    figure()
    subplot(2,1,1)
    plot(p,thf_servo,'k')
    xlabel('percent of stride')
    ylabel('femur angle servo (deg)')
    subplot(2,1,2)
    plot(p,tht_servo,'k')
    xlabel('percent of stride')
    ylabel('tibia angle servo (deg)')

    show()

if __name__ == '__main__':
    main()

