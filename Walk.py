from numpy import *
from matplotlib.pyplot import *

from Leg import Leg


class Walk:
    def __init__(self,stride_length=0.01,stride_height=0.01):
        self.stride_height = stride_height
        self.stride_length = stride_length
        self.P = array([0, 12.5,25,37.5,50,62.5,75,87.5,100])
        # front right leg
        self.FR_x = array([0,-self.stride_length/4,-self.stride_length/2,-self.stride_length/2,0,self.stride_length/2,self.stride_length/2,self.stride_length/4,0])
        self.FR_y = array([0,0,0,self.stride_height,self.stride_height,self.stride_height,0,0,0])
        # left rear leg
        self.LR_x = array([self.stride_length/2,self.stride_length/4,0,-self.stride_length/4,-self.stride_length/2,-self.stride_length/2,0,self.stride_length/2,self.stride_length/2])
        self.LR_y = array([0,0,0,0,0,self.stride_height,self.stride_height,self.stride_height,0])
        # left front leg
        self.FL_x = array([0,self.stride_length/2,self.stride_length/2,self.stride_length/4,0,-self.stride_length/4,-self.stride_length/2,-self.stride_length/2,0])
        self.FL_y = array([self.stride_height,self.stride_height,0,0,0,0,0,self.stride_height,self.stride_height])
        # right rear leg
        self.RR_x = array([-self.stride_length/2,-self.stride_length/2,0,self.stride_length/2,self.stride_length/2,self.stride_length/4,0,-self.stride_length/4,-self.stride_length/2])
        self.RR_y = array([0,self.stride_height,self.stride_height,self.stride_height,0,0,0,0,0])

    def getPos(self,p):
        pp = p%100
        frx = interp(pp,self.P,self.FR_x)
        fry = interp(pp,self.P,self.FR_y)
        flx = interp(pp,self.P,self.FL_x)
        fly = interp(pp,self.P,self.FL_y)
        lrx = interp(pp,self.P,self.LR_x)
        lry = interp(pp,self.P,self.LR_y)
        rrx = interp(pp,self.P,self.RR_x)
        rry = interp(pp,self.P,self.RR_y)
        return array([frx,fry]),array([flx,fly]),array([lrx,lry]),array([rrx,rry])



def main():
    frLeg = Leg(side=1)
    flLeg = Leg(side=2)
    lrLeg = Leg(side=2)
    rrLeg = Leg(side=1)

    walker = Walk()
    vp = arange(0,200,1)
    fl = zeros((len(vp),2))
    fr = zeros((len(vp),2))
    lr = zeros((len(vp),2))
    rr = zeros((len(vp),2))

    flServo = zeros((len(vp),2))
    frServo = zeros((len(vp),2))
    lrServo = zeros((len(vp),2))
    rrServo = zeros((len(vp),2))

    for k in range(0,len(vp)):
        fr[k,:],fl[k,:],lr[k,:],rr[k,:] = walker.getPos(vp[k])
        flServo[k,:] = flLeg.servoAngles(fl[k,0],fl[k,1])
        frServo[k,:] = frLeg.servoAngles(fr[k,0],fr[k,1])
        lrServo[k,:] = lrLeg.servoAngles(lr[k,0],lr[k,1])
        rrServo[k,:] = rrLeg.servoAngles(rr[k,0],rr[k,1])


    figure()
    subplot(2,1,1)
    plot(vp,fr[:,0],vp,fl[:,0],vp,lr[:,0],vp,rr[:,0])
    xlabel('Stride Percent')
    ylabel('Foot X (m)')
    legend(['front right','front left','rear left','rear right'])
    subplot(2,1,2)
    plot(vp,fr[:,1],vp,fl[:,1],vp,lr[:,1],vp,rr[:,1])
    xlabel('Stride Percent')
    ylabel('Foot Y (m)')
    

    figure()
    subplot(2,1,1)
    plot(vp,frServo[:,0],vp,flServo[:,0],vp,lrServo[:,0],vp,rrServo[:,0])
    xlabel('Stride Percent')
    ylabel('femur servo angle (deg)')
    legend(['front right','front left','rear left','rear right'])
    subplot(2,1,2)
    plot(vp,frServo[:,1],vp,flServo[:,1],vp,lrServo[:,1],vp,rrServo[:,1])
    xlabel('Stride Percent')
    ylabel('tibia servo angle (deg)')
    show()





if __name__ == '__main__':
    main()
