from numpy import *
from matplotlib.pyplot import *

from Leg3d import Leg3d


class LegPattern:
    def __init__(self,stride_length=0.01,stride_height=0.01):
        self.stride_height = stride_height
        self.stride_length = stride_length
        self.P = array([0, 12.5,25,37.5,50,62.5,75,87.5,100])
        # front right leg
        self.leg_y = array([0,-self.stride_length/4,-self.stride_length/2,-self.stride_length/2,0,self.stride_length/2,self.stride_length/2,self.stride_length/4,0])
        self.leg_z = array([0,0,0,self.stride_height,self.stride_height,self.stride_height,0,0,0])
        self.leg_x = array([0,0,0,0,0,0,0,0,0])


    def getPos(self,p):
        pp = p%100
        legx = interp(pp,self.P,self.leg_x)
        legy = interp(pp,self.P,self.leg_y)
        legz = interp(pp,self.P,self.leg_z)
        return array([legx,legy,legz])



def main():
    frLeg = Leg3d(side=1)

    walker = LegPattern()
    vp = arange(0,200,1)
    fr = zeros((len(vp),3))

    frServo = zeros((len(vp),3))
    

    for k in range(0,len(vp)):
        fr[k,:] = walker.getPos(vp[k])
        frServo[k,:] = frLeg.servoAngles(fr[k,0],fr[k,1],fr[k,2])


    figure()
    subplot(3,1,1)
    plot(vp,fr[:,0])
    xlabel('Stride Percent')
    ylabel('Foot X (m)')
    legend(['front right'])
    subplot(3,1,2)
    plot(vp,fr[:,1])
    xlabel('Stride Percent')
    ylabel('Foot Y (m)')
    subplot(3,1,3)
    plot(vp,fr[:,2])
    xlabel('Stride Percent')
    ylabel('Foot Z (m)')
    

    figure()
    subplot(3,1,1)
    plot(vp,frServo[:,0])
    xlabel('Stride Percent')
    ylabel('femur servo angle (deg)')
    legend(['front right'])
    subplot(3,1,2)
    plot(vp,frServo[:,1])
    xlabel('Stride Percent')
    ylabel('tibia servo angle (deg)')
    subplot(3,1,3)
    plot(vp,frServo[:,2])
    xlabel('Stride Percent')
    ylabel('hip servo angle (deg)')
    show()





if __name__ == '__main__':
    main()
