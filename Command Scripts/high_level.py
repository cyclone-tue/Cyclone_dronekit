from dronekit import connect, VehicleMode,Vehicle
import time
import math
from pymavlink import mavutil
import matplotlib.pyplot as plt
import threading

import sys, select

class HighLevelThread(threading.Thread):

    def __init__(self, group=None, target=None, name=None, verbose=None, drone=None, logging=None):
        threading.Thread.__init__(self, group=group, target=target, name=name, verbose=verbose)
        self.drone = drone
        self.vehicle = drone.vehicle
        self.pathPlanLock = threading.Lock()
        self.path=[]
        self.localPath=[]
        self.t0=time.time()
        self.localPath=[]
        self.stop = threading.Event()
        self.newPath=True
        self.logging = logging

    def passPath(self, path):
        self.pathPlanLock.acquire(True)
        self.path=self.dtToT(path)

        self.newPath=True
        self.logging.debug('Path received')
        self.pathPlanLock.release()

    def dtToT(self, path):
        lpath=[]
        t=0

        for i in range(len(path)):
            lpath.append(path[i])
            dt=lpath[i][0]
            lpath[i][0]=t
            t=t+dt

        return lpath

    def getTarget(self):
        hasLow=False
        hasHigh=False

        t=time.time()-self.t0

        for i in range(len(self.localPath)):
            if t > self.localPath[i][0]:
                low= self.localPath[i]
                lowi=i
                hasLow=True
            else:
                high=self.localPath[i]
                highi=i
                hasHigh=True
                break

        if hasHigh and hasLow:
            if lowi > 0:
                #delete the past
                del self.localPath[0:lowi]
            #interpolate between low and high

            return self.interpolate(low,high,t)
        elif hasHigh:
            return high
        elif hasLow:
            if lowi > 0:
                #delete the past
                del self.localPath[0:lowi]

            #hover still in the last point
            #zero velocity
            low[4]=0
            low[5]=0
            low[6]=0
            #zero acceleration
            low[7]=0
            low[8]=0
            low[9]=0

            return low
        else:
            self.logging.error("No target!")
            return 0

    def interpolate(self, low, high, t):
        dt=high[0]-low[0]
        dtlow=t-low[0]

        fracHigh=dtlow/dt

        self.logging.debug("fracHigh = ")
        self.logging.debug(fracHigh)

        return [(l+(h-l)*fracHigh) for l,h in zip(*[low,high])]

    def run(self):

        g=9.81

        p=3
        d=1.6

        #NOTE: z control not implemented yet, because of weird thrust 0.5 setting
        pz=1
        dz=1

        #NOTE: z control not implemented yet, because of weird thrust 0.5 setting, so remove this:
        az=0
        taz=az-g

        counter=0 #counter for message rate setter
        self.drone.set_message_rate() #TODO move to Cyclone object


        while not self.stop.isSet():

            if self.newPath:
                self.pathPlanLock.acquire(True)
                self.localPath=self.path[:]
                self.newPath=False
                self.t0=time.time()
                self.pathPlanLock.release()

            if len(self.localPath) > 0:
                psi=self.vehicle.attitude.yaw

                x=self.vehicle.location.local_frame.north
                y=self.vehicle.location.local_frame.east
                z=self.vehicle.location.local_frame.down
                vx=self.vehicle.velocity[0]
                vy=self.vehicle.velocity[1]
                vz=self.vehicle.velocity[2]

                t=self.getTarget()
                self.logging.debug(t)
                tx=t[1]
                ty=t[2]
                tz=t[3]
                tvx=t[4]
                tvy=t[5]
                tvz=t[6]
                tax=t[7]
                tay=t[8]
                #taz=t[9]-g
                #NOTE: z control not implemented yet, because of weird thrust 0.5 setting, so uncomment that

                ex=tx-x
                ey=ty-y
                ez=tz-z
                evx=tvx-vx
                evy=tvy-vy
                evz=tvz-vz

                xfb=p*ex+d*evx
                yfb=p*ey+d*evy
                zfb=p*ez+d*evz

                xa=tax+xfb
                ya=tay+yfb
                za=taz+zfb

                #clipping
                maxgsxy=2
                maxgsz=1
                xa = min(maxgsxy*g, xa)
                xa = max(xa, -maxgsxy*g)
                ya = min(maxgsxy*g, ya)
                ya = max(ya, -maxgsxy*g)
                za = min(g+maxgsz*g, za)
                za = max(za, -g-maxgsz*g)

                #NOTE: z control not implemented yet, because of weird thrust 0.5 setting, so remove this:
                za=-g

                #TODO een keer proberen phi0 uit te reken a.d.h.v. theta i.p.v. thetha0; ook taz of za?
                theta0=math.atan2(-(math.cos(psi)*xa+math.sin(psi)*ya),-za)
                phi0=math.atan2(math.cos(theta0)*(math.cos(psi)*ya-math.sin(psi)*xa),-za)
                psi0=math.atan2(vy,vx)

                self.drone.set_attitude(roll_angle=phi0, pitch_angle=theta0, heading=0, thrust=0.5)

                #TODO check this rate
                time.sleep(0.025)

                counter=counter+1
                if counter>=10:
                    counter=0
                    self.drone.set_message_rate()

                if(abs(xa)==maxgsxy*g or abs(ya)==maxgsxy*g):
                    self.logging.warning("Clipping x/y")

                if(za==g+maxgsz*g or za==-g-maxgsz*g):
                    self.logging.warning("Clipping z")
        #end of while loop
              
        #STOPPING
        #self.drone.set_attitude(roll_angle=0, pitch_angle=0, heading=0, thrust=0.5)
        local_position = self.drone.vehicle.location.local_frame
        self.drone.goto_local_NED(local_position.north, local_position.east, local_position.down, mavutil.mavlink.MAV_FRAME_LOCAL_NED) # Stay where you are.

        self.logging.debug("Highlevel controller: I'm out, chillazz")
        
    #end of run