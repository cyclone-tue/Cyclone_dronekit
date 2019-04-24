from dronekit import connect, VehicleMode,Vehicle
import time
import math
from pymavlink import mavutil
import matplotlib.pyplot as plt
import threading

import sys, select

class HighLevelThread(threading.Thread):

    def __init__(self, group=None, target=None, name=None, verbose=None, drone=None):
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

    """
    connection_string = '127.0.0.1:14551'
    print("Connecting to vehicle on: %s" % (connection_string,))
    vehicle = connect(connection_string, wait_ready=True)

    vehicle.starttime = time.time()

    vehicle.i=0
    vehicle.total=0
    @vehicle.on_attribute('location.local_frame')
    def listener(self, attr_name, value):
        endtime=time.time()
        difference=endtime-self.starttime
        self.starttime=endtime
        #print difference
        self.i=self.i+1
        self.total=self.total+difference
        if self.i>=100:
            self.i=0
            totaltime = self.total / 100
            self.total = 0
            print("time: " + str(totaltime))
    """

    def passPath(self, path):
        self.pathPlanLock.acquire(True)
        self.path=self.dtToT(path)
        self.t0=time.time()
        self.newPath=True
        print('Path received, lenghth ')
        self.pathPlanLock.release()

    def dtToT(self, path):
        lpath=[]
        t=0

        for i in range(len(path)):
            print(path[i])
            lpath.append(path[i])
            dt=lpath[i][0]
            lpath[i][0]=t
            t=t+dt

        return lpath

    def getTarget(self):
        hasLow=False
        hasHigh=False
        for i in range(len(self.localPath)):
            if (time.time()-self.t0) < self.localPath[i][0]:
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
                del self.localPath[0:lowi]
            #interpolate between low and high
            return [(l+h)/2 for l,h in zip(*[low,high])]
        elif hasHigh:
            return high
        elif hasLow:
            if lowi > 0:
                del self.localPath[0:lowi]

            return low
        else:
            return 0

    def condition_yaw(self, heading, relative=False, duration = 0.0):
        """
        Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

        This method sets an absolute heading by default, but you can set the `relative` parameter
        to `True` to set yaw relative to the current yaw heading.

        By default the yaw of the vehicle will follow the direction of travel. After setting
        the yaw using this function there is no way to return to the default yaw "follow direction
        of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

        For more information see:
        http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
        """
        if relative:
            is_relative = 1 #yaw relative to direction of travel
        else:
            is_relative = 0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)


    def arm_and_takeoff(self, aTargetAltitude): #TODO remove
        """
        Arms vehicle and fly to aTargetAltitude.
        """

        print "Basic pre-arm checks"
        # Don't try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print " Waiting for vehicle to initialise..."
            time.sleep(1)

        print "Arming motors"
        # Copter should arm in GUIDED mode
        self.vehicle.mode    = VehicleMode("GUIDED")
        self.vehicle.armed   = True

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print " Waiting for arming..."
            time.sleep(1)

        print "Taking off!"
        self.vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print " Altitude: ", self.vehicle.location.global_relative_frame.alt
            #Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
                print "Reached target altitude"
                break
            time.sleep(1)

    def set_attitude(self, roll_angle = 0.0, pitch_angle = 0.0, heading = 0.0, thrust = 0.5): #TODO move to cyclone
        """
        Pitches in to make certain distance manoeuver
        """

        msg = self.vehicle.message_factory.set_attitude_target_encode(
        0, #milliseconds since boot
        0, 0, #System and component ID
        0b00000111, #Mappings: If any of these bits are set, the corresponding input should be ignored: (LSB is bit 1) bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle bit 8: attitude. Currently, throttle and attitude must be set to 0, i.e. not ignored
        self.to_quaternion_rad(roll_angle, pitch_angle, heading), #Attitude quaternion
        0, #Body roll rate in rad/s
        0, #Body pitch rate in rad/s
        0, #Body yaw rate in rad/s
        thrust #Normalized thrust
        )

        self.vehicle.send_mavlink(msg)



        # create the CONDITION_YAW command using command_long_encode()

    def set_message_rate(self, period=20000 , message_id=32):
        """
        Try to set the message rate for a particular mavlink message.
        ATTENTION: Period INTERVAL IS IN MICROSECONDS
        """

        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # command
            0,  # confirmation
            message_id,  # param 1, yaw in degrees
            period,  # param 2, yaw speed deg/s
            1,  # param 3, direction -1 ccw, 1 cw
            0,  # param 4, relative offset 1, absolute angle 0
            0, 0, 0)  # param 5 ~ 7 not used

        # send command to vehicle
        self.vehicle.send_mavlink(msg)

        frequency=1.0/float(period)*1000000.0
        print("Setting message " + str(message_id) + " to rate " + str(frequency) + " Hz")

    def to_quaternion_deg(self, roll = 0.0, pitch = 0.0, yaw = 0.0):
        """
        Convert degrees to quaternions
        """
        t0 = math.cos(math.radians(yaw * 0.5))
        t1 = math.sin(math.radians(yaw * 0.5))
        t2 = math.cos(math.radians(roll * 0.5))
        t3 = math.sin(math.radians(roll * 0.5))
        t4 = math.cos(math.radians(pitch * 0.5))
        t5 = math.sin(math.radians(pitch * 0.5))

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5

        return [w, x, y, z]

    def to_quaternion_rad(self, roll = 0.0, pitch = 0.0, yaw = 0.0):
        """
        Convert degrees to quaternions
        """
        t0 = math.cos((yaw * 0.5))
        t1 = math.sin((yaw * 0.5))
        t2 = math.cos((roll * 0.5))
        t3 = math.sin((roll * 0.5))
        t4 = math.cos((pitch * 0.5))
        t5 = math.sin((pitch * 0.5))

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5

        return [w, x, y, z]

    def run(self):
        #self.arm_and_takeoff(5)

        g=9.81

        p=3
        d=1.6

        #NOTE: z control not implemented yet, because of weird thrust 0.5 setting
        pz=1
        dz=1

        az=0
        taz=az-g


        #starttime=time.time()

        #state1=vehicle.location.local_frame.north
        #state2=vehicle.location.local_frame.north
        #endtime=time.time()
        counter=0

        self.set_message_rate() #TODO move to Cyclone object

        # while state1 == state2:
        #     break
        #     endtime=time.time()
        #     state2=vehicle.location.local_frame.north
        #
        #     if state1<>state2:
        #         counter=counter+1
        #         state1=state2
        #         if counter>=100:
        #             break
        #
        # totaltime=(endtime-starttime)/100
        # print("time: " + str(totaltime))


        # while time.time()-starttime < 5:
        #     R = 5
        #     T = 10
        #     omega = 2 * math.pi / T
        #     tax = R*omega/5
        #
        #     psi = vehicle.attitude.yaw
        #     vx = vehicle.velocity[0]
        #     vy = vehicle.velocity[1]
        #
        #     xa = tax
        #     ya = 0
        #
        #     theta0 = math.atan2(-(math.cos(psi) * xa + math.sin(psi) * ya), -taz)
        #     phi0 = math.atan2(math.cos(theta0) * (math.cos(psi) * ya - math.sin(psi) * xa), -taz)
        #     psi0 = math.atan2(vy, vx)
        #
        #     set_attitude(roll_angle=phi0, pitch_angle=theta0, heading=0, thrust=0.5)
        #     time.sleep(0.1)
        #
        #     counter = counter + 1
        #     if counter >= 10:
        #         counter = 0
        #         set_message_rate()

        #starttime=time.time()


        #fig=plt.figure()
        #plt.axis([0,100,-2,2])


        while not self.stop.isSet() and self.path: #TODO check path in the loop

            if self.newPath: #TODO make function
                self.pathPlanLock.acquire(True)
                self.localPath=self.path[:]
                self.newPath=False
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
                tx=t[1]
                ty=t[2]
                tz=t[3]
                tvx=t[4]
                tvy=t[5]
                tvz=t[6]
                tax=t[7]
                tay=t[8]
                #taz=t[9]-g
                #NOTE: z control not implemented yet, because of weird thrust 0.5 setting

                #R=2.5
                #T=15
                #omega=2*math.pi/T

                #tx=R*math.sin(omega*(time.time()-starttime))+1
                #ty=R*math.cos(omega*(time.time()-starttime))-R+1
                #tvx=R*omega*math.cos(omega*(time.time()-starttime))
                #tvy=-R*omega*math.sin(omega*(time.time()-starttime))
                #tax=-R*omega*omega*math.sin(omega*(time.time()-starttime))
                #tay=-R*omega*omega*math.cos(omega*(time.time()-starttime))


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

                #NOTE: z control not implemented yet, because of weird thrust 0.5 setting
                za=-g

                #TODO een keer proberen phi0 uit te reken a.d.h.v. theta i.p.v. thetha0; ook taz of za?
                theta0=math.atan2(-(math.cos(psi)*xa+math.sin(psi)*ya),-za)
                phi0=math.atan2(math.cos(theta0)*(math.cos(psi)*ya-math.sin(psi)*xa),-za)
                psi0=math.atan2(vy,vx)

                self.set_attitude(roll_angle=phi0, pitch_angle=theta0, heading=0, thrust=0.5)
                time.sleep(0.025)

                counter=counter+1
                if counter>=10:
                    counter=0
                    self.set_message_rate()

                if(abs(xa)==maxgsxy*g or abs(ya)==maxgsxy*g):
                    print("Clipping x/y")

                if(za==g+maxgsz*g or za==-g-maxgsz*g):
                    print("Clipping z")

                #print("x:",x," y:",y)
                #print("theta:",math.degrees(theta0)," phi:",math.degrees(phi0)," psi:",math.degrees(psi))

                #print ex

                #plt.scatter(time.time()-starttime,ex,time.time()-starttime,x)
                #plt.pause(0.01)


                #i, o, e = select.select([sys.stdin], [], [], 0.0001)
                #if i == [sys.stdin]: break


        #STOPPING

        #arm_and_takeoff(5)
        #print " Draaien "

        #set_attitude(heading=45)
        #time.sleep(3)


        #print("Move forward")
        #set_attitude(roll_angle = -5, heading=45, thrust = 0.5)
        #set_heading(heading=45)
        #time.sleep(5)
        #print("Move backward")
        #set_attitude(roll_angle = 10, heading=90, thrust = 0.5)
        #set_heading(heading=90)
        #time.sleep(5)
        #set_attitude(roll_angle = 0, thrust = 0.5)
        #time.sleep(5)


        #print("Setting LAND mode...")
        #vehicle.mode = VehicleMode("LAND")
        #time.sleep(1)

        # Close vehicle object before exiting script
        #print("Close vehicle object")
        #vehicle.close()