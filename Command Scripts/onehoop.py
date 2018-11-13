from dronekit import LocationGlobalRelative

import configs
from pymavlink import mavutil
from cyclone import Cyclone
import ctypes
import argparse
import logging
from visionThread import VisionThread


class flight():
    def __init__(self):
        parser = argparse.ArgumentParser(
            description='Print out vehicle state information. Connects to SITL on local PC by default.')
        parser.add_argument(
            '--connect', help="vehicle connection target string. If not specified, SITL automatically started and used.")
        parser.add_argument(
            '--simulate', help="Include if you want to connect to a simulation in stead of running on an actual drone. Does not wait for guided mode change."
        )

        args = parser.parse_args()
        self.simulation = False
        if args.simulate:
            self.simulation = True
        else:
            self.simulation = False
        logging.basicConfig()
        self.logger = logging.Logger("onehoop")
        formatter = logging.Formatter("%(asctime)s [%(threadName)-12.12s] [%(name)s-%(levelname)-5.5s] %(message)s")
        fileHandler = logging.FileHandler("onehoop.log", mode='w')
        streamHandler = logging.StreamHandler()
        fileHandler.setFormatter(formatter)
        streamHandler.setFormatter(formatter)
        self.logger.addHandler(fileHandler)
        self.logger.addHandler(streamHandler)
        handlers = [fileHandler, streamHandler]
        self.drone = self.getDrone(args, handlers)
        self.initVision()



    def fly(self):
        if self.simulation:
            self.drone.arm_and_takeoff(5)
        else:
            self.drone.awake_script()
        self.drone.obtain_home_location()
        self.drone.set_airspeed(5)

        while True:
            self.goToHoop()
            if not self.simulation:
                self.drone.awake_script()

        

    def end(self):
        self.vision.stop.set()
        self.logger.debug("Joining vision")
        self.vision.join(20.0)
        self.logger.info("Stopped vision")
        self.drone.vehicle.close()
        # del self.drone.vehicle
        if self.simulation:
            self.sitl.stop()
            del self.sitl
        del self.drone


    def getDrone(self, args, handlers):

        connection_string = args.connect

        #sitl = None
        if not connection_string:
            import dronekit_sitl
            self.sitl = dronekit_sitl.start_default(51.449, 5.492)
            #sitl = dronekit_sitl.SITL()
            #sitl.download("copter", "3.3")
            #sitl.launch(["--home=51.449,5.492,1,0 -- rate 30"], await_ready=True)
            # sitl.block_until_ready(verbose=True)
            connection_string = self.sitl.connection_string()

        return Cyclone(connection_string, configs, handlers=handlers)


    def initVision(self):
        camera = 0
        calibrationFile = "../Python-C++ interface/marker/drone_calibration.txt"
        if(self.simulation):
            calibrationFile = "../Python-C++ interface/marker/laptop_calibration.txt"

        self.vision = VisionThread(calibrationFile, self.drone, cameraID=camera, logging=self.logger, name="VisionThread")
        self.vision.start()


    def goToHoop(self):
        logging.info("Looking for path")
        
        points_to_cover = 10        # First n number of points to cover in the trajectory after every recalculation.

        list_location = []      # Tuples and lists for storing the trajectory information.
        frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED
        #startPosition = (0, 0, 0) # The position from which the path planning was calculated. All positions from the path are relative to this point.
        while self.drone.vehicle.mode.name == "GUIDED":

            self.vision.pathLock.acquire(True) # block until lock is aquired
            if self.vision.newPath:
                list_location = self.vision.path[:]
                self.vision.newPath = False
                fromPosition = self.vision.fromPosition
                self.vision.pathLock.release()
                self.drone.set_home_location(fromPosition)
                self.logger.info("Following path")
                self.followPath(list_location[:points_to_cover], frame)
                list_location = list_location[points_to_cover:]
            else:
                self.vision.pathLock.release()
                # self.logger.debug("Could not find path")
                if len(list_location) > 0:
                    self.logger.info("Following previous path")
                    self.followPath(list_location[:points_to_cover], frame)
                    list_location = list_location[:points_to_cover]


            #foundPath = ctypes.c_bool() # is set to true if a path is found, false otherwise
            #visualize = ctypes.c_bool(self.simulation) # True if the pathplanning should be visualized using opencv. This can be used for debug purposes.

            #trajectory = self.path_planning(ctypes.pointer(foundPath), visualize)
            #nrow = 100      # Size of the planned path.
            #ncol = 12



            #if foundPath:
            #    self.drone.set_home_location()                  # Reset EKF origin for further computations.
            #    # frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED     # Define the frame to use (local NED w.r.t. EKF origin).
            #    logging.info("Found path")
            #    list_location = []
            #    LocationTuples = []
            #    for i in range(nrow):
            #        # (x, y, z) waypoints w.r.t. the original position of the drone are
            #        # parsed at column 0, 4 and 8 of the computed path.
            #        # North is in the z direction, east is in the x direction and down is in the y direction.
            #        LocationTuples.append((matrix_index(trajectory, ncol, i, 8), matrix_index(
            #            trajectory, ncol, i, 0), matrix_index(trajectory, ncol, i, 4)))
            #        # For all the waypoints recoreded, convert them from local NED w.r.t.
            #        # the heading of the drone to global NED (rotating axes w.r.t. yaw angle).
            #        list_location = LocationTuples

            #    self.followPath(list_location[:points_to_cover], frame)
            #    list_location = list_location[points_to_cover:]
            #else:
            #    # The marker is not currently found
            #    logging.debug("Could not find path")
            #    if len(list_location) > 0:
            #        # We can folow some parts of the previous parts
            #        logging.info("Following previous path")
            #        self.followPath(list_location[:points_to_cover], frame)
            #        list_location = list_location[points_to_cover:]



    def followPath(self, list_location, frame):
        for i in range(0, len(list_location)):
            # Given the global NED waypoints w.r.t. the home location (EKF origin), navigate the drone by specifying the frame.

            #self.drone.goto_local_NED(list_location[i][0], list_location[i][1], list_location[i][2], frame)  #waits until target is reached
            # Offset calculation for positions
            dNorth = list_location[i][0]# - (self.drone.vehicle.location.local_frame.north - startPosition.north)
            dEast = list_location[i][1]# - (self.drone.vehicle.location.local_frame.east - startPosition.east)
            dDown = list_location[i][2]# - (self.drone.vehicle.location.local_frame.down - startPosition.down)
            logging.info("Point {} out of {}".format(i, len(list_location)))
            logging.info('Goto ({}, {}, {})'.format(dNorth, dEast, dDown))
            self.drone.goto_local_NED(dNorth, dEast, dDown, frame) # Waits until target is reached
        return



# Method for matrix indexing.
def matrix_index(a, rowsize, m, n):
    return(a[rowsize * n + m])


if __name__ == "__main__":
    myFlight = flight()
    try:
        myFlight.fly()
    except KeyboardInterrupt:
        myFlight.logger.info("Ending")
    myFlight.end()
    del myFlight

