from dronekit import LocationGlobalRelative, LocationLocal

import configs
from pymavlink import mavutil
from cyclone import Cyclone
import ctypes
import argparse
import logging
from visionThread import VisionThread
from high_level import HighLevelThread

class flight():
    """
    Class containing all flight related code to fly through one hoop
    """
    def __init__(self):
        """
        Constructor for the flight class.
        :return:
            Instance of flight class
        """
        # Parser for program parameters. It parses the connect and simulate arguments
        parser = argparse.ArgumentParser(
            description='Print out vehicle state information. Connects to SITL on local PC by default.')
        parser.add_argument(
            '--connect', help="vehicle connection target string. Is not necessary if --simulate is specified. It will then start a sitl.")
        parser.add_argument(
            '--simulate', help="Include if you want to connect to a simulation in stead of running on an actual drone. The script will take off automatically."
        )

        args = parser.parse_args() # The arguments passed to the program.
        self.simulation = False
        if args.simulate:
            self.simulation = True

        # Logging setup methods.
        logging.basicConfig()
        self.logger = logging.Logger("onehoop")
        formatter = logging.Formatter("%(asctime)s [%(threadName)-12.12s] [%(name)s-%(levelname)-5.5s] %(message)s") # Format to use for logging.
        fileHandler = logging.FileHandler("onehoop.log", mode='w') # Output logging info to file.
        streamHandler = logging.StreamHandler() # Output logging info to stdout.
        fileHandler.setFormatter(formatter)
        streamHandler.setFormatter(formatter)
        self.logger.addHandler(fileHandler)
        self.logger.addHandler(streamHandler)
        handlers = [fileHandler, streamHandler]
        self.drone = self.getDrone(args, handlers) # Initialise the connection to the drone.
        self.initVision() # Initialise the vision thread
        self.initControl() # Initialise the high level control thread

    # This function is called after setup is completed and flies through the hoop.
    # It is possible to switch between manual and guided/autonomous mode while in this function.

    def fly(self):
        """
        Call this function after setup to initialize the flight code and start flying through the hoop.
        :return:
            none
        """
        print("this")
        if self.simulation:
            # self.drone.set_home_location()
            self.drone.wait_for_user()
            #self.drone.arm_and_takeoff(1.28) # Take of automatically if this is a simulation.
        else:
            self.drone.awake_script() # Wait for guided mode if this is not a simulation.
        self.drone.obtain_home_location() # Download the home location from the drone.
        self.drone.set_groundspeed(5) # Set the maximum horizontal airspeed to 5 m/s.
        print("that")

        while True:
            self.logger.debug("in loop")
            self.drone.obtain_home_location()
            self.goToHoop() # Try to go to the hoop. This only exits if we leave guided mode.
            if not self.simulation:
                self.drone.awake_script() # Wait for the drone to go into guided mode.



    def end(self):
        """
        This function cleans up the vision thread and connection to the drone.
        It also closes the sitl if it was running.
        :return:
            none
        """
        self.vision.stop.set() # Tell the vision thread to stop.
        self.high_level.stop.set()
        self.logger.debug("Joining vision")
        self.vision.join(20.0) # Wait for the vision thread to stop for maximally 20 seconds.
        self.logger.info("Stopped vision")
        self.logger.debug("Joingin high_level")
        self.high_level.join(20.0)
        self.logger.debug("Stopped high_level")
        self.drone.vehicle.close() # Close the connection to the drone.
        # del self.drone.vehicle
        if self.simulation and self.sitl != None:
            self.sitl.stop() # Stop the sitl
            del self.sitl
        del self.drone


    def getDrone(self, args, handlers):
        """
        Start a simulation if requested by not specifying a connection string.
        Then establish a connection to the drone using connection string.
        :param args: Th parsed command line arguments
        :param handlers: Handlers to pass to the Cyclone class
        :return: an instance of the Cyclone class
        """

        connection_string = args.connect

        #sitl = None
        if not connection_string: # start a sitl if connection string is not specified.
            import dronekit_sitl
            self.sitl = dronekit_sitl.start_default(51.449, 5.492) # Start the sitl at specified coordinates.
            #sitl = dronekit_sitl.SITL()
            #sitl.download("copter", "3.3")
            #sitl.launch(["--home=51.449,5.492,1,0 -- rate 30"], await_ready=True)
            # sitl.block_until_ready(verbose=True)
            connection_string = self.sitl.connection_string() # Retrieve the connection string.

        return Cyclone(connection_string, configs, handlers=handlers) # Return a Cyclone object connected to the drone.


    def initVision(self):

        """
        Initialize and start the vision thread
        :return:
            none
        """
        camera = 0 # The camera to use as input
        calibrationFile = "../Python-C++ interface/marker/Vision/drone_calibration.txt" # Use the drone calibration if not in simulation
        if(self.simulation):
            calibrationFile = "../Python-C++ interface/marker/Vision/laptop_calibration.txt" # Use laptop calibration if in simulation

        self.vision = VisionThread(calibrationFile, self.drone, cameraID=camera, logging=self.logger, name="VisionThread") # Initialize the vision thread
        self.vision.start() # Start the vision thread.

    def initControl(self):
        self.high_level=HighLevelThread(name="HighLevelThread",drone=self.drone, logging=self.logger) #TODO move to initHighlevel
        self.high_level.start()


    def goToHoop(self):

        self.invalidatePath()

        points_to_cover = 10        # First n number of points to cover in the trajectory after every recalculation.

        # list_location = []      # Tuples and lists for storing the trajectory information.
        frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED
        #startPosition = (0, 0, 0) # The position from which the path planning was calculated. All positions from the path are relative to this point.
        while self.drone.vehicle.mode.name == "GUIDED":

            if self.getPath():
                #self.drone.set_home_location()                  # Reset EKF origin for further computations.
                frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED     # Define the frame to use (local NED w.r.t. EKF origin).
                print("Found path 2")
                LocationTuples = []
                #for i in range(nrow):
                # (x, y, z) waypoints w.r.t. the original position of the drone are
                # parsed at column 0, 4 and 8 of the computed path.
                # North is in the z direction, east is in the x direction and down is in the y direction.
                #LocationTuples.append((matrix_index(trajectory, ncol, 0, i), matrix_index(
                # trajectory, ncol, 1, i), matrix_index(trajectory, ncol, 2, i)))
                # For all the waypoints recoreded, convert them from local NED w.r.t.
                # the heading of the drone to global NED (rotating axes w.r.t. yaw angle).
                #list_location = LocationTuples

                #print(trajectory[0])
                #print(trajectory[1])
                #print(list_location)

                #self.followPath(self.list_location[:points_to_cover], frame)
                #self.list_location = self.list_location[points_to_cover:]
                self.high_level.passPath(self.list_location)

            else:
                pass
                # self.logger.debug("Could not find path")
                #if len(self.list_location) > 0:
                #    pass
                #    # self.logger.info("Following previous path")
                #    self.followPath(self.list_location[:points_to_cover], frame) #TODO uncomment this
                #    self.list_location = self.list_location[points_to_cover:] #TODO uncomment this

    def getPath(self):
        self.vision.pathLock.acquire(True) # block until lock is aquired
        if self.vision.newPath:
            self.fromPosition = self.vision.fromPosition
            self.fromAttitude = self.vision.fromAttitude
            self.list_location = self.vision.path[:]
            # self.logger.debug("Last location is: {}, {}, {}".format(list_location[99][0], list_location[99][1], list_location[99][2]))
            self.vision.newPath = False
            self.vision.pathLock.release()
            return True
        else:
            self.vision.pathLock.release()
            return False

    def invalidatePath(self):
        self.getPath()
        self.list_location = []

    def followPath(self, list_location, frame):
        for i in range(0, len(list_location)):

            # Given the global NED waypoints w.r.t. the home location (EKF origin), navigate the drone by specifying the frame.

            #self.drone.goto_local_NED(list_location[i][0], list_location[i][1], list_location[i][2], frame)  #waits until target is reached
            # Offset calculation for positions
            posNorth = list_location[i][0]# - (self.drone.vehicle.location.local_frame.north - startPosition.north)
            posEast = list_location[i][1]# - (self.drone.vehicle.location.local_frame.east - startPosition.east)
            posDown = list_location[i][2]# - (self.drone.vehicle.location.local_frame.down - startPosition.down)

            currentLocation = self.drone.vehicle.location.local_frame
            targetLocation = LocationLocal(posNorth, posEast, posDown)

            distance = self.drone.get_distance_metres_EKF(currentLocation, targetLocation)
            if distance >= self.drone.minimum_distance_threshold:
                self.logger.info("Point {} out of {}".format(i, len(list_location)))
                self.logger.info('Goto ({}, {}, {})'.format(posNorth, posEast, posDown))
                # self.logger.info("Start location ({}, {}, {})".format(startLocation.north, startLocation.east, startLocation.down))
                self.drone.goto_local_NED(posNorth, posEast, posDown, frame) # Waits until target is reached
            else:
                self.logger.warn("Point {} out of {} not considered. Distance too close.".format(i, len(list_location)))
                self.logger.info("Distance was {} meter".format(distance))



# Method for indexing array as if it is a matrix.
def matrix_index(a, rowsize, m, n):
    return(a[rowsize * m + n])


if __name__ == "__main__":
    myFlight = flight()
    try:
        myFlight.fly()
    except KeyboardInterrupt:
        myFlight.logger.info("Ending")
    myFlight.end()
    del myFlight
