from dronekit import LocationGlobalRelative

import configs
from pymavlink import mavutil
from cyclone import Cyclone
import ctypes
import argparse
import logging
from visionThread import VisionThread

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


    # This function is called after setup is completed and flies through the hoop.
    # It is possible to switch between manual and guided/autonomous mode while in this function.
    def fly(self):
        """
        Call this function after setup to initialize the flight code and start flying through the hoop.
        :return:
            none
        """
        if self.simulation:
            # self.drone.set_home_location()
            self.drone.arm_and_takeoff(1.28) # Take of automatically if this is a simulation.
        else:
            self.drone.awake_script() # Wait for guided mode if this is not a simulation.
        self.drone.obtain_home_location() # Download the home location from the drone.
        self.drone.set_airspeed(5) # Set the maximum horizontal airspeed to 5 m/s.

        while True:
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
        self.logger.debug("Joining vision")
        self.vision.join(20.0) # Wait for the vision thread to stop for maximally 20 seconds.
        self.logger.info("Stopped vision")
        self.drone.vehicle.close() # Close the connection to the drone.
        # del self.drone.vehicle
        if self.simulation and self.sitl:
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
        calibrationFile = "../Python-C++ interface/marker/drone_calibration.txt" # Use the drone calibration if not in simulation
        if(self.simulation):
            calibrationFile = "../Python-C++ interface/marker/laptop_calibration.txt" # Use laptop calibration if in simulation

        self.vision = VisionThread(calibrationFile, self.drone, cameraID=camera, logging=self.logger, name="VisionThread") # Initialize the vision thread
        self.vision.start() # Start the vision thread.


    def goToHoop(self):

        self.invalidatePath()
        
        points_to_cover = 10        # First n number of points to cover in the trajectory after every recalculation.

        # list_location = []      # Tuples and lists for storing the trajectory information.
        frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED
        #startPosition = (0, 0, 0) # The position from which the path planning was calculated. All positions from the path are relative to this point.
        while self.drone.vehicle.mode.name == "GUIDED":

            currentState = self.drone.get_state()
            currentState[0] = 0
            currentState[1] = 0
            currentState[2] = 0
            currentTorque = self.drone.get_torques_and_thrust()
            currentStateC = (ctypes.c_double * len(currentState))(*currentState)
            currentTorqueC = (ctypes.c_double * len(currentTorque))(*currentTorque)

            pathLength = ctypes.c_int()                     # is set to true if a path is found, false otherwise
            visualize = ctypes.c_bool(self.simulation)      # True if the pathplanning should be visualized using opencv. This can be used for debug purposes.
            trajectory = self.path_planning(ctypes.pointer(currentStateC), ctypes.pointer(currentTorqueC), ctypes.pointer(pathLength), visualize)
            ncol = int(pathLength.value)      # Size of the planned path.
            nrow = 17


            if pathLength.value != 0:
                self.drone.set_home_location()                  # Reset EKF origin for further computations.
                frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED     # Define the frame to use (local NED w.r.t. EKF origin).
                print("Found path")
                list_location = []
                LocationTuples = []
                for i in range(nrow):
                    # (x, y, z) waypoints w.r.t. the original position of the drone are
                    # parsed at column 0, 4 and 8 of the computed path.
                    # North is in the z direction, east is in the x direction and down is in the y direction.
                    LocationTuples.append((matrix_index(trajectory, ncol, 0, i), matrix_index(
                        trajectory, ncol, 1, i), matrix_index(trajectory, ncol, 2, i)))
                    # For all the waypoints recoreded, convert them from local NED w.r.t.
                    # the heading of the drone to global NED (rotating axes w.r.t. yaw angle).
                    list_location = LocationTuples

                print(trajectory[0])
                print(trajectory[1])
                print(list_location)

                self.followPath(list_location[:points_to_cover], frame)
                list_location = list_location[points_to_cover:]
            else:
                # self.logger.debug("Could not find path")
                if len(self.list_location) > 0:
                    pass
                    # self.logger.info("Following previous path")
                    self.followPath(self.list_location[:points_to_cover], self.fromPosition, frame) #TODO uncomment this
                    self.list_location = self.list_location[:points_to_cover] #TODO uncomment this

    def getPath(self):
        self.vision.pathLock.acquire(True) # block until lock is aquired
        if self.vision.newPath:
            self.list_location = self.vision.path[:]
            # self.logger.debug("Last location is: {}, {}, {}".format(list_location[99][0], list_location[99][1], list_location[99][2]))
            self.vision.newPath = False
            self.fromPosition = self.vision.fromPosition
            self.vision.pathLock.release()
            return True
        else:
            self.vision.pathLock.release()
            return False

    def invalidatePath(self):
        self.getPath()
        self.list_location = []

    def followPath(self, list_location, startLocation, frame):
        for i in range(0, len(list_location)):
            # Given the global NED waypoints w.r.t. the home location (EKF origin), navigate the drone by specifying the frame.

            #self.drone.goto_local_NED(list_location[i][0], list_location[i][1], list_location[i][2], frame)  #waits until target is reached
            # Offset calculation for positions
            dNorth = list_location[i][0] + startLocation.north# - (self.drone.vehicle.location.local_frame.north - startPosition.north)
            dEast = list_location[i][1] + startLocation.east# - (self.drone.vehicle.location.local_frame.east - startPosition.east)
            dDown = list_location[i][2] + startLocation.down# - (self.drone.vehicle.location.local_frame.down - startPosition.down)
            self.logger.info("Point {} out of {}".format(i, len(list_location)))
            self.logger.info('Goto ({}, {}, {})'.format(dNorth, dEast, dDown))
            # self.logger.info("Start location ({}, {}, {})".format(startLocation.north, startLocation.east, startLocation.down))
            self.drone.goto_local_NED(dNorth, dEast, dDown, frame) # Waits until target is reached
        return



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

