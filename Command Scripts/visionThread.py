import threading
import ctypes
from dronekit import LocationLocal, Attitude


class VisionThread(threading.Thread):


    def __init__(self, calibrationFile, drone, cameraID=0, logging=None, group=None, target=None, name=None,
                 args=(), kwargs=None, verbose=None):
        threading.Thread.__init__(self, group=group, target=target, name=name, verbose=verbose)
        self.args = args
        self.kwargs = kwargs
        self.path = []
        self.calibrationFile = calibrationFile
        self.cameraID = cameraID
        self.newPath = False
        self.fromPosition = None
        self.fromAttitude = None
        self.pathLock = threading.Lock()
        self.logging = logging
        self.stop = threading.Event()
        self.drone = drone


    def initVision(self, calibrationFile, cameraID=0):
        # Import compiled library of vision and pathplanning.
        so = ctypes.cdll.LoadLibrary
        lib = so("../Python-C++ interface/libCycloneVision.so")
        self.logging.info("Loaded vision library")
        setup = lib.setup
        camera = cameraID
        calibration = calibrationFile
        self.logging.info("Ran setup using camera={} and calibration={}".format(camera, calibration))
        setup(calibration)
        self.path_planner = lib.output_to_py
        self.path_planner.restype = ctypes.POINTER(ctypes.c_double)
        self.cleanup = lib.cleanup


    def run(self):
        self.logging.info("Starting vision thread")
        self.initVision(self.calibrationFile, cameraID=self.cameraID)
        while not self.stop.isSet():
            currentState = self.drone.get_state()
            currentState[0] = 0
            currentState[1] = 0
            currentState[2] = 0
            currentState[3] = 0
            currentState[4] = 0
            currentState[5] = 0
            currentState[6] = 0
            currentState[7] = 0
            currentState[8] = 0
            currentState[9] = 0
            currentState[10] = 0
            currentState[11] = 0

            currentTorque = self.drone.get_torques_and_thrust()
            currentStateC = (ctypes.c_double * len(currentState))(*currentState)
            currentTorqueC = (ctypes.c_double * len(currentTorque))(*currentTorque)
            pathLength = ctypes.c_int()                     # is set to true if a path is found, false otherwise
            visualize = ctypes.c_bool(True)      # True if the pathplanning should be visualized using opencv. This can be used for debug purposes.
            trajectory = self.path_planner(ctypes.pointer(currentStateC), ctypes.pointer(currentTorqueC), ctypes.pointer(pathLength), visualize)
            #foundPath = ctypes.c_bool()  # is set to true if a path is found, false otherwise
            #visualize = ctypes.c_bool(True)  # True if the pathplanning should be visualized using opencv. This can be used for debug purposes.

            #trajectory = self.path_planner(ctypes.pointer(foundPath), visualize)
            ncol = int(pathLength.value)    # Size of the planned path.
            nrow = 17

            if pathLength.value != 0:
                self.logging.debug("Found path")
                self.pathLock.acquire(True) # block until lock is aquired
                fromLocation = self.drone.vehicle.location.local_frame
                fromRotation = self.drone.vehicle.attitude
                self.fromPosition = LocationLocal(fromLocation.north, fromLocation.east, fromLocation.down)
                self.fromAttitude = Attitude(0, fromRotation.yaw, 0)
                self.path = []
                self.newPath = True
                for i in range(ncol):
                    point = LocationLocal(matrix_index(trajectory, ncol, i, 0), matrix_index(trajectory, ncol, i, 1), matrix_index(trajectory, ncol, i, 2))
                    #self.logging.debug("Point {} of path information: {}, {}, {}".format(i, point.north, point.east, point.down))
                    rotated = self.drone.rotate_location(point, self.fromAttitude)
                    translated = self.drone.translate_location(rotated, fromLocation)
                    self.path.append((translated.north, translated.east, translated.down))
                    # (x, y, z) waypoints w.r.t. the original position of the drone are
                    # parsed at column 0, 4 and 8 of the computed path.
                    # North is in the z direction, east is in the x direction and down is in the y direction.
                    #self.path.append((matrix_index(trajectory, nrow, i, 8), matrix_index(
                        #trajectory, nrow, i, 0), matrix_index(trajectory, nrow, i, 4)))
                # self.logging.debug("Last location is: north: {}, east: {}, down: {}".format(self.path[nrow - 1][0], self.path[nrow - 1][1], self.path[nrow -1][2]))
                self.pathLock.release()
        self.cleanup()
        self.logging.info("Stopping vision thread")


# Method for matrix indexing.
def matrix_index(a, rowsize, m, n):
    return(a[rowsize * n + m])