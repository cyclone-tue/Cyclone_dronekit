import threading
import ctypes

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
        self.pathLock = threading.Lock()
        self.logging = logging
        self.stop = threading.Event()
        self.drone = drone


    def initVision(self, calibrationFile, cameraID=0):
        # Import compiled library of vision and pathplanning.
        so = ctypes.cdll.LoadLibrary
        lib = so("../Python-C++ interface/libCycloneVision.so")
        self.logging.info("Loaded vision library")
        setup = lib.setupVariables
        camera = cameraID
        calibration = calibrationFile
        setup(camera, calibration)
        self.logging.info("Ran setup using camera={} and calibration={}".format(camera, calibration))
        path_planning = lib.output_to_py
        path_planning.restype = ctypes.POINTER(ctypes.c_double)
        self.path_planner = path_planning
        self.cleanup = lib.cleanup


    def run(self):
        self.logging.info("Starting vision thread")
        self.initVision(self.calibrationFile, cameraID=self.cameraID)
        while not self.stop.isSet():
            foundPath = ctypes.c_bool() # is set to true if a path is found, false otherwise
            visualize = ctypes.c_bool(True) # True if the pathplanning should be visualized using opencv. This can be used for debug purposes.

            trajectory = self.path_planner(ctypes.pointer(foundPath), visualize)
            nrow = 100      # Size of the planned path.
            ncol = 12

            if foundPath:
                self.logging.debug("Found path")
                self.pathLock.acquire(True) # block until lock is aquired
                self.fromPosition = self.drone.vehicle.location
                self.path = []
                self.newPath = True
                for i in range(nrow):
                    # (x, y, z) waypoints w.r.t. the original position of the drone are
                    # parsed at column 0, 4 and 8 of the computed path.
                    # North is in the z direction, east is in the x direction and down is in the y direction.
                    self.path.append((matrix_index(trajectory, ncol, i, 8), matrix_index(
                        trajectory, ncol, i, 0), matrix_index(trajectory, ncol, i, 4)))

                self.pathLock.release()
        self.cleanup()
        self.logging.info("Stopping vision thread")

# Method for matrix indexing.
def matrix_index(a, rowsize, m, n):
    return(a[rowsize * n + m])