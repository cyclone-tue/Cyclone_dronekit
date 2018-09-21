import configs
from pymavlink import mavutil
from cyclone import Cyclone
from dronekit import LocationGlobalRelative, LocationGlobal
import argparse


class flight():
    def __init__():
        self.drone = flight.getDrone()
        self.path_planning = flight.initVision()


    def fly(self):
        self.drone.awake_script()
        #drone.arm_and_takeoff(5)
        self.drone.obtain_home_location()
        self.drone.set_airspeed(5)
        
        remaining_waypoints, frame = self.goToHoop()
        self.followPath(remaining_waypoints, frame)
        

    def end(self):
        del self.drone


    def getDrone():
        parser = argparse.ArgumentParser(
            description='Print out vehicle state information. Connects to SITL on local PC by default.')
        parser.add_argument(
            '--connect', help="vehicle connection target string. If not specified, SITL automatically started and used.")

        args = parser.parse_args()
        connection_string = args.connect

        #sitl = None
        if not connection_string:
            import dronekit_sitl
            sitl = dronekit_sitl.start_default()
            connection_string = sitl.connection_string()

        return Cyclone(connection_string, configs)    


    def initVision():
        import ctypes       # Import compiled library of vision and pathplanning.
        so = ctypes.cdll.LoadLibrary
        lib = so("../Python-C++ interface/libcpp_py_vision.so")
        path_planning = lib.output_to_py
        path_planning.restype = ctypes.POINTER(ctypes.c_double)
        return path_planning


    def goToHoop(self):
        
        points_to_cover = 10        # First n number of points to cover in the trajectory after every recalculation.


        path_planning_iteration = True      # Flag for deciding whether to plan the path (if hoop is still in sight).
        
        while path_planning_iteration:

            list_location = []      # Tuples and lists for storing the trajectory information.
            
            trajectory = self.path_planning()
            nrow = 100      # Size of the planned path.
            ncol = 12
            
            self.drone.set_home_location()                  # Reset EKF origin for further computations.
            home_yaw = self.drone.vehicle.attitude.yaw      # Record the heading of the drone after resetting EKF origin.
            frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED     # Define the frame to use (local NED w.r.t. EKF origin).

            
            # check whether hoop is in sight
            path_planning_iteration = False           # Flag for checking whether the hoop is still in sight (if not, computed path are all 0).
            for item in trajectory:
                if item != 0:
                    path_planning_iteration = True
                    break
            else:
                continue


            # extract waypoints from trajectory
            LocationTuples = []
            for i in range(nrow):
                # (x, y, z) waypoints w.r.t. the original position of the drone are parsed at column 0, 4 and 8 of the computed path.
                LocationTuples.append((matrix_index(trajectory, ncol, (i, 0)), matrix_index(
                    trajectory, ncol, (i, 4)), matrix_index(trajectory, ncol, (i, 8))))
                # For all the waypoints recoreded, convert them from local NED w.r.t. the heading of the drone to global NED (rotating axes w.r.t. yaw angle).    
                list_location.append(self.drone.local_NED_to_global_NED(*LocationTuples[i]), home_yaw)
                
            # fly through first few waypoints
            self.followPath(list_location[:points_to_cover], frame)
      
        return [list_location, frame]


    def followPath(self, list_location, frame):
        for i in range(0, len(list_location)):
            # Given the global NED waypoints w.r.t. the home location (EKF origin), navigate the drone by specifying the frame.
            print('Goto ({}, {}, {})'.format(list_location[i]))
            self.drone.goto_local_NED(list_location[i][0], list_location[i][1], list_location[i][2], frame)  #waits until target is reached
        return



# Method for matrix indexing.
def matrix_index(a, rowsize, (m, n)):
    return(a[rowsize * n + m])


if __name__ == "__main__":
    myFlight = flight()
    myFlight.fly()
    myFlight.end()
    del myFlight

