import configs
from pymavlink import mavutil
from cyclone import Cyclone
from dronekit import LocationGlobalRelative, LocationGlobal

import argparse
parser = argparse.ArgumentParser(
    description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument(
    '--connect', help="vehicle connection target string. If not specified, SITL automatically started and used.")

args = parser.parse_args()
connection_string = args.connect

sitl = None

if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

drone = Cyclone(connection_string, configs)

# Import compiled library of vision and pathplanning.
import ctypes
so = ctypes.cdll.LoadLibrary
lib = so("../Python-C++ interface/test.so")
path_planning = lib.output_to_py
path_planning.restype = ctypes.POINTER(ctypes.c_double)

# Tuples and lists for storing the trajectory information.
LocationTuples = []
# VelocityTuples = []
list_location = []
# list_velocity = []

# Flag for deciding whether to plan the path (if hoop is still in sight).
path_planning_iteration = True

# First n number of points to cover in the trajectory after every recalculation.
points_to_cover = 10

# Method for matrix indexing.
def matrix_index(a, nrow, (m, n)):
    return(a[nrow * n + m])

# Main Script:
#drone.awake_script()
# drone.arm_and_takeoff(5)
drone.obtain_home_location()
drone.set_airspeed(5)

while (path_planning_iteration):
    print "path_planning(): \n"
    trajectory = path_planning()
    # Reset EKF origin for further computations.
    drone.set_home_location()

    # Record the heading of the drone after resetting EKF origin.
    home_yaw = drone.vehicle.attitude.yaw

    # Define the frame to use (local NED w.r.t. EKF origin).
    frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED

    # Size of the planned path.
    nrow = 100
    ncol = 12

    # Flag for checking whether the hoop is still in sight (if not, computed path are all 0).
    check_zero = True
    print("Extracting location waypoints from every point in the trajectory...")
    for i in range(nrow):
        # (x, y, z) waypoints w.r.t. the original position of the drone are parsed at column 0, 4 and 8 of the computed path.
        LocationTuples.append((matrix_index(trajectory, nrow, (i, 0)), matrix_index(
            trajectory, nrow, (i, 4)), matrix_index(trajectory, nrow, (i, 8))))

        # If any of the value of the path is non-zero, change the flag.
        if (matrix_index(trajectory, nrow, (i, 0)) != 0 or
                matrix_index(trajectory, nrow, (i, 1)) != 0 or
                matrix_index(trajectory, nrow, (i, 2)) != 0 or
                matrix_index(trajectory, nrow, (i, 3)) != 0 or
                matrix_index(trajectory, nrow, (i, 4)) != 0 or
                matrix_index(trajectory, nrow, (i, 5)) != 0 or
                matrix_index(trajectory, nrow, (i, 6)) != 0 or
                matrix_index(trajectory, nrow, (i, 7)) != 0 or
                matrix_index(trajectory, nrow, (i, 8)) != 0 or
                matrix_index(trajectory, nrow, (i, 9)) != 0 or
                matrix_index(trajectory, nrow, (i, 10)) != 0 or
                matrix_index(trajectory, nrow, (i, 11)) != 0):
            check_zero = False
        # VelocityTuples.append((matrix_index(converted, nrow, (i, 3)), matrix_index(
        #     converted, nrow, (i, 4)), matrix_index(converted, nrow, (i, 5))))

    # for i in range(0, 99, 9):
    #     list_location.append(drone.local_NED_to_global_NED(*LocationTuples[i]))
        # list_velocity.append(VelocityTuples[i])
    for i in range(nrow):
        # For all the waypoints recoreded, convert them from local NED w.r.t. the heading of the drone to global NED (rotating axes w.r.t. yaw angle).
        list_location.append(drone.local_NED_to_global_NED(*LocationTuples[i]), home_yaw)

    print('Coordinates in local NED of the waypoints (starting at path planning origin):')
    for row in list_location:
        print(row)

    # print('Coordinates in WGS84 of the waypoints:')
    # for row in list_location:
    #     print(drone.global_NED_to_wp(drone.vehicle.location.global_relative_frame, row[0], row[1], row[2]))

    for i in range(points_to_cover):
        # Given the global NED waypoints w.r.t. the home location (EKF origin), navigate the drone by specifying the frame.
        print('Goto ({}, {}, {})'.format(list_location[i]))
        drone.goto_local_NED(list_location[i][0], list_location[i][1], list_location[i][2], frame)
        # if i == 0:
        #     drone.goto_global_NED(*list_location[i])
        # else:
        #     drone.goto_global_NED(list_location[i][0] - list_location[i-1][0], list_location[i][1] - list_location[i-1][1], list_location[i][2] - list_location[i-1][2])
    # Update the distance of the current position of the drone to the hoop. (1m is the distance of the drone from the hoop originally).
    distance_to_hoop = 1 - drone.distance_covered_along_track(home_yaw)

# When the vision cannot be used anymore, continue the navigation of the most recently computed path.
for i in range(points_to_cover, len(list_location)):
    print('Goto ({}, {}, {})'.format(list_location[i]))
    drone.goto_local_NED(list_location[i][0], list_location[i][1], list_location[i][2], frame)

# for i in range(len(list_velocity)):
#     print('Start Mission %d' % (i + 1))
#     drone.set_velocity_local_NED(*list_velocity[i], 1)

del drone
