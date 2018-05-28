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
lib = so("../Python-C++ interface/libcpp_py_vision.so")
path_planning = lib.output_to_py
path_planning.restype = ctypes.POINTER(ctypes.c_double)

# Tuples and lists for storing the trajectory information.
LocationTuples = []
# VelocityTuples = []
list_location = []
# list_velocity = []

# Distance threshold used for the horizon receding control.
# It is hardcoded to be the distance limit for
# using the camera to find hoop and compute trajectory.
# TODO: Replaced by more dynamic approach, e.g. vision/pp returning feedback on whether hoop is visible
distance_threshold = 0.5

# First n number of points to cover in the trajectory after every recalculation.
points_to_cover = 10

# Method for matrix indexing.
def matrix_index(a, nrow, (m, n)):
    return(a[nrow * n + m])

# Main Script:
drone.awake_script()
# drone.arm_and_takeoff(5)
drone.obtain_home_location()
drone.set_airspeed(5)

distance_to_hoop = 1024
while (distance_to_hoop >= 0.5):
    print "path_planning(): \n"
    trajectory = path_planning()
    drone.set_home_location()
    home_yaw = drone.vehicle.attitude.yaw
    frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED
    nrow = 100
    ncol = 12
    print("Extracting location waypoints from every point in the trajectory...")
    for i in range(nrow):
        LocationTuples.append((matrix_index(trajectory, nrow, (i, 0)), matrix_index(
            trajectory, nrow, (i, 4)), matrix_index(trajectory, nrow, (i, 8))))
        # VelocityTuples.append((matrix_index(converted, nrow, (i, 3)), matrix_index(
        #     converted, nrow, (i, 4)), matrix_index(converted, nrow, (i, 5))))

    # for i in range(0, 99, 9):
    #     list_location.append(drone.local_NED_to_global_NED(*LocationTuples[i]))
        # list_velocity.append(VelocityTuples[i])
    for i in range(nrow):
        list_location.append(drone.local_NED_to_global_NED(*LocationTuples[i]), home_yaw)

    print('Coordinates in local NED of the waypoints (starting at path planning origin):')
    for row in list_location:
        print(row)

    # print('Coordinates in WGS84 of the waypoints:')
    # for row in list_location:
    #     print(drone.global_NED_to_wp(drone.vehicle.location.global_relative_frame, row[0], row[1], row[2]))

    for i in range(points_to_cover):
        print('Goto ({}, {}, {})'.format(list_location[i]))
        drone.goto_local_NED(list_location[i][0], list_location[i][1], list_location[i][2], frame)
        # if i == 0:
        #     drone.goto_global_NED(*list_location[i])
        # else:
        #     drone.goto_global_NED(list_location[i][0] - list_location[i-1][0], list_location[i][1] - list_location[i-1][1], list_location[i][2] - list_location[i-1][2])
    distance_to_hoop = 1 - drone.distance_covered_along_track(home_yaw)
for i in range(points_to_cover, len(list_location)):
    print('Goto ({}, {}, {})'.format(list_location[i]))
    drone.goto_local_NED(list_location[i][0], list_location[i][1], list_location[i][2], frame)
# for i in range(len(list_velocity)):
#     print('Start Mission %d' % (i + 1))
#     drone.set_velocity_local_NED(*list_velocity[i], 1)

del drone
