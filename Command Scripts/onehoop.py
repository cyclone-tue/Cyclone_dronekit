import configs
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

# Import compiled library of pathplanning
import ctypes
so = ctypes.cdll.LoadLibrary
lib = so("../Python-C++ interface/libcpp_py_vision.so")
LocationTuples = []
VelocityTuples = []
list_location = []
list_velocity = []


def matrix_index(a, nrow, (m, n)):
    return(a[nrow * n + m])

# Main Script:
drone.arm_and_takeoff(5)
drone.set_airspeed(5)

print "Vision&PP output_to_py(): \n"
output_to_py_func = lib.output_to_py
output_to_py_func.restype = ctypes.POINTER(ctypes.c_double)
converted = output_to_py_func()
nrow = 100
ncol = 12
print("Extracting location and velocity from every point in the trajectory...")
for i in range(nrow):
    LocationTuples.append((matrix_index(converted, nrow, (i, 0)), matrix_index(
        converted, nrow, (i, 4)), matrix_index(converted, nrow, (i, 8))))
    VelocityTuples.append((matrix_index(converted, nrow, (i, 3)), matrix_index(
        converted, nrow, (i, 4)), matrix_index(converted, nrow, (i, 5))))

for i in range(0, 99, 9):
    list_location.append(drone.local_NED_to_global_NED(*LocationTuples[i]))
    # list_velocity.append(VelocityTuples[i])

print('Coordinates of the waypoints:')
for row in list_location:
    print(row)

for i in range(len(list_location)):
    print('Start Mission %d' % (i + 1))
    if i == 0:
        drone.goto_global_NED(*list_location[i])
    else:
        drone.goto_global_NED(list_location[i][0] - list_location[i-1][0], list_location[i][1] - list_location[i-1][1], list_location[i][2] - list_location[i-1][2])

# for i in range(len(list_velocity)):
#     print('Start Mission %d' % (i + 1))
#     drone.set_velocity_local_NED(*list_velocity[i], 1)

del drone
