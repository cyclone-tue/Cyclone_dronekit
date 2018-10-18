import configs
from cyclone import Cyclone
from dronekit import LocationGlobalRelative, LocationLocal
from pymavlink import mavutil
import argparse
import time

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
# drone.awake_script()
drone.arm_and_takeoff(5)
drone.set_airspeed(5)
drone.set_home_location()
print(drone.vehicle.location.local_frame)
# drone.goto_global_NED(0, 5, 0)
drone.goto_local_NED(5, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED)
print(drone.vehicle.location.local_frame)
drone.goto_local_NED(5, 5, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED)
print(drone.vehicle.location.local_frame)
drone.goto_local_NED(5, 5, 5, mavutil.mavlink.MAV_FRAME_LOCAL_NED)
print(drone.vehicle.location.local_frame)
del drone