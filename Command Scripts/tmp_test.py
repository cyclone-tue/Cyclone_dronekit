import configs
from cyclone import Cyclone
from dronekit import LocationGlobalRelative, LocationLocal
from pymavlink import mavutil
import argparse
import time
import logging

parser = argparse.ArgumentParser(
    description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument(
    '--connect', help="vehicle connection target string. If not specified, SITL automatically started and used.")

logging.basicConfig()
formatter = logging.Formatter("%(asctime)s [%(threadName)-12.12s] [%(name)s-%(levelname)-5.5s] %(message)s")
streamHandler = logging.StreamHandler()
streamHandler.setFormatter(formatter)
handlers = [streamHandler]


args = parser.parse_args()
connection_string = args.connect

sitl = None

if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

drone = Cyclone(connection_string, configs, handlers=handlers)
# drone.awake_script()
drone.obtain_home_location()
drone.arm_and_takeoff(5)
drone.set_airspeed(5)
print(drone.vehicle.location.local_frame)
print("Initialized")
#drone.goto_global_NED(0, 5, 0)
drone.goto_local_NED(5, 0, -5, mavutil.mavlink.MAV_FRAME_LOCAL_NED)
print(drone.vehicle.location.local_frame)
drone.goto_local_NED(5, 5, -5, mavutil.mavlink.MAV_FRAME_LOCAL_NED)
print(drone.vehicle.location.local_frame)
drone.goto_local_NED(5, 5, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED)
print(drone.vehicle.location.local_frame)



del drone