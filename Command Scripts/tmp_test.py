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
parser.add_argument(
    '--simulate', help="Specify if the vehicle should act like a simulation."
)

logging.basicConfig()
formatter = logging.Formatter("%(asctime)s [%(threadName)-12.12s] [%(name)s-%(levelname)-5.5s] %(message)s")
streamHandler = logging.StreamHandler()
streamHandler.setFormatter(formatter)
handlers = [streamHandler]


args = parser.parse_args()
connection_string = args.connect
simulate = False
if args.simulate:
    simulate = True

sitl = None

if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

drone = Cyclone(connection_string, configs, handlers=handlers)
# drone.awake_script()
drone.obtain_home_location()
if simulate:
    groundHeight =drone.vehicle.location.global_relative_frame.alt
    drone.arm_and_takeoff(groundHeight + 5)
else:
    drone.wait_for_user()
#drone.set_home_location()
#print("Set home location. Waiting for 10 seconds")
#time.sleep(10)
drone.set_airspeed(5)
startPosition = drone.vehicle.location.local_frame
print("Initialized")
#drone.goto_global_NED(0, 5, 0)
drone.goto_local_NED(startPosition.north + 5, startPosition.east + 0, startPosition.down -5, mavutil.mavlink.MAV_FRAME_LOCAL_NED)
print(drone.vehicle.location.local_frame)
drone.goto_local_NED(startPosition.north + 5, startPosition.east + 5, startPosition.down -5, mavutil.mavlink.MAV_FRAME_LOCAL_NED)
print(drone.vehicle.location.local_frame)
drone.goto_local_NED(startPosition.north + 5, startPosition.east + 5, startPosition.down + 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED)
print(drone.vehicle.location.local_frame)

drone.obtain_home_location()
del drone