import math

import configs
from cyclone import Cyclone
from dronekit import LocationGlobalRelative, LocationLocal, Attitude
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
if simulate:
    groundHeight =drone.vehicle.location.global_relative_frame.alt
    drone.arm_and_takeoff(groundHeight + 5)
    drone.obtain_home_location()
else:
    drone.wait_for_user()
drone.obtain_home_location()
#drone.set_home_location()
#print("Set home location. Waiting for 10 seconds")
#time.sleep(10)
drone.set_airspeed(5)
startPosition = drone.vehicle.location.local_frame
ang = drone.vehicle.attitude
print("Roll, Pitch, Yaw: {}, {}, {}".format(ang.roll, ang.pitch, ang.yaw))
print("Initialized")
#drone.goto_global_NED(0, 5, 0)
point1 = drone.translate_location(drone.rotate_location(LocationLocal(5, 0, -5), Attitude(0, ang.yaw, 0)), startPosition)
point2 = drone.translate_location(drone.rotate_location(LocationLocal(5, 5, -5), Attitude(0, ang.yaw, 0)), startPosition)
point3 = drone.translate_location(drone.rotate_location(LocationLocal(5, 5, 0), Attitude(0, ang.yaw, 0)), startPosition)
print("Point1: {}, {}, {}".format(point1.north, point1.east, point1.down))
print("Point2: {}, {}, {}".format(point2.north, point2.east, point2.down))
print("Point3: {}, {}, {}".format(point3.north, point3.east, point3.down))
drone.goto_local_NED(point1.north, point1.east, point1.down, mavutil.mavlink.MAV_FRAME_LOCAL_NED)
print(drone.vehicle.location.local_frame)
drone.goto_local_NED(point2.north, point2.east, point2.down, mavutil.mavlink.MAV_FRAME_LOCAL_NED)
print(drone.vehicle.location.local_frame)
drone.goto_local_NED(point3.north, point3.east, point3.down, mavutil.mavlink.MAV_FRAME_LOCAL_NED)
print(drone.vehicle.location.local_frame)

drone.obtain_home_location()
del drone