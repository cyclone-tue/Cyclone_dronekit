import configs
from cyclone import Cyclone
from dronekit import LocationGlobalRelative
import argparse

# test_waypoint = (51.449153, 5.493041, 5)  # lattitude, longditude, height[m](w.r.t. home location)
# listWPs = [(51.449280, 5.493137, 5),
#            (51.449218, 5.493094, 5),
#            (51.449156, 5.493051, 5),
#            (51.449077, 5.493113, 5),
#            (51.448998, 5.493175, 5),
#            (51.4489925, 5.493336, 5),
#            (51.448987, 5.493497, 5),
#            (51.449061, 5.4935695, 5),
#            (51.449135,	5.493642, 5),
#            (51.4492035, 5.493595, 5),
#            (51.449272,	5.493548, 5)]

listWPs = [(51.44907379, 5.49353947, 1.3),
           (51.44907165, 5.49347068, 1.3),
           (51.44906840, 5.49361497, 1.3)]


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
for i in range(len(listWPs)):
    print('Start Mission %d' % (i + 1))
    drone.goto_wp_global(LocationGlobalRelative(*listWPs[i]))
# drone.mode_rtl()
del drone
