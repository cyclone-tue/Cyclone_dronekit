"""
This is the main class for the drone. It includes all the methods used.
The exact command scripts should import this class by "from cyclone import Cyclone".
"""
from dronekit import connect, Vehicle, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
import math


class Cyclone(object):

    def __init__(self, connection_string, configs):
        """Constructor.
        This is the contructor for the Cyclone class.
        It connects to the drone and initialize the parameters set in configs.

        Args:
            connection_string: Connection target to be defined in the command script
            configs: Parameters to be imported in the command script
        Returns:
            nothing
        """
        self.vehicle = connect(connection_string, wait_ready=True)
        print "Connected to the vehicle..."
        self.doPause = False
        self.earth_radius = configs.earth_radius
        self.meters_per_degree = configs.meters_per_degree
        self.sleep_time = configs.sleep_time
        self.distance_threshold = configs.distance_threshold

    def __del__(self):
        """Destructor.
        This is the destructor for the Cyclone class.
        It closes the connection to the drone.

        Args:
            nothing
        Returns:
            nothing
        """
        self.vehicle.close()

    # Basic methods

    def set_airspeed(self, airspeed):
        """Setter for airspeed.
        It sets the maximum allowed airspeed for the actuation.

        Args:
            airspeed: Maximum allowed airspeed in [m/s]
        Returns:
            nothing
        """
        self.vehicle.airspeed = airspeed

    def set_groundspeed(self, groundspeed):
        """Setter for groundspeed.
        It sets the maximum allowed groundspeed for the actuation.

        Args:
            airspeed: Maximum allowed groundspeed in [m/s]
        Returns:
            nothing
        """
        self.vehicle.groundspeed = groundspeed

    def mode_guided(self):
        """Guided mode switch.
        It switches the drone to guided mode.

        Args:
            nothing
        Returns:
            nothing
        """
        self.vehicle.mode = VehicleMode("GUIDED")
        while not self.vehicle.mode.name == 'GUIDED':
            print "Changing into GUIDED mode"
            time.sleep(self.sleep_time)

    def mode_auto(self):
        """Auto mode switch.
        It switches the drone to auto mode.

        Args:
            nothing
        Returns:
            nothing
        """
        self.vehicle.mode = VehicleMode("AUTO")
        while not self.vehicle.mode.name == 'AUTO':
            print "Switching to AUTO mode..."
            time.sleep(self.sleep_time)

    def mode_rtl(self):
        """RTL mode switch.
        It switches the drone to Return-To-Launch mode.

        Args:
            nothing
        Returns:
            nothing
        """
        self.vehicle.mode = VehicleMode("RTL")
        while not self.vehicle.mode.name == "RTL":
            print('Switching to RTL mode...')
            time.sleep(self.sleep_time)

    def arm_check(self):
        """Checking for arming state.
        It checkes whether the drone is armed.

        Args:
            nothing
        Returns:
            nothing
        """
        while not self.vehicle.is_armable:
            print "Waiting for vehicle to initialise..."
            time.sleep(self.sleep_time)

        print "Drone is armable, please arm the drone."

        while not self.vehicle.armed:
            print "Waiting for arming the drone"
            time.sleep(self.sleep_time)

        print "Drone is armed."

    def awake_script(self):
        """Wakes up the script.
        It freezes the script and waits until the flight mode is changed to GUIDED manually.
        Usually it's used transition from human control to command scripts.

        Args:
            nothing
        Returns:
            nothing
        """
        while not self.vehicle.mode.name == 'GUIDED':
            print "(Main): Switch to GUIDED mode to awake the script"
            time.sleep(self.sleep_time)

    def pause(self):
        """Script pauser.
        It freezes the script when the pause condition is true.

        Args:
            nothing
        Returns:
            nothing
        """
        while self.doPause:
            time.sleep(self.sleep_time)

    # @self.vehicle.on_attribute('last_heartbeat')
    # def listener(self, attr_name, value):
    #     global doPause
    #     if value > 1.5 and not doPause:
    #         print "Pausing script due to bad link"
    #         doPause = True
    #     if value < 1.5 and doPause:
    #         doPause = False
    #         print "Un-pausing script"

    # Location/Distance estimations

    def global_NED_to_wp(self, original_location, dNorth, dEast, dDown):
        """Converter from global NED frame to global waypoint.
        It converts the given positions in the global NED frame to global waypoints defined in coordinates of WSG84 standard.

        Args:
            original_location: Drone's location
            dNorth: Target position in terms of North to the drone [m]
            dEast: Target position in terms of East to the drone [m]
            dDown: Target position in terms of Down to the drone [m]
        Returns:
            targetlocation: Target location in LocationGlobalRelative form
        """
        # Coordinate offsets in radians
        dLat = dNorth/self.earth_radius
        dLon = dEast/(self.earth_radius*math.cos(math.pi*original_location.lat/180))

        # New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        if type(original_location) is LocationGlobal:
            targetlocation = LocationGlobal(newlat, newlon, original_location.alt - dDown)
        elif type(original_location) is LocationGlobalRelative:
            targetlocation = LocationGlobalRelative(newlat, newlon, original_location.alt - dDown)
        else:
            raise Exception("Invalid Location object passed")

        return targetlocation

    def get_distance_metres(self, aLocation1, aLocation2):
        """Distance between two locations.
        It calculates the distance between two locations in [m].

        Args:
            aLocation1: First location in LocationGlobalRelative form
            aLocation2: Second location in LocationGlobalRelative form
        Returns:
            (distance): Distance between the two locations in [m]
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * self.meters_per_degree

    # TODO: Clarify origin and coordinate system used in the return path, currently assuming it's based on drone's position and heading
    def local_NED_to_global_NED(self, dNorth, dEast, dDown):
        """Converter from local NED to global NED
        It converts the local NED positions to a global NED positions.

        Args:
            dNorth: Position in terms of North(Front) to the drone [m]
            dEast: Position in terms of East(Right) to the drone [m]
            dDown: Position in terms of Down to the drone [m]
        Returns:
            (target): Tuples of positions in terms of global NED positions to the drone in [m]
        """
        yaw = self.vehicle.attitude.yaw
        global_dNorth = dNorth * math.cos(yaw) + dEast * math.cos(yaw + math.pi / 2)
        global_dEast = dNorth * math.sin(yaw) + dEast * math.sin(yaw + math.pi / 2)
        global_dDown = dDown
        return (global_dNorth, global_dEast, global_dDown)

    # Movement functions

    def arm_and_takeoff(self, aTargetAltitude):
        """Arm and takeoff.
        It actuates the drone to do pre-arm checks, arm the motors and take off to the set altitude.

        Args:
            aTargetAltitude: Target altitude to take off in [m]
        Returns:
            nothing
        """
        print "Basic pre-arm checks"
        while not self.vehicle.is_armable:
            print " Waiting for vehicle to initialise..."
            time.sleep(self.sleep_time)

        print "Arming motors"
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print " Waiting for arming..."
            time.sleep(self.sleep_time)

        print "Taking off!"
        self.vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

        while True:
            print " Altitude: ", self.vehicle.location.global_relative_frame.alt
            if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude*0.95:
                print "Reached target altitude"
                break
            time.sleep(self.sleep_time)

    def condition_yaw(self, heading, relative=False):
        """Set the yaw.
        It sets the yaw angle of the drone.

        Args:
            heading: Target yaw angle in [deg]
            relative: Flag for whether the target yaw angle is relative
        Returns:
            nothing
        """
        if relative:
            is_relative = 1  # yaw relative to direction of travel
        else:
            is_relative = 0  # yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative,  # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def goto_global_NED(self, dNorth, dEast, dDown):
        """Actuation method for a global NED target.
        It actuates the drone to fly to a global NED location.

        Args:
            dNorth: Target position in terms of North to the drone [m]
            dEast: Target position in terms of East to the drone [m]
            dDown: Target position in terms of Down to the drone [m]
        Returns:
            nothing
        """
        currentLocation = self.vehicle.location.global_relative_frame
        targetLocation = self.global_NED_to_wp(currentLocation, dNorth, dEast, dDown)
        targetDistance = self.get_distance_metres(currentLocation, targetLocation)
        self.vehicle.simple_goto(targetLocation)

        while self.vehicle.mode.name == "GUIDED":  # Stop action if we are no longer in guided mode.
            remainingDistance = self.get_distance_metres(
                self.vehicle.location.global_relative_frame, targetLocation)
            print "Distance to target: ", remainingDistance
            if remainingDistance <= self.distance_threshold:  # Just below target, in case of undershoot.
                print "Reached target"
                break
            time.sleep(self.sleep_time)


    def set_position_target_global_int(self, aLocation):
        """MAVLink command wrapping method for a navigatiing to a global WGS84 target.
        It actuates the drone to fly to a global WGS84 coordinate (MAVLink command: SET_POSITION_TARGET_GLOBAL_INT).

        Args:
            aLocation: Target location wrapped in global relative frame.
        Returns:
            nothing
        """
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
            0b0000111111111000,  # type_mask (only speeds enabled)
            aLocation.lat * 1e7,  # lat_int - X Position in WGS84 frame in 1e7 * meters
            aLocation.lon * 1e7,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
            aLocation.alt,
            # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
            0,  # X velocity in NED frame in m/s
            0,  # Y velocity in NED frame in m/s
            0,  # Z velocity in NED frame in m/s
            0, 0, 0,  # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def set_position_target_local_NED(self, dNorth, dEast, dDown):
        """Actuation method for a local NED target.
        It actuates the drone to fly to a local NED location.

        Args:
            dNorth: Position in terms of North(Front) to the drone [m]
            dEast: Position in terms of East(Right) to the drone [m]
            dDown: Position in terms of Down to the drone [m]
        Returns:
            nothing
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            # frame: positions relative to the current vehicle position and heading
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111111000,  # type_mask (only positions enabled)
            # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
            dNorth, dEast, dDown,
            0, 0, 0,  # x, y, z velocity in m/s  (not used)
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def set_velocity_local_NED(self, velocity_x, velocity_y, velocity_z, duration):
        """Actuation method for local NED velocities.
        It actuates the drone to fly to with local NED defined velocities.

        Args:
            velocity_x: Linear velocity of the drone in x(to the front) [m/s]
            velocity_y: Linear velocity of the drone in y(to the right) [m/s]
            velocity_z: Linear velocity of the drone in z(down) [m/s]
            duration: Duration for maintaining the velocities [s]
        Returns:
            nothing
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            # frame: velocities relative to the current vehicle position and heading
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle on 1 Hz cycle
        for x in range(0, duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)

    def goto_local_NED(self, dNorth, dEast, dDown):
        """Actuation method for a local NED target.
        It actuates the drone to fly to a local NED location.

        Args:
            dNorth: Target position in terms of North(Front) to the drone [m]
            dEast: Target position in terms of East(Right) to the drone [m]
            dDown: Target position in terms of Down to the drone [m]
        Returns:
            nothing
        """
        currentLocation = self.vehicle.location.global_relative_frame
        targetLocation = self.global_NED_to_wp(currentLocation, dNorth, dEast, dDown)
        targetDistance = self.get_distance_metres(currentLocation, targetLocation)
        self.set_position_target_local_NED(dNorth, dEast, dDown)

        while self.vehicle.mode.name == "GUIDED":
            remainingDistance = self.get_distance_metres(
                self.vehicle.location.global_relative_frame, targetLocation)
            print "Distance to target: ", remainingDistance
            if remainingDistance <= self.distance_threshold:
                print "Reached target"
                break
            time.sleep(self.sleep_time)

    def goto_wp_global(self, targetLocation, func='mav'):
        """Actuation method for global waypoint.
        It actuates the drone to fly to a global waypoint.

        Args:
            targetLocation: Target waypoint defined in LocationGlobalRelative
            func: Flag for which MAVLink command to use, 'mav' - MAV_CMD_NAV_WAYPOINT or 'set' - SET_POSITION_TARGET_GLOBAL_INT
        Returns:
            nothing
        """
        currentLocation = self.vehicle.location.global_relative_frame
        targetDistance = self.get_distance_metres(currentLocation, targetLocation)
        if (func == 'mav'):
            self.vehicle.simple_goto(targetLocation)
        else:
            self.set_position_target_global_int(targetLocation)

        while self.vehicle.mode.name == "GUIDED":
            remainingDistance = self.get_distance_metres(
                self.vehicle.location.global_relative_frame, targetLocation)
            print "Distance to target: ", remainingDistance
            if remainingDistance <= self.distance_threshold:
                print "Reached target"
                break
            time.sleep(self.sleep_time)

    # Mission Related Functions

    def download_mission(self):
        """Mission downloader.
        It downloads the mission from the Pixhawk storage.

        Args:
            nothing
        Returns:
            cmds: Comamnds downloaded
        """
        print('Downloading mission from the vehicle...')
        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready()
        return cmds

    def modify_mission(self, cmds, listofWP):
        """Mission modifier.
        It modifies the downloaded mission with a list of waypoints.

        Args:
            cmds: Downloaded commands
            listofWP: Predefined list of waypoints
        Returns:
            missionlist: List of planned mission commands
        """
        print('Modifying mission with the updated list of waypoints...')
        missionlist = []
        # Run Vision and Pathplanning, path = readPathPlanned()
        # for i in range(nrow):
        # listofWP.append( (matrix_index(path, nrow, (i, index_of_x)), matrix_index(path, nrow, (i, index_of_y)), matrix_index(path, nrow, (i, index_of_z)) )
        for i in range(len(listofWP)):
            # cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                          # 0, 0, 0, 0, 0, 0, listofWP[i][0], listofWP[i][1], -listofWP[i][2])
            cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                          0, 0, 2, 0, 0, 0, listofWP[i][0], listofWP[i][1], -listofWP[i][2])
            missionlist.append(cmd)
        return missionlist

    def update_mission(self, cmds, missionlist):
        """Mission updater.
        It updates the modified mission.

        Args:
            cmds: Downloaded commands
            missionlist: List of planned mission commands
        Returns:
            nothing
        """
        print('Updating modified mission...')
        cmds.clear()
        for cmd in missionlist:
            cmds.add(cmd)
        cmds.upload()

    def distance_to_current_waypoint(self):
        """Remaining distance to the next waypoint.
        It calculates the distance of the current waypoint to the next waypoint.

        Args:
            nothing
        Returns:
            distancetopoint: Remaining distance to the next waypoint in [m]
        """
        nextwaypoint = self.vehicle.commands.next
        if nextwaypoint == 0:
            return None
        missionitem = self.vehicle.commands[nextwaypoint-1]  # commands are zero indexed
        lat = missionitem.x
        lon = missionitem.y
        alt = missionitem.z
        targetWaypointLocation = LocationGlobalRelative(lat, lon, alt)
        distancetopoint = self.get_distance_metres(
            self.vehicle.location.global_relative_frame, targetWaypointLocation)
        return distancetopoint
