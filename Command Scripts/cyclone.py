"""
This is the main class for the drone. It includes all the methods used.
The exact command scripts should import this class by "from cyclone import Cyclone".
"""
from dronekit import connect, Vehicle, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal,Command
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
        print("Connecting to vehicle using: " + connection_string )
        self.vehicle = connect(connection_string, wait_ready=True)
        print("Connected to the vehicle...")
        self.doPause = False
        self.earth_radius = configs.earth_radius
        self.meters_per_degree = configs.meters_per_degree
        self.sleep_time = configs.sleep_time
        self.distance_threshold = configs.distance_threshold
        self.coordinate_threshold = configs.coordinate_threshold
        self.local_home = LocationLocal(0, 0, 0)

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
            print("Changing into GUIDED mode")
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
            print("Switching to AUTO mode...")
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
            print("Waiting for vehicle to initialise...")
            time.sleep(self.sleep_time)

        print("Drone is armable, please arm the drone.")

        while not self.vehicle.armed:
            print("Waiting for arming the drone")
            time.sleep(self.sleep_time)

        print("Drone is armed.")

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
            print("(Main): Switch to GUIDED mode to awake the script")
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

    def obtain_home_location(self):
        """Wait for the home location to be downloaded from the drone."""
        while not self.vehicle.home_location:
            cmds = self.vehicle.commands
            cmds.download()
            cmds.wait_ready()
            if not self.vehicle.home_location:
                print('Obtaining home location...')
        print("Home Location: %s" % self.vehicle.home_location)

    def set_home_location(self):
        """Define the current location of the drone as the home location."""
        self.vehicle.home_location = self.vehicle.location.global_frame
        self.local_home = self.vehicle.location.local_frame

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

    def get_distance_metres_EKF(self, aLocation1, aLocation2):
        """Distance between two local locations.

        Args:
            aLocation1: First location in LocationLocal form
            aLocation2: Second location in LocationLocal form
        Returns:
            (distance): Distance between the two locations in [m]
        """
        dNorth = aLocation2.north - aLocation1.north
        dEast = aLocation2.east - aLocation1.east
        dDown = aLocation2.down - aLocation1.down
        return math.sqrt(dNorth**2 + dEast**2 + dDown**2)


    def get_distance_metres(self, aLocation1, aLocation2):
        """Distance between two global locations.
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

    def local_NED_to_global_NED(self, dNorth, dEast, dDown, yaw):
        """Converter from local NED to global NED
        It converts the local NED positions to a global NED positions.

        Args:
            dNorth: Position in terms of North(Front) to the drone [m]
            dEast: Position in terms of East(Right) to the drone [m]
            dDown: Position in terms of Down to the drone [m]
        Returns:
            (target): Tuples of positions in terms of global NED positions to the drone in [m]
        """
        global_dNorth = dNorth * math.cos(yaw) + dEast * math.cos(yaw + math.pi / 2)
        global_dEast = dNorth * math.sin(yaw) + dEast * math.sin(yaw + math.pi / 2)
        global_dDown = dDown
        return (global_dNorth, global_dEast, global_dDown)

    def distance_covered_along_track(self, org_yaw):
        """Compute the distance of the drone covered along the straight path
        starting from the home_location with the orignal heading.

        Args:
            org_yaw: Yaw angle of the drone at the home_location[deg]
        Returns:
            covered_distance: Computed covered distance along the track[m]
        """
        current_yaw = self.vehicle.attitude.yaw
        covered_distance = (math.sqrt(self.vehicle.location.local_frame.north**2 + self.vehicle.location.local_frame.east**2)) \
                           * math.cos(abs(current_yaw - org_yaw))
        return covered_distance
    # Movement functions

    def arm_and_takeoff(self, aTargetAltitude):
        """Arm and takeoff.
        It actuates the drone to do pre-arm checks, arm the motors and take off to the set altitude.

        Args:
            aTargetAltitude: Target altitude to take off in [m]
        Returns:
            nothing
        """
        print("Basic pre-arm checks")
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(self.sleep_time)

        print("Arming motors")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(self.sleep_time)

        print("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude*0.95:
                print("Reached target altitude")
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

    def set_position_target_local_NED(self, dNorth, dEast, dDown, frame):
        """Actuation method for a local NED target.
        It actuates the drone to fly to a local NED location.

        Args:
            dNorth: Position in terms of North(Front) to the drone [m]
            dEast: Position in terms of East(Right) to the drone [m]
            dDown: Position in terms of Down to the drone [m]
            frame: frame to use: mavutil.mavlink.MAV_FRAME_LOCAL_NED - relative to home position, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED relative to current position
                   mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED - relative to current position with heading, details in http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
        Returns:
            nothing
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            # frame: positions relative to the current vehicle position and heading
            frame,
            0b0000111111111000,  # type_mask (only positions enabled)
            # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
            dNorth, dEast, dDown,
            0, 0, 0,  # x, y, z velocity in m/s  (not used)
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def set_velocity_local_NED(self, velocity_x, velocity_y, velocity_z, duration, frame=mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED):
        """Actuation method for local NED velocities.
        It actuates the drone to fly to with local NED defined velocities.

        Args:
            velocity_x: Linear velocity of the drone in x(to the front) [m/s]
            velocity_y: Linear velocity of the drone in y(to the right) [m/s]
            velocity_z: Linear velocity of the drone in z(down) [m/s]
            duration: Duration for maintaining the velocities [s]
            frame: frame to use: mavutil.mavlink.MAV_FRAME_LOCAL_NED - relative to home position, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED relative to current position
                   mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED - relative to current position with heading, details in http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
        Returns:
            nothing
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            # frame: velocities relative to the current vehicle position and heading
            frame,
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle on 1 Hz cycle
        for x in range(0, duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)

    def goto_global_NED(self, dNorth, dEast, dDown, func=None):
        """Actuation method for a global NED target.
        It actuates the drone to fly to a global NED location(global North East and Down to the drone regardless of the heading).

        TODO: debug: when using set_position_target_local_NED, dEast is not performed
        Args:
            dNorth: Target position in terms of North to the drone [m]
            dEast: Target position in terms of East to the drone [m]
            dDown: Target position in terms of Down to the drone [m]
            func: Flag for which MAVLink command to use, None - SET_POSITION_TARGET_GLOBAL_INT or 'mav' - MAV_CMD_NAV_WAYPOINT
        Returns:
            nothing
        """
        if func is None:
            startLocation = self.vehicle.location.local_frame
            targetLocation = LocationLocal(startLocation.north + dNorth, startLocation.east + dEast, startLocation.down + dDown)
            targetDistance = self.get_distance_metres_EKF(startLocation, targetLocation)
            org_yaw = self.vehicle.attitude.yaw
            self.set_position_target_local_NED(dNorth, dEast, dDown, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED)
        elif func == 'mav':
            startLocation = self.vehicle.location.global_relative_frame
            targetLocation = self.global_NED_to_wp(startLocation, dNorth, dEast, dDown)
            targetDistance = self.get_distance_metres(startLocation, targetLocation)
            self.goto_wp_global(targetLocation, func)
        print("StartLocation: {}, {}, {}".format(startLocation.north, startLocation.east, startLocation.down))
        print("TargetLocation: {}, {}, {}".format(targetLocation.north, targetLocation.east, targetLocation.down))
        while self.vehicle.mode.name == "GUIDED":  # Stop action if we are no longer in guided mode.
            currentLocation = self.vehicle.location.local_frame
            #remainingDistance = self.get_distance_metres_EKF(currentLocation, targetLocation)
            if func is None:
                remainingDistance = targetDistance - self.get_distance_metres_EKF(startLocation, currentLocation) * math.cos(abs(self.vehicle.attitude.yaw - org_yaw))
            elif func == 'mav':
                remainingDistance = self.get_distance_metres(currentLocation, targetLocation)
            print("Distance to target: {}".format(remainingDistance))
            if remainingDistance <= self.distance_threshold:
                print("Reached target")
                break
            time.sleep(self.sleep_time)


    def goto_local_NED(self, dNorth, dEast, dDown, frame):
        """Actuation method for a local NED target.
        It actuates the drone to fly to a local NED location (NED w.r.t. heading of the drone).
        TODO: debug: when using set_position_target_local_NED, dEast is not performed
        Args:
            dNorth: Target position in terms of North(Front) to the drone [m]
            dEast: Target position in terms of East(Right) to the drone [m]
            dDown: Target position in terms of Down to the drone [m]
            frame: frame to use: mavutil.mavlink.MAV_FRAME_LOCAL_NED - relative to home position, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED relative to current position
                   mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED - relative to current position with heading, details in http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
        Returns:
            nothing
        """
        if (frame == mavutil.mavlink.MAV_FRAME_LOCAL_NED or mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED):
            startLocation = self.vehicle.location.local_frame
            print("StartLocation: {}, {}, {}".format(startLocation.north, startLocation.east, startLocation.down))
            org_yaw = self.vehicle.attitude.yaw
            global_NED = self.local_NED_to_global_NED(dNorth, dEast, dDown, org_yaw)
            targetOffset = LocationLocal(dNorth, dEast, dDown)
            #targetLocation = LocationLocal(startLocation.north + global_NED[0], startLocation.east + global_NED[1], startLocation.down + global_NED[2])
            #targetLocation = LocationLocal(startLocation.north + targetOffset.north, startLocation.east + targetOffset.east, startLocation.down + targetOffset.down)
            print('targetOffset: {}, {}, {}'.format(targetOffset.north, targetOffset.east, targetOffset.down))
            # targetDistance = self.get_distance_metres_EKF(startLocation, targetLocation)
            distanceVector = LocationLocal(targetOffset.north - (startLocation.north - self.local_home.north), targetOffset.east - (startLocation.east - self.local_home.east), targetOffset.down - (startLocation.down - self.local_home.down))
            targetDistance = math.sqrt(distanceVector.north**2 + distanceVector.east**2 + distanceVector.down**2)
            print("Distance to fly: {}".format(targetDistance))
            self.set_position_target_local_NED(targetOffset.north, targetOffset.east, targetOffset.down, frame)

        elif (frame == mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT):
            startLocation = self.vehicle.location.global_relative_frame
            org_yaw = self.vehicle.attitude.yaw
            global_NED = self.local_NED_to_global_NED(dNorth, dEast, dDown, org_yaw)
            targetLocation = self.global_NED_to_wp(startLocation, global_NED[0], global_NED[1], global_NED[2])
            targetDistance = self.get_distance_metres(startLocation, targetLocation)
            self.set_position_target_global_int(targetLocation)



        while self.vehicle.mode.name == "GUIDED":
            currentLocation = self.vehicle.location.local_frame
            # remainingDistance = self.get_distance_metres(self.vehicle.location.global_relative_frame, targetLocation)
            # remainingDistance is the distance covered along the straight path from the startLocation of this navigation.
            remainingDistance = targetDistance - self.get_distance_metres_EKF(startLocation, currentLocation)# * math.cos(abs(self.vehicle.attitude.yaw - org_yaw))
            #remainingDistance = self.get_distance_metres_EKF(currentLocation, targetOffset)
            print("Distance to target: {}".format(remainingDistance))
            print("Current location: {}, {}, {}".format(currentLocation.north, currentLocation.east, currentLocation.down))
            if remainingDistance <= self.distance_threshold:
                print("Reached target")
                break
            time.sleep(self.sleep_time)

    def goto_wp_global(self, targetLocation, func=None):
        """Actuation method for global waypoint.
        It actuates the drone to fly to a global waypoint.
        TODO: the estimation between the current location and the target location is still hacky

        Args:
            targetLocation: Target waypoint defined in LocationGlobalRelative
            func: Flag for which MAVLink command to use, None - SET_POSITION_TARGET_GLOBAL_INT or 'mav' - MAV_CMD_NAV_WAYPOINT
        Returns:
            nothing
        """
        startLocation = self.vehicle.location.global_relative_frame
        targetDistance = self.get_distance_metres(startLocation, targetLocation)
        if func is None:
            self.set_position_target_global_int(targetLocation)
        elif func == 'mav':
            self.vehicle.simple_goto(targetLocation)
        else:
            pass

        while self.vehicle.mode.name == "GUIDED":  # Stop action if we are no longer in guided mode.
            currentLocation = self.vehicle.location.global_relative_frame
            print('Approaching target waypoint...')
            print(currentLocation)
            if (abs(targetLocation.lat - currentLocation.lat) * 1e7 <= self.coordinate_threshold) and (abs(targetLocation.lon - currentLocation.lon) * 1e7 <= self.coordinate_threshold):
                print("Reached target")
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
