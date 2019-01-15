"""
Configurations for the Cyclone class, shall be invoked in the command script as "import configs"
"""
earth_radius = 6378137.0        # Radius of "spherical" earth
meters_per_degree = 1.113195e5  # For converting lat/long degrees to meters
sleep_time = 0.1                # Sleeping time for while loops
distance_threshold = 0.05       # Threshold on distance for detecting whether waypoint is reached [m]
coordinate_threshold = 5        # Threshold on coordinate for detectiong whether waypoint is reached [10^-7 deg]
distance_threshold_percent = 0.5 # Alternative threshold on distance based on percentage of distance to fly [-]
minimal_target_distance = 0.1   # Minimum distance between current location and target location to be considerd as necessary for the path [m]