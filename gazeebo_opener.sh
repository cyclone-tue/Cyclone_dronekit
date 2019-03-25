#!/bin/bash
# gazeebo opener

gnome-terminal -e sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console
gnome-terminal -e gazebo --verbose worlds/iris_arducopter_runway.world

python onehoop.py --connect=127.0.0.1:14550 --simulate=True
mode guided
arm throttle