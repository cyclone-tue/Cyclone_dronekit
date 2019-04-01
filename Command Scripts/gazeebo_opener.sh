#!/bin/bash
# gazeebo opener

gnome-terminal --command="bash -c 'python onehoop.py --connect=127.0.0.1:14551 --simulate=True; $SHELL'"
gnome-terminal --command="bash -c 'gazebo --verbose worlds/iris_arducopter_runway.world; $SHELL'"

sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console
mode guided
arm throttle