# Cyclone_dronekit
Welcome to the cyclone software architecture. This file describes the install process for this repository and how to run it one the drone.


## Cloning the repository
Cloning the repository requires a couple of steps.
* Choose a directroy as to clone the repository into and open a command line in this location.
* Run `git clone --recursive https://github.com/cyclone-tue/Cyclone_dronekit.git`
* A new directory Cyclone_dronekit will be created. This directory contains all the code from the repository.

## Installing vision dependencies
This step is only necessary if you want to use the vision capabilities of the drone. Otherwise you can skip this step.
* First install the dependencies for the vision code:
  * [OpenCV](https://opencv.org/)
  * [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
* Install the H264 codec by running the following commands
  * `sudo apt install libdvdnav4 libdvdread4 gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly libdvd-pkg`
  * `sudo apt install ubuntu-restricted-extras` 
* Afterwards you should be able to open the Python-C++ interface folder in a terminal and run `./build_lib.sh`. This should output a shared object file into the this directory.

## Running the python script
This step involves running a command script either in local simulation or on the drone.
* Open the Command Scripts folder in a terminal.
* Run `python <script>.py` where <script> is the name of the script you want to run. This runs the command script in a local simulation. Append `--connect 127.0.0.1:14000` to the script to run it on the drone from the jetson.


## Running the script in simulation
* Setup
  * Install Gazebo and ROS and the ardupilot_gazebo plugin following https://github.com/khancyr/ardupilot_gazebo.
  * Install ardupilot following http://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux

* Type `sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console`.
* In a new terminal, type `gazebo --verbose worlds/iris_arducopter_runway.world` in the terminal.

* In a new terminal, type `python <my_script.py> --connect=<gazebo_ip>`, where <gazebo_ip> is the code indicated by `--out` in the gazebo terminal. For example `python onehoop.py --connect=127.0.0.1:14550 --simulate=True`.

* type `mode guided`.
* type `arm throttle`. It may disarm when you wait for a while.
* type `takeoff <height>`.

* press a key and enter in the python scripts terminal to awake the script.