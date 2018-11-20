cmake_minimum_required(VERSION 3.12)
project(Cyclone_dronekit)

set(CMAKE_CXX_STANDARD 14)

add_executable(Cyclone_dronekit
        "Command Scripts/configs.py"
        "Command Scripts/cyclone.py"
        "Command Scripts/mimic_auto.py"
        "Command Scripts/onehoop.py"
        "Command Scripts/tmp_test.py"
        "Command Scripts/video2.mp4"
        "Python-C++ interface/marker/Detect_Markers.cpp")
