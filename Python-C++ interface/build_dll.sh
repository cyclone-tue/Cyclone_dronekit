g++ -std=c++11 -o libcpp_py_vision.so -shared -fPIC src/main.cpp `pkg-config --cflags --libs opencv`

