import ctypes
so = ctypes.cdll.LoadLibrary
lib = so("./libcpp_py_vision.so")
print "output_to_py(): \n"
output_to_py_func = lib.output_to_py
output_to_py_func.restype = ctypes.POINTER(ctypes.c_double)
converted = output_to_py_func()
nrow = 100
ncol = 12
LocationTuples = []
VelocityTuples = []



def matrix_index(a, nrow, (m, n)):
    return(a[nrow * n + m])


print("Extracting location and velocity from every point in the trajectory...")

for i in range(nrow):
    LocationTuples.append((matrix_index(converted, nrow, (i, 0)), matrix_index(converted, nrow, (i, 1)), matrix_index(converted, nrow, (i, 2))))
    VelocityTuples.append((matrix_index(converted, nrow, (i, 3)), matrix_index(converted, nrow, (i, 4)), matrix_index(converted, nrow, (i, 5))))
print("Tuples of location (x, y, z) at time t=1, 10, 20 ... 100 are: \n")
for i in range(0, 99, 9):
    print(LocationTuples[i])
print("\n")
print("Tuples of velocity (Vx, Vy, Vz) at time t=1, 10, 20 ... 100 are: \n")
for i in range(0, 99, 9):
    print(VelocityTuples[i])
print("\n")
