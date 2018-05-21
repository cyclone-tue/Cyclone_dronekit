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
StateTuples = []


def matrix_index(a, nrow, (m, n)):
    return(a[nrow * n + m])


print("Extracting location and velocity from every point in the trajectory...")

for i in range(nrow):
    StateTuples.append((matrix_index(converted, nrow, (i, 0)), matrix_index(converted, nrow, (i, 1)), matrix_index(converted, nrow, (i, 2)), matrix_index(converted, nrow, (i, 3)), matrix_index(converted, nrow, (i, 4)), matrix_index(converted, nrow, (i, 5)),
                        matrix_index(converted, nrow, (i, 6)), matrix_index(converted, nrow, (i, 7)), matrix_index(converted, nrow, (i, 8)), matrix_index(converted, nrow, (i, 9)), matrix_index(converted, nrow, (i, 10)), matrix_index(converted, nrow, (i, 11))))
    LocationTuples.append((matrix_index(converted, nrow, (i, 0)), matrix_index(converted, nrow, (i, 4)), matrix_index(converted, nrow, (i, 8))))
    VelocityTuples.append((matrix_index(converted, nrow, (i, 3)), matrix_index(converted, nrow, (i, 2)), matrix_index(converted, nrow, (i, 5))))
# print("Tuples of location (x, y, z) at time t=1 ... 100 are: \n")
# for i in range(0, nrow, 1):
#     print(LocationTuples[i])
# print("\n")
# print("Tuples of velocity (Vx, Vy, Vz) at time t=1 ... 100 are: \n")
# for i in range(0, nrow, 1):
#     print(VelocityTuples[i])
# print("\n")

with open("trajectory.txt", "w") as text_file:
    # text_file.write("Tuples of location (x, y, z) at time t=1 ... 100 are: \n")
    text_file.write("Tuples of all the states at time t=1 ... 100 are: \n")
    index = 1
    # for row in LocationTuples:
    for row in StateTuples:
        # line = ''.join(str(x) + '  ' for x in row)
        # text_file.write(str(index) + '  ' + line + '\n')
        text_file.write('{0:<4}{1[0]:<20}{1[1]:<4}{1[2]:<4}{1[3]:<4}{1[4]:<20}{1[5]:<10}{1[6]:<10}{1[7]:<10}{1[8]:<20}{1[9]:<20}{1[10]:<20}{1[11]:<20}\n'.format(index, row))
        index = index + 1

