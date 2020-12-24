#!/usr/bin/python3
import numpy
import pandas
import matplotlib.pyplot as plot

x = numpy.arange(9).reshape(3, 3)

print(x.shape)

data = pandas.read_table("../data/data.txt", sep="\t")
# data = data.values()
data = data.as_matrix()

print("shape = ", data.shape)
actual_position = data[:, 0]
target_position = data[:, 1]
actual_torque = data[:, 2]
time = data[:, 3]


lw = 3


fig = plot.figure()

ax_left = fig.add_subplot(111)

# plot.subplot(2, 1, 1)
ax_left.plot(time, actual_position, linewidth=3, color="red")
# plot.ylim([250000, 350000])
# plot.grid(True)
# plot.xlim([2250, 2750])

# plot.subplot(2, 1, 1)
ax_left.plot(time, target_position, linewidth=1, color="blue")
# ax_left.grid()
# plot.ylim([250000, 350000])
# plot.xlim([2250, 2750])

# plot.subplot(2, 1, 2)

ax_right = ax_left.twinx()
ax_right.plot(time, actual_torque, linewidth=1, color="black")
# plot.ylim([-2500, 2500])
# plot.xlim([3250, 3750])
ax_left.grid()

plot.show()

# print()

# # print(x1_ref)

# plot.show()

# print(col[0])





