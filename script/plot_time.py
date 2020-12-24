#!/usr/bin/env python3
import numpy
import pandas
import matplotlib.pyplot as plt


time = pandas.read_table("../data/DC_time.csv", sep=",")
plt.figure()

plt.plot(time.diff()[0:None])

plt.show()



