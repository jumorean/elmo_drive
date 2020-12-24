#!/usr/bin/env python3
import numpy
import pandas
import matplotlib.pyplot as plt


grf = pandas.read_table("../data/ground_force.csv", sep=",")
plt.figure()

leg = ["rf", "rh", "lh", "lf"]
# plt.plot(grf["rf"])
# plt.plot(grf["rh"])
# plt.plot(grf["lh"])
plt.plot(grf["time"], grf["lf"])


plt.legend(leg, fontsize=20)
plt.show()
