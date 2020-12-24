#!/usr/bin/python3
import numpy
import pandas
import matplotlib.pyplot as plt
import math

target_joint_position = pandas.read_table("../data/target_joint_position.csv", sep=",")
actual_joint_position = pandas.read_table("../data/actual_joint_position.csv", sep=",")
target_joint_velocity = pandas.read_table("../data/target_joint_velocity.csv", sep=",")
actual_joint_velocity = pandas.read_table("../data/actual_joint_velocity.csv", sep=",")
target_joint_torque = pandas.read_table("../data/target_joint_torque.csv", sep=",")
actual_joint_torque = pandas.read_table("../data/actual_joint_torque.csv", sep=",")

joints_name = {"RF_HAA": 1, "RF_HFE": 2, "RF_KFE": 3,
               "RH_HAA": 4, "RH_HFE": 5, "RH_KFE": 6,
               "LH_HAA": 7, "LH_HFE": 8, "LH_KFE": 9,
               "LF_HAA": 10, "LF_HFE": 11, "LF_KFE": 12
               }

t = target_joint_position['time']
t_ = actual_joint_position['time']
t__ = target_joint_velocity['time']
t___ = actual_joint_velocity['time']
t____ = target_joint_torque['time']
t_____ = actual_joint_torque['time']
start = None
end = None


class JointData:
    def __init__(self, name):
        self.name = name
        head = joints_name[name]
        self.target_p = target_joint_position[str(head)][start:end] * 180.0 / math.pi
        self.actual_p = actual_joint_position[str(head)][start:end] * 180.0 / math.pi
        self.target_v = target_joint_velocity[str(head)][start:end] * 180.0 / math.pi
        self.actual_v = actual_joint_velocity[str(head)][start:end] * 180.0 / math.pi
        self.target_t = target_joint_torque[str(head)][start:end]
        self.actual_t = actual_joint_torque[str(head)][start:end]

    def plot(self):
        plt.figure(self.name)
        plt.subplot(3, 1, 1)
        plt.plot(t, self.target_p, linewidth=3, color="green")
        plt.plot(t, self.actual_p, linewidth=1.5, color="red")
        plt.legend(["target", "actual"], fontsize=16)
        plt.subplot(3, 1, 2)
        plt.plot(t, self.target_v, linewidth=3, color="green")
        plt.plot(t, self.actual_v, linewidth=1.5, color="red")
        plt.legend(["target", "actual"], fontsize=16)
        plt.subplot(3, 1, 3)
        plt.plot(t, self.target_t, linewidth=3, color="orange")
        plt.plot(t, self.actual_t, linewidth=1.5, color="blue")
        plt.legend(["target", "actual"], fontsize=16)


# target_data = pandas.read_table("../data/target_data.txt", sep="\t")
joints = {}
for name in joints_name:
    joints[name] = JointData(name)

for name in joints:
    # if "LF" in name:
    joints[name].plot()

# print(joints["LF_HFE"].target_p)

# plt.figure("ec_DCtime")
# plt.plot(t[0:100]/1000000, linewidth=3, color="blue")
# # plt.plot(t[0:1000]/1000000, linewidth=3, color="blue")
# # plt.plot(t[0:1000]/1000000, linewidth=3, color="blue")
# # plt.plot(t[0:1000]/1000000, linewidth=3, color="blue")
# # plt.plot(t____[0:1000]/1000000, linewidth=3, color="yellow")
# plt.plot(t_____[0:100]/1000000, linewidth=1, color="red")
# print(t/1000000)
plt.show()




