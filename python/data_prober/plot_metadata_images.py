#! /usr/bin/python3

from Metadata import Metadata
from ImageSynchronizer import ImageSynchronizer
import numpy as np
import matplotlib.pyplot as plt

sync = ImageSynchronizer("test/")
sync.intrinsic_synchro()
sync.extrinsic_synchro()
# sync.export_extrinsic_synchronized_index_file("synchronized_cam_indexes.txt")
print("Uncomment line 11 to generate syncronised image indexes file. See ImageSynchronizer for code details.")


# Front cam
frontLeftStamps  = sync.dataFrontLeft.get_nparray('timestamp')
frontRightStamps = sync.dataFrontRight.get_nparray('timestamp')
# Rear cam
rearLeftStamps  = sync.dataRearLeft.get_nparray('timestamp')
rearRightStamps = sync.dataRearRight.get_nparray('timestamp')
# Nav cam
navLeftStamps  = sync.dataNavLeft.get_nparray('timestamp')
navRightStamps = sync.dataNavRight.get_nparray('timestamp')

tokamak = Metadata()
# tokamak.parse_metadata('tokamak/dataformat.txt','tokamak/tokamak.txt')

# time beginning of acquisition
t0 = min([frontLeftStamps[0], rearLeftStamps[1], navLeftStamps[2]])

# Plotting #######################################################

# Ploting image timestamps
fig0, axes = plt.subplots(2,1, sharex=True, sharey=False)
axes[0].plot((frontLeftStamps - t0) / 1000000.0, label="front left stamp")
axes[0].plot((rearLeftStamps - t0)  / 1000000.0, label="rear left stamp")
axes[0].plot((navLeftStamps - t0)   / 1000000.0, label="nav left stamp")
axes[0].legend(loc="upper left")
axes[0].set_xlabel("image index")
axes[0].set_ylabel("time (s)")
axes[0].tick_params(axis='both',which='both')
axes[0].grid()

lMin = min([len(frontLeftStamps), len(rearLeftStamps), len(navLeftStamps)])
axes[1].plot((rearLeftStamps[:lMin] - frontLeftStamps[:lMin]) / 1000000.0, label="(rear - front) left stamp")
axes[1].plot((navLeftStamps[:lMin]  - frontLeftStamps[:lMin]) / 1000000.0, label="(nav - front) left stamp")
axes[1].plot((navLeftStamps[:lMin]  - rearLeftStamps[:lMin] ) / 1000000.0, label="(nav - rear) left stamp")
axes[1].legend(loc="upper left")
axes[1].set_xlabel("image index")
axes[1].set_ylabel("time (s)")
axes[1].tick_params(axis='both',which='both')
axes[1].grid()

# plotting time between succesive frames
fig1, axes = plt.subplots(3,1, sharex=True, sharey=False)
axes[0].plot(, (frontRightStamps - frontLeftStamps) / 1000.0, label="front stamp difference (right-left)")
axes[0].legend(loc="upper right")
axes[0].set_xlabel("left image stamp (s)")
axes[0].set_ylabel("desync time (ms)")
axes[0].tick_params(axis='both',which='both')
axes[0].grid()

axes[1].plot((rearLeftStamps - t0) / 1000000.0, (rearRightStamps  - rearLeftStamps)  / 1000.0,  label="rear stamp difference (right-left)")
axes[1].legend(loc="upper right")
axes[1].set_xlabel("left image stamp (s)")
axes[1].set_ylabel("desync time (ms)")
axes[1].tick_params(axis='both',which='both')
axes[1].grid()

axes[2].plot((navLeftStamps - t0) / 1000000.0, (navRightStamps   - navLeftStamps)   / 1000.0,   label="nav stamp difference (right-left)")
axes[2].legend(loc="upper right")
axes[2].set_xlabel("left image stamp (s)")
axes[2].set_ylabel("desync time (ms)")
axes[2].tick_params(axis='both',which='both')
axes[2].grid()

# ploting time between succesive frames
fig2, axes = plt.subplots(3,1, sharex=True, sharey=False)
axes[0].plot((frontLeftStamps[1:] - t0) / 1000000.0, (frontLeftStamps[1:] - frontLeftStamps[:-1]) / 1000000.0, label="front left time between successive frames")
axes[0].legend(loc="upper right")
axes[0].set_xlabel("left image stamp (s)")
axes[0].set_ylabel("period (s)")
axes[0].tick_params(axis='both',which='both')
axes[0].grid()

axes[1].plot((rearLeftStamps[1:] - t0) / 1000000.0, (rearLeftStamps[1:] - rearLeftStamps[:-1]) / 1000000.0,  label="rear left time between successive frames")
axes[1].legend(loc="upper right")
axes[1].set_xlabel("left image stamp (s)")
axes[1].set_ylabel("period (s)")
axes[1].tick_params(axis='both',which='both')
axes[1].grid()

axes[2].plot((navLeftStamps[1:] - t0) / 1000000.0, (navLeftStamps[1:] - navLeftStamps[:-1]) / 1000000.0,   label="nav left time between successive frames")
axes[2].legend(loc="upper right")
axes[2].set_xlabel("left image stamp (s)")
axes[2].set_ylabel("period (s)")
axes[2].tick_params(axis='both',which='both')
axes[2].grid()

plt.show()
# plt.show(block=False)
