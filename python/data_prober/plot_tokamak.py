#! /usr/bin/python3

from Metadata import Metadata
from ImageSynchronizer import ImageSynchronizer
import numpy as np
import matplotlib.pyplot as plt

tokamak = Metadata()
tokamak.parse_metadata('tokamak/dataformat.txt','tokamak/tokamak.txt')
x_tokamak = tokamak.get_nparray('x')
y_tokamak = tokamak.get_nparray('y')

fig1 = plt.figure()
plt.plot(x, y, label="robot position")
plt.legend(loc="upper right")
plt.xlabel("East (m)")
plt.ylabel("North (m)")
plt.gca().set_aspect('equal', adjustable='box')

plt.show(block=False)
