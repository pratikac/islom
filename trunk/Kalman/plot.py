#!/usr/bin/python

import sys
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

index = []
gRate = []
accelAngle = []
kalmanAngle = []

f = open('data1.dat', 'r');

if f:
	line = f.readline()
	if line == "Start\n":
		line = f.readline()
		while line != "Done\n":
			s = line.split(',')
			index.append(s[0])
			gRate.append(s[1])
			accelAngle.append(s[2])
			kalmanAngle.append(s[3])
			line = f.readline()

plt.plot(index, gRate, 'r-')
plt.plot(index, accelAngle, 'b-')
plt.plot(index, kalmanAngle, 'g-')

plt.grid()
plt.show()

