#!/usr/bin/python

import sys
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

index = []
vel_profile = []
vel = []
command = []

FREQ = 10						# 50 Hz
dt = 1/FREQ
ENC_COUNT = 304					# 19x16
ENC2VEL = 1.0/4/ENC_COUNT*360		# result in deg/s

f = open('motor2.txt', 'r');

count = 0;
if f:
	line = f.readline()
	if line == "Start\n":
		line = f.readline()
		while line != "Done\n":
			s = line.split('\t')
			index.append(count)
			vel_profile.append(float(s[0])*ENC2VEL)
			vel.append(float(s[1])*ENC2VEL)
			command.append(float(s[2]))
			
			line = f.readline()
			count = count + 1
f.close()
print "Read ",len(index), " lines."

fig = plt.figure(1)
ax = fig.add_subplot(111)
plt.plot(index, vel_profile, 'r-', label='Velocity command')
plt.plot(index, vel, 'b-', label='Velocity')
plt.legend(loc="best")
plt.xlabel('Time (s) x 50')
plt.ylabel('Angular Velocity (deg/s)')

plt.grid()
plt.show()

