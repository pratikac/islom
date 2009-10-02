#!/usr/bin/env python

import sys
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from time import sleep
import numpy as np

x = []
y1 = []
y2 = []
y3 = []
dx = 0.1
x.append(0)
y1.append(0)
y2.append(1)
y3.append(1)
count = 0
MAX = 100

def animate():
	global count
	while count < 1000:
		count = count + 1
		curr = len(x) -1
		t1 = x[curr] + dx
		t2 = np.sin(t1)
		t3 = np.cos(t1)
		x.append(t1)
		y1.append(t2)
		y2.append(t3)
		y3.append(t2+t3)

		if (count %10) == 0:
			if curr > MAX:
				plot_from = curr - MAX
			if curr <= MAX:
				plot_from = 0
			print plot_from	
			plt.plot(x[plot_from:],y1[plot_from:],'r-')
			plt.plot(x[plot_from:],y2[plot_from:],'b-')
			plt.plot(x[plot_from:],y3[plot_from:],'g-')
			f.canvas.draw()
		#sleep(0.01)
	return
	
f = plt.figure()
plt.ioff()
plt.grid()
win = f.canvas.manager.window
f.canvas.manager.window.after(1000, animate)
plt.show()
