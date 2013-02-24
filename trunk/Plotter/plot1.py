#!/usr/bin/env python

from matplotlib.pyplot import *
from matplotlib.patches import Circle
from numpy import *

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
MAX = 300

def animate():
    global count
    while count < 10000:
        count = count + 1
        curr = len(x) -1
        t1 = x[curr] + dx
        t2 = sin(t1)
        t3 = cos(t1)
        x.append(t1)
        y1.append(t2)
        y2.append(t3)
        y3.append(t2+t3)

        if (count %5) == 0:
            if curr > MAX:
                plot_from = curr - MAX
            if curr <= MAX:
                plot_from = 0
            print plot_from
            f.clf()
            plot(x[plot_from:],y1[plot_from:],'r-')
            plot(x[plot_from:],y2[plot_from:],'b-')
            plot(x[plot_from:],y3[plot_from:],'g-')
            f.canvas.draw()
    return
    
f = figure()
ioff()
grid()
win = f.canvas.manager.window
f.canvas.manager.window.after(1000, animate)
show()
