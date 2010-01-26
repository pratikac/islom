#! /usr/bin/env python

from numpy import *
from pylab import *
from scipy import *

DEG2RAD = pi/180.0
RAD2DEG = 180.0/pi

TILL = 70.0
FROM = 15.0
POINTS = 16

x = arange(FROM,TILL,0.001)
y = tan(x*DEG2RAD)

yD = []
DELTA = (tan(TILL*DEG2RAD) - tan(FROM*DEG2RAD))/POINTS
for i in range(POINTS):
    yD.append(tan(FROM*DEG2RAD) + i*DELTA)
xD = arctan(yD)*RAD2DEG

"""
Memory usage : 2x16 + 1x16 = 48 Bytes
"""
def myarctan(v):
    if v == 0:
        return 0
    elif v < yD[0]:
        return v*FROM/yD[0] + 0.06
    else:
        v = v - yD[0]
        t = int(v/DELTA)
        return xD[t] + (v - t*DELTA)/DELTA*(xD[t+1]-xD[t]) + 0.06

xP = arange(0,TILL-1,1)
yP = []
for i in range(len(xP)):
    yP.append(myarctan(tan(xP[i]*DEG2RAD)))

figure(1)
plot(xP, yP-xP)
grid()
show()
