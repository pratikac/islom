import sys
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

i = 0
xAccel = []
yAccel = []
gRate = []

f = open('rewac_data1.txt', 'r');

s = f.readline()
print s
