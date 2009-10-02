#!/usr/bin/python

import sys
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

#Hopper params
M = 2						#platform mass
m = 0.4						#leg mass
g = 9.8						#gravity
l0 = 0.3					#spring relaxed length
H = 0.6						#dropping height
h = H						#height after nth hop		
k = 300.0					#spring constant
r = 0.02					#radius of pulley
T = 0.0						#motor torque
t_wound = 0.1				#time to wound the spring
w_max = 0.0					#max. omega of wound motor

#Motor model - Faulhabeur 2342CR024
Km = 26.1/1000				#Torque constant			
Ke = 2.730/1000*60/2/np.pi  #Omega constant
Ra = 7.1					#Winding resistance
eta = 3.71					#Gear ratio
V = 24						#voltage
eff = 0.88					#gearbox-efficiency
L = 265E-6					#Inductance
Jrot = 5.8E-7				#Rotor inertia
Jtot = 100E-7				#total inertia
I = 0.0						#armature current


def printall():
    print "m: ", m
    print "w: ", w_max
    print "x: ", x
    print "Treq: ", T
    print "Tavail: ", Tavail
    print "I: ", I

#plot hopping height vs no. of hops

"""
harray1 = []
for i in range(100):
        harray1.append(h)
        xarray.append(x)
        h = (M*h + m*l0)/(M+m)    

plt.figure(1)
plt.plot(range(100), harray1, 'b-')
plt.grid()
plt.show()
"""

#plot mass, extension, max. torque, highest omega
x_array = []
Treq_array = []
Tavail_array = []
m_array= []
r_array= []
w_array = []
for i in range(100):
    r = 0.01 + 7E-4*i
    Ewaste = M*g*(H-l0)/(1+ M/m)
    x = np.sqrt(Ewaste*2/k)
    T = r*(k*x - M*g)
    w_max = 2*x/r/t_wound			#triangular omega vs time
    I = (V - Ke*w_max/eta)/Ra		#neglect inductance, find current
    Tavail = eta*Km*I

    x_array.append(1000*x)
    w_array.append(w_max)
    m_array.append(1000*m)
    r_array.append(1000*r)
    Treq_array.append(1000*T)
    Tavail_array.append(1000*Tavail)
    print I
    #printall()

plt.figure(1)
plt.plot(range(100), r_array, 'r-', label='Radius, mm')
plt.plot(range(100), w_array, 'm-', label='Max Omega, rad/s')
plt.plot(range(100), x_array, 'b-', label='Extension, mm')
plt.plot(range(100), Treq_array, 'g-', label='Torque req., mNm')
plt.plot(range(100), Tavail_array, 'k-', label='Torque avail., mNm')
plt.grid()
plt.legend( ('Radius, mm', 'Max Omega, rad/s', 'Extension, mm', 'Torque req., mNm', 'Torque avail., mNm'), loc="upper left" )
plt.title('H=0.6, M=2, m=0.4, l0=0.3, k=300, t=0.1, FH2342')
plt.show()

