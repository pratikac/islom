#!/usr/bin/python

import sys
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

#Hopper params
M = 2                       #platform mass
m = 0.4                     #leg mass
g = 9.8                     #gravity
l0 = 0.3                    #spring relaxed length
H = 0.8                     #dropping height
h = H                       #height after nth hop       
k = 300.0                   #spring constant
r = 0.02                    #radius of pulley
T = 0.0                     #motor torque
t_wound = 0.2               #time to wound the spring
t_acc = t_wound/6
w_max = 0.0                 #max. omega of wound motor

#Motor model - Faulhabeur 2232
#Km = 32.1/1000              #Torque constant            
#Ke = 3.36/1000*60/2/np.pi  #Omega constant
#Ra = 16.4                    #Winding resistance
#eta = 14                    #Gear ratio
#V = 24                      #voltage

#Motor model - Faulhabeur 2237 
#Km = 31.8/1000              #Torque constant            
#Ke = 3.33/1000*60/2/np.pi  #Omega constant
#Ra = 15.7                    #Winding resistance
#eta = 14                    #Gear ratio
#V = 24                      #voltage

#Motor model - Faulhabeur 2342
Km = 26.1/1000              #Torque constant            
Ke = 2.73/1000*60/2/np.pi  #Omega constant
Ra = 7.1                    #Winding resistance
eta = 43                    #Gear ratio
V = 24                      #voltage

# Design of machine elements, pg. no 514
# Worm - worm-wheel-rack
# wheel torque = M2, input torque = M1
# M2 = (M1 d2/ d1)* (cos a_n - mu tan y)/ (cos a_n * tan y + mu)
# a_n = pressure angle = 20 deg
# y = worm lead angle  = ? (shud be less than 25 deg for a_n = 20
# mu = co-eff of fric
# d2/d1 = ratio of diameters
# M2 = kx * (d2/2)

# worm elements
r1 = 0.005
r2 = 0.01
a_n = 20.0*np.pi/180
helix_angle = 25.0*np.pi/180
mu = 0.3

#plot mass, extension, max. torque, highest omega
x_array = []
Treq_array = []
Tavail_array = []
m_array= []
r_array= []
w_array = []
P_array=[]

for i in range(100):
    m = 0.3 + 0.008*i
    Ewaste = M*g*(H-l0)/(1+ M/m)
    x = np.sqrt(Ewaste*2/k)

# For Design 1
    #T = r*(k*x - M*g)
    #w_max = x/r/(t_wound - 2*t_acc + t_acc)                     #trapezoidal omega vs time
    
# For Design 2
    v_max = x/(t_wound - 2*t_acc + t_acc)                       #trapezoidal velocity vs time
    w2_max = v_max/r2    
    w1_max = np.tan(helix_angle)*w2_max*r1/r2
    
    T = k*x*r1/(np.cos(a_n) - mu*np.tan(helix_angle))*(np.cos(a_n)*np.tan(helix_angle) + mu)

    I = (V - Ke*w1_max*eta)/Ra                                   #neglect inductance, find current
    Tavail = eta*Km*I
    #print "I: ", I, "P_op: ", T*w1_max, "P_waste: ", I*I*Ra

    x_array.append(1000*x)
    w_array.append(w1_max/2/np.pi*60)
    m_array.append(m)
    r_array.append(1000*r)
    Treq_array.append(1000*T)
    Tavail_array.append(1000*Tavail)
    P_array.append(T*w1_max*100)

plt.figure(1)
plt.plot(m_array, x_array, 'b-', label='Extension, mm')
plt.plot(m_array, Treq_array, 'g-', label='Torque req., mNm')
plt.plot(m_array, w_array, 'r-', label='Omega [RPM]')
plt.plot(m_array, P_array, 'y-', label='Power req x100, W')
#plt.plot(m_array, Tavail_array, 'k-', label='Torque avail., mNm')
plt.xlabel('m (kg)')
plt.grid()
plt.legend(loc="best")
plt.title('H=0.8, M=2, l0=0.3, k=300, t=0.2')
plt.show()

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

