#!/usr/bin/python

import sys
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

#For the new design 
#Hopper params
M = 2						# platform mass
m = 0.2						# leg mass
g = 9.8						# gravity
l0 = 0.3					# spring relaxed length
H = 0.8 					# dropping height
k = 300.0					# spring constant
L = 0.7                     # length of the leg
h_m_cg = 0.5                # cg height of the leg
h_M_cg = L-l0               # It hangs from L till l0
w = np.sqrt(k/M)
stop_dis = l0 - m*g/k

w_hop_array = []
w_nat_array = []
H_array = []
M_array = []
m_array = []

while m < 1:
	m = m + 0.01
	m_array.append(m)
	x1 = 0                      # height of M at full compression (during stance)

	# Conserve energy at max. height and max. extension of spring
	a1 = 0.5*k
	b1 = -M*g
	c1 = h_m_cg*m*g + M*g*(L-l0) - (m+M)*g*H

	x1 = (-b1 + np.sqrt(b1*b1 - 4*a1*c1) )/2/a1
	print x1
	print "Leg mechanism max. height: ", L-l0-x1

    # C.G. falls from H till x2 (c.g. just before impact)
	Ewaste = M*g*(H-l0)/(1 + M/m)
	pre_extension = np.sqrt(Ewaste*2/k)
	
	x2 = (h_m_cg*m + (h_M_cg-pre_extension)*M)/ (m+M)
	t1 = np.sqrt(2* (H - x2)/g)					# time of fall to this point
	v_cg_t1 = np.sqrt(2*g*(H-x2))				# assume both M, m fall by same height => v_cg = v_m = v_M
	print "t1: ", t1

	# SHM after t1 as follows
	# Initial vel = v_cg_t1
	# Initial displacement = 0
	# Take the solution with -ve v
	# x = A sin(wt)
	A = v_cg_t1/w
	t2 = np.arcsin((stop_dis - l0)/A)/w + 2*np.pi/w/2
	print "t2: ", t2

	# v_M_t2 = np.abs(A*w*np.cos(w*t2))
	# v_cg_t2 = v_M_t2*M/(m+M)
	# print "v_cg_t2: ", v_cg_t2
	x_cg_t2 = (h_m_cg*m + M*(L - stop_dis) )/(m+M)

	v_cg_t2 = np.sqrt(2*g*(H - x_cg_t2))			# After adding the impulse velocity

	# go to height H from this height in time t3
	# g/2*t^2 -ut + H(x_cg_t2) = 0
	# a1 = 0.5*g
	# b1 = -v_cg_t2
	# c1 = (H - x_cg_t2)
	# t3 = (-b1 - np.sqrt(b1*b1 - 4*a1*c1) )/2/a1
	t3 = v_cg_t2/g
	print "t3: ", t3

	T = t1 + t2 + t3
	w_hop = 2*np.pi/T
	print "w_hop: ", w_hop
	w_nat = np.sqrt(k/(m*M)*(m+M))
	print "w_nat: ", w_nat

	w_hop_array.append(w_hop)
	w_nat_array.append(w_nat)


plt.figure()
plt.plot(m_array, w_hop_array, 'b-', label='Hopping freq. rad/s')
plt.plot(m_array, w_nat_array, 'r-', label='Natural freq. rad/s')
plt.grid()
plt.legend( ('Hopping freq. rad/s', 'Natural freq. rad/s'), loc="best")
plt.title('Variation with m (kg), H = 0.8 m, M = 2')
plt.show()













