#!/usr/bin/python

from numpy import *
from pylab import *

#For the new design 
#Hopper params
M = 2						# platform mass
m = 0.5						# leg mass
g = 9.8						# gravity
l0 = 0.3					# spring relaxed length
H = 0.8 					# dropping height
k = 300.0					# spring constant
L = 0.8                     # length of the leg
h_m_cg = 0.5                # cg height of the leg
h_M_cg = L-l0               # It hangs from L till l0
stop_dis = l0 - m*g/k

v = 0.5						# Horizantal velocity of cg

# Rewac variables
DEG2RAD = pi/180
RAD2DEG = 180/pi

init_pitch = 45*DEG2RAD
rho_steel = 7850
dist_wheel = 0.12
r_wheel = 0.08
m_wheel = pi*2*r_wheel*pi*0.01*0.01*rho_steel			
d_cg = m_wheel*dist_wheel/(m+m_wheel+1)


T_min_reorient = 0			# Keep it around 50-60% of T_air	


#Motor model - Faulhabeur 2342CR024
Km = 26.1/1000				#Torque constant			
Ke = 2.730/1000*60/2/np.pi  #Omega constant
Ra = 7.1					#Winding resistance
eta = 3.71					#Gear ratio
V = 24						#voltage

plotx_array = []
ploty1_array = []
ploty2_array = []
ploty3_array = []
ploty4_array = []
ploty5_array = []
ploty6_array = []

# dist_wheel = 0.05
r_wheel = 0.04
count = 0
while count<100:
	count += 1
    # dist_wheel += 0.002
	r_wheel += 0.001
	plotx_array.append(r_wheel)
	
	# Stupid vars, recalculate
	m_wheel = pi*2*r_wheel*pi*0.01*0.01*rho_steel			
	d_cg = m_wheel*dist_wheel/(m+m_wheel+1)
	M = m_wheel + 1.0
	w = sqrt(k/M)
	
	Ewaste = M*g*(H-l0)/(1+ M/m)
	pre_extension = np.sqrt(Ewaste*2/k)
	h_M_cg = L - l0 - pre_extension
	
	h_cg = (h_m_cg*m + h_M_cg*M)/(m+M)	# Height of CG from ground

	J_wheel = m_wheel*r_wheel*r_wheel
	# Neglect the contribution to J_body by the reaction wheel for now
	J_body = M*(pow((h_cg - h_M_cg),2) + d_cg*d_cg) + m*(pow((h_cg - h_m_cg),2) + d_cg*d_cg)

	x1 = 0                      # height of M at full compression (during stance)

	# Conserve energy at max. height and max. extension of spring
	a1 = 0.5*k
	b1 = -M*g
	c1 = h_m_cg*m*g + M*g*(L-l0) - (m+M)*g*H

	x1 = (-b1 + sqrt(b1*b1 - 4*a1*c1) )/2/a1
	# print x1
	# print "Leg mechanism max. height: ", L-l0-x1

	# C.G. falls from H till x2 (c.g. just before impact)
	x2 = h_cg
	t1 = sqrt(2* (H - x2)/g)				# time of fall to this point
	v_cg_t1 = sqrt(2*g*(H-x2))				# assume both M, m fall by same height => v_cg = v_m = v_M
	# print "t1: ", t1

	# SHM after t1 as follows
	# Initial vel = v_cg_t1
	# Initial displacement = 0
	# Take the solution with -ve v
	# x = A sin(wt)
	A = v_cg_t1/w
	t2 = arcsin((stop_dis - l0)/A)/w + 2*pi/w/2

	# v_M_t2 = abs(A*w*cos(w*t2))
	# v_cg_t2 = v_M_t2*M/(m+M)
	# print "v_cg_t2: ", v_cg_t2
	x_cg_t2 = (h_m_cg*m + M*(L - stop_dis) )/(m+M)

	v_cg_t2 = sqrt(2*g*(H - x_cg_t2))			# After adding the impulse velocity

	# go to height H from this height in time t3
	# g/2*t^2 -ut + H(x_cg_t2) = 0
	# a1 = 0.5*g
	# b1 = -v_cg_t2
	# c1 = (H - x_cg_t2)
	# t3 = (-b1 - sqrt(b1*b1 - 4*a1*c1) )/2/a1
	t3 = v_cg_t2/g
	# print "t3: ", t3

	# t2 is the stance time
	# t1+t3 is the time available to correct orientation
	T_air = t1+t3
	pitch = init_pitch
	pitch_lf = pitch - v*t2/h_cg			# pitch at lift-off
	# print "pitch_lf: ", pitch_lf

	# omega_air = I*moment_arm/J_body, this operates for T_max
	# Pain here, do we talk about the time variation of the impact force for the stance period??
	theta_dot = m*v_cg_t1*((h_cg-pre_extension)*sin(pitch) + d_cg*cos(pitch))/J_body
	# print "theta_dot: ", theta_dot
	
	# Need to go to init_pitch from pitch_lf with this theta_dot (and omega by rewac)
	# Also, need a small omega_reorient to avoid problem with coriolis
	T_min_reorient = 0.3*T_air
	omega_reorient = (init_pitch - pitch_lf - 0.5*theta_dot*T_min_reorient*T_min_reorient)/T_min_reorient
	omega_wheel = -(omega_reorient*(J_body+J_wheel))/J_wheel
	I = (V - Ke*absolute(omega_wheel)*eta)/Ra
	P = V*I										# Worst case power
	T = Km*eta*I								# Available torque
	omega_dot = omega_wheel/(T_air*0.2)
	T_needed = J_wheel*abs(omega_dot)

	ploty4_array.append(T*1000 - T_needed*1000)
	ploty1_array.append(omega_wheel*60/2/pi)
	ploty2_array.append(P*100)
	ploty3_array.append(theta_dot)
	ploty6_array.append(m_wheel*10000)

print "Done"

figure(1)
plot(plotx_array, ploty1_array, 'b-', label='Omega_wheel (RPM)')
plot(plotx_array, ploty4_array, 'g-', label='Torque (avail - needed)(mN.m)')
# plot(plotx_array, ploty5_array, 'k-', label='T_max needed (mN.m)')
# plot(plotx_array, ploty2_array, 'y-', label='Max. Power (W*100)')
# plot(plotx_array, ploty6_array, 'm-', label='m_wheel (gm*10)')
title("Variation with wheel radius (dist_cg = 12 cm)(m)")
legend(loc="best")
grid()

show()










