#!/usr/bin/python

import sys
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

index = []
gRate = []
accelX = []
accelY = []
accelAngle = []
gyroAngle = []
kalmanAngle = []

GYRO_SCALE = 0.07326            # deg/s/LSB
ACCEL_SCALE = 0.0004625         # g/LSB
g = 9.80665
FREQ = 10                       # 50 Hz
dt = 1/FREQ
RAD2DEG = 180/np.pi

f = open('data6.txt', 'r');

prev_gyroAngle = 0.0
count = 0;
if f:
    line = f.readline()
    if line == "Start\n":
        line = f.readline()
        while line != "Done\n":
            s = line.split('\t')
            index.append(count)
            accelX.append(float(s[0])*ACCEL_SCALE*g)
            accelY.append(float(s[1])*ACCEL_SCALE*g)
            gRate.append(float(s[2])*GYRO_SCALE)
            
            accelAngle.append( np.arctan2(float(s[0]), float(s[1]))*RAD2DEG)
            prev_gyroAngle += dt*float(s[2])*GYRO_SCALE
            gyroAngle.append(prev_gyroAngle)
            line = f.readline()
            count = count + 1
f.close()
print "Read ",len(index), " lines."

# Persistant states.
P_00 = 1
P_01 = 0
P_10 = 0
P_11 = 1

# Constants.  
A_01 = dt;
B_00 = dt;

# Accelerometer variance
Sz = 22*ACCEL_SCALE*g;

# Gyro covariance ??
#Sw_00 = 0.001;
#Sw_01 = 0.003;
#Sw_10 = 0.003;
#Sw_11 = 0.003;

Sw_00 = 92E-6;
Sw_01 = 0;
Sw_10 = 0;
Sw_11 = 0;

# Output.
x_00 = 0.0
x_10 = 0.0

# Update the State Estimation and compute the Kalman Gain.
# The estimated angle is returned.
def kalman_update(gyro_rate, accel_angle):
    
    global x_00, x_10
    global P_00, P_01, P_10, P_11
    # Inputs
    u = gyro_rate
    y = accel_angle

    # temporary
    s_00 = 0
    inn_00 = 0
    K_00 = 0
    K_10 = 0
    AP_00 = 0
    AP_01 = 0
    AP_10 = 0
    AP_11 = 0
    APAT_00 = 0
    APAT_01 = 0
    APAT_10 = 0
    APAT_11 = 0
    KCPAT_00 = 0
    KCPAT_01 = 0
    KCPAT_10 = 0
    KCPAT_11 = 0

    # Update the state estimate by extrapolating current state estimate with input u.
    # x = A * x + B * u
    x_00 = x_00 + (A_01 * x_10) + (B_00 * u)

    # Compute the innovation -- error between measured value and state.
    # inn = y - c * x
    inn_00 = y - x_00

    # Compute the covariance of the innovation.
    # s = C * P * C' + Sz
    s_00 = P_00 + Sz

    # Compute AP matrix for use below.
    # AP = A * P
    AP_00 = P_00 + A_01 * P_10
    AP_01 = P_01 + A_01 * P_11
    AP_10 = P_10
    AP_11 = P_11

    # Compute the kalman gain matrix.
    # K = A * P * C' * inv(s)
    K_00 = AP_00 / s_00
    K_10 = AP_10 / s_00

    # Update the state estimate.
    # x = x + K * inn
    x_00 = x_00 + K_00 * inn_00
    x_10 = x_10 + K_10 * inn_00

    # Compute the new covariance of the estimation error.
    # P = A * P * A' - K * C * P * A' + Sw
    APAT_00 = AP_00 + (AP_01 * A_01)
    APAT_01 = AP_01
    APAT_10 = AP_10 + (AP_11 * A_01)
    APAT_11 = AP_11
    KCPAT_00 = (K_00 * P_00) + (K_00 * P_01) * A_01
    KCPAT_01 = (K_00 * P_01)
    KCPAT_10 = (K_10 * P_00) + (K_10 * P_01) * A_01
    KCPAT_11 = (K_10 * P_01)
    P_00 = APAT_00 - KCPAT_00 + Sw_00
    P_01 = APAT_01 - KCPAT_01 + Sw_01
    P_10 = APAT_10 - KCPAT_10 + Sw_10
    P_11 = APAT_11 - KCPAT_11 + Sw_11



c = 0
while c < len(index) :
    kalman_update(gRate[c], accelAngle[c])
    kalmanAngle.append(x_00)
    c = c + 1

s = 0
t = len(index)
fig = plt.figure(1)
ax = fig.add_subplot(111)
#plt.plot(index, accelX, 'r-')
#plt.plot(index, accelY, 'b-')
plt.plot(index[s:t], accelAngle[s:t], 'r-', label='Accel angle')
plt.plot(index[s:t], gRate[s:t], 'b-', label='Gyro rate')
plt.plot(index[s:t], kalmanAngle[s:t], 'g-', label='Kalman angle')
#plt.plot(index, gyroAngle, 'r-', label='Gyro angle')
plt.legend(loc="best")
ax.set_ylim((-360,360))
plt.xlabel("Time [s x50]")
plt.ylabel("Angle [deg]")

plt.grid()
plt.savefig("data5.pdf")
plt.show()

