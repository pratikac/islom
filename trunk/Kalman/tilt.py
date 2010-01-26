#!/usr/bin/env python

from numpy import *
from pylab import *

tilt = []
gRate = []
index = []
compAngle = []
muCAngle = []

count = 0
f=open("tilt1.txt")
if f:
    line = f.readline()
    if line == "Start\n":
        line = f.readline()
        while line != "Done\n":
            s = line.split('\t')
            index.append(count)
            gRate.append(float(s[0])/256.0)
            tilt.append(float(s[1])/256.0)
            muCAngle.append(float(s[2])/256.0)
            line = f.readline()
            count += 1
f.close()
print "Read ",len(index), " lines."

ACCEL_SCALE = 0.0004625         # g/LSB
GYRO_SCALE = 0.07326            # deg/s/LSB
g = 9.80665
FREQ = 10
dt = 1.0/FREQ
RAD2DEG = 180/np.pi

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
    #print "gRate : ",u," Tilt : ", y

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
    kalman_update(gRate[c], tilt[c])
    compAngle.append(x_00)
    c = c + 1

figure(1)
plot(index, gRate, 'b', label='gRate')
plot(index, tilt, 'r', label='Tilt')
#plot(index, muCAngle, 'g', label='muC Kalman')
plot(index, compAngle, 'y', label='Comp Kalman')
grid()
show()

