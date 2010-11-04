// Kalman filter variables
#define GYRO_SCALE              (0.07326)
#define ACCEL_SCALE             (0.0004625)
#define gravity                 (9.806)
#define FSAMP                   (50)
#define RAD2DEG                 (180.0/3.1415)

// 10.6
volatile INT16 GYRO_MULT = GYRO_SCALE*64.0;
volatile INT16 TILT_MULT = 0.1*64.0;

// Persistant states (10.6) -- covaraince is < 1
volatile INT16 P_00 = 64.0;
volatile INT16 P_01 = 0;
volatile INT16 P_10 = 0;
volatile INT16 P_11 = 64.0;

// Constants (10.6)
volatile INT16 A_01 = 64.0/FSAMP;
volatile INT16 B_00 = 64.0/FSAMP;

// Accelerometer variance (10.6) = 22*ACCEL_SCALE*g
volatile INT16 Sz = 22*ACCEL_SCALE*gravity*64.0;

// Gyro variance (10.6) = 96E-6
volatile INT16 Sw_00 = 1;

// Output (10.6)
volatile INT16 x_00 = 0;
volatile INT16 x_10= 0;

// Filter vars (all 10.6)
volatile INT16 inn_00 = 0, s_00 = 0, AP_00 = 0, AP_01 = 0, AP_10 = 0, AP_11 = 0, K_00 = 0, K_10 = 0;
volatile INT16 to_send[7] = {0};

void do_kalman(INT16 gRead, INT16 tilt)
{
    // Update the state estimate by extrapolating current state estimate with input u.
    // x = A * x + B * u
    x_00 = x_00 + ((INT32)((INT32)A_01 * (INT32)x_10))/64 + ((INT32)((INT32)B_00 * (INT32)gRead))/64;
    to_send[0] = x_00;

    // Compute the innovation -- error between measured value and state.
    // inn = y - c * x
    inn_00 = tilt - x_00;
    to_send[1] = inn_00;    

    // Compute the covariance of the innovation.
    // s = C * P * C' + Sz
    s_00 = P_00 + Sz;
    to_send[2] = s_00;

    // Compute AP matrix for use below.
    // AP = A * P
    AP_00 = P_00 + ((INT32)((INT32)A_01 * (INT32)P_10))/64;
    AP_01 = P_01 + ((INT32)((INT32)A_01 * (INT32)P_11))/64;
    AP_10 = P_10;
    AP_11 = P_11;
    to_send[3] = AP_00;
    
    // Compute the kalman gain matrix.
    // K = A * P * C' * inv(s)
    K_00 = ((INT32)((INT32)AP_00*64)) / s_00;
    K_10 = ((INT32)((INT32)AP_10*64)) / s_00;
    to_send[4] = K_00;

    // Update the state estimate
    // x = x + K * inn
    x_00 = x_00 + ((INT32)((INT32)K_00 * (INT32)inn_00))/64;
    x_10 = x_10 + ((INT32)((INT32)K_10 * (INT32)inn_00))/64;
    to_send[5] = x_00;

    // Compute the new covariance of the estimation error
    // P = A * P * A' - K * C * P * A' + Sw
    P_00 = AP_00 + ((INT32)((INT32)AP_01 * (INT32)A_01) - (INT32)((INT32)K_00 * (INT32)P_00))/64 + (INT32)(((INT32)((INT32)K_00 * (INT32)P_01)/64) * (INT32)A_01)/64 + Sw_00;
    P_01 = AP_01 - (INT32)((INT32)K_00 * (INT32)P_01)/64;
    P_10 = AP_10 + ((INT32)((INT32)AP_11 * (INT32)A_01) - (INT32)((INT32)K_10 * (INT32)P_00))/64 + (INT32)(((INT32)((INT32)K_10 * (INT32)P_01)/64) * (INT32)A_01)/64;
    P_11 = AP_11 - (INT32)((INT32)K_10 * (INT32)P_01)/64;
}

