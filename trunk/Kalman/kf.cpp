#include <iostream>
#include <math.h>
#include <inttypes.h>
#include <fstream>
#include <string>
using namespace std;

ifstream fin;

// Kalman filter variables
#define GYRO_SCALE              (0.07326)
#define ACCEL_SCALE             (0.0004625)
#define gravity                 (9.806)
#define FSAMP                   (10)
#define RAD2DEG                 (180.0/3.1415)

int16_t GYRO_MULT = GYRO_SCALE*256.0;
int16_t TILT_MULT = 0.1*256.0;

// Persistant states (8.8) -- covaraince is < 1
volatile int16_t P_00 = 0.8*256.0;
volatile int16_t P_01 = 0;
volatile int16_t P_10 = 0;
volatile int16_t P_11 = 0.8*256.0;

// Constants (8.8)
int16_t A_01 = (float)(256.0/FSAMP);
int16_t B_00 = (float)(256.0/FSAMP);

// Accelerometer variance (8.8) = 22*ACCEL_SCALE*g
int16_t Sz = 22*ACCEL_SCALE*gravity*256.0;

// Gyro variance (8.8) = 96E-6
int16_t Sw_00 = 96E-6*256.0;


// Output (8.8)
volatile int16_t x_00 = 0;
volatile int16_t x_10= 0;

void run_kalman(int16_t gRead, int16_t tilt)
{
    // Update the state estimate by extrapolating current state estimate with input u.
    // x = A * x + B * u
    //
    x_00 = x_00 + ((int32_t)(A_01 * x_10) + (int32_t)(B_00 * gRead))/256;
    cout<<"t1[0] : "<<x_00/256.0<<endl;

    // Compute the innovation -- error between measured value and state.
    // inn = y - c * x
    int16_t inn_00 = tilt - x_00;

    // Compute the covariance of the innovation.
    // s = C * P * C' + Sz
    int16_t s_00 = P_00 + Sz;
    cout<<"t1[1] : "<<s_00/256.0<<endl;

    // Compute AP matrix for use below.
    // AP = A * P
    int16_t AP_00 = P_00 + (int32_t)(A_01 * P_10)/256;
    int16_t AP_01 = P_01 + (int32_t)(A_01 * P_11)/256;
    int16_t AP_10 = P_10;
    int16_t AP_11 = P_11;

    // Compute the kalman gain matrix.
    // K = A * P * C' * inv(s)
    int16_t K_00 = ((int32_t)(AP_00*256) / s_00);
    int16_t K_10 = ((int32_t)(AP_10*256) / s_00);
    cout<<"t1[2] : "<<K_00/256.0<<endl;
    cout<<"t1[3] : "<<K_10/256.0<<endl;

    // Update the state estimate.
    // x = x + K * inn
    x_00 = x_00 + (int32_t)(K_00 * inn_00)/256;
    x_10 = x_10 + (int32_t)(K_10 * inn_00)/256;

    // Compute the new covariance of the estimation error.
    // P = A * P * A' - K * C * P * A' + Sw
    P_00 = AP_00 + ((int32_t)(AP_01 * A_01) - (int32_t)(K_00 * P_00))/256 + (int32_t)(((int32_t)(K_00 * P_01)/256) * A_01)/256 + Sw_00;
    P_01 = AP_01 - (int32_t)(K_00 * P_01)/256;
    P_10 = AP_10 + ((int32_t)(AP_11 * A_01) - (int32_t)(K_10 * P_00))/256 + (int32_t)(((int32_t)(K_10 * P_01)/256) * A_01)/256;
    P_11 = AP_11 - (int32_t)(K_10 * P_01)/256;
    
}

int main()
{
    /*
    cout<<"------------ constants -------------"<<endl;
    cout<<"A_01 : "<<A_01<<endl;
    cout<<"P_00 : "<<P_00<<endl;
    cout<<"P_01 : "<<P_01<<endl;
    cout<<"P_10 : "<<P_10<<endl;
    cout<<"P_11 : "<<P_11<<endl;
    cout<<"Sz : "<<Sz<<endl;
    cout<<"Sw_00 : "<<Sw_00<<endl;
    cout<<"---------------------"<<endl;
    */

    fin.open("tilt1.txt");
    string a;
    fin>>a;
    int16_t t1[4];
    int16_t gRead[500]= {0}, tilt[500] = {0}, muCAngle[500] = {0}, compAngle[500] = {0};
    for(int i=0; i<10; i++)
    {
        fin>>gRead[i];
        fin>>tilt[i];
        fin>>muCAngle[i];
        fin>>t1[0];fin>>t1[1];fin>>t1[2];fin>>t1[3];

        run_kalman(gRead[i], tilt[i]);
        compAngle[i] = x_00;
        cout<<"gRead : "<<gRead[i]/256.0<<" Tilt : "<<tilt[i]/256.0<<" muC : "<<muCAngle[i]/256.0<<" Comp : "<<compAngle[i]/256.0<<endl;
        cout<<t1[0]/256.0<<" "<<t1[1]/256.0<<" "<<t1[2]/256.0<<" "<<t1[3]/256.0<<endl<<endl;
    }
    return 0;
}
