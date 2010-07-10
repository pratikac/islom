/*
    Copyright (c) 2006 Michael P. Thompson <mpthompson@gmail.com>

    Permission is hereby granted, free of charge, to any person 
    obtaining a copy of this software and associated documentation 
    files (the "Software"), to deal in the Software without 
    restriction, including without limitation the rights to use, copy, 
    modify, merge, publish, distribute, sublicense, and/or sell copies 
    of the Software, and to permit persons to whom the Software is 
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be 
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
    DEALINGS IN THE SOFTWARE.

    $Id:$

    The concepts for this code were derived from the June, 2001 
    Embedded Systems Programming magazine article titled 
    "Kalman Filtering" by Dan Simon.
*/


#include <math.h>
#include <stdio.h>

#define SAMPLE_COUNT 512

// Sample data gathered directly from the SparkFun IMU 5 Degrees of Freedom
// IMU every 1/50th of a second using 10-bit ADC on an ATmega168.  The first
// column is the rate from the gyro (degrees/sec) and the second column is the 
// accelerometer pitch attitude from horizontal in degrees.
float sample_data[SAMPLE_COUNT][2] =
{
    {0.016088, 1.668337},
    {0.016088, 1.101706},
    {0.016088, 1.652305},
    {0.016088, 1.668337},
    {0.016088, 1.652305},
    {0.016088, 1.101706},
    {0.016088, 1.101706},
    {0.016088, 1.112400},
    {0.016088, 1.652305},
    {0.016088, 1.112400},
    {0.016088, 1.101706},
    {0.016088, 1.112400},
    {0.016088, 1.652305},
    {0.016088, 1.091216},
    {0.016088, 1.652305},
    {0.016088, 1.652305},
    {0.016088, 1.101706},
    {0.016088, 1.652305},
    {0.016088, 1.652305},
    {0.016088, 1.101706},
    {0.016088, 1.101706},
    {0.016088, 1.652305},
    {0.016088, 1.652305},
    {0.016088, 1.652305},
    {0.016088, 1.101706},
    {0.016088, 1.652305},
    {0.016088, 1.101706},
    {0.016088, 1.652305},
    {0.016088, 1.101706},
    {0.016088, 1.101706},
    {0.016088, 1.091216},
    {0.016088, 1.652305},
    {0.016088, 1.668337},
    {0.016088, 1.112400},
    {0.016088, 1.091216},
    {0.016088, 1.101706},
    {0.016088, 1.101706},
    {0.048263, 1.652305},
    {0.016088, 1.101706},
    {0.016088, 1.101706},
    {0.016088, 1.668337},
    {0.016088, 1.101706},
    {0.016088, 1.101706},
    {0.016088, 1.101706},
    {-0.016088, 1.636577},
    {0.048263, 2.862406},
    {0.016088, -1.534341},
    {0.016088, 2.891270},
    {0.016088, 0.000000},
    {0.016088, 2.834111},
    {0.048263, 0.535459},
    {0.016088, 3.433631},
    {0.048263, -1.051181},
    {0.016088, 3.433631},
    {0.048263, -0.525635},
    {-0.016088, 2.834111},
    {0.016088, 0.540510},
    {0.016088, 1.684684},
    {0.016088, 1.101706},
    {0.016088, 1.080924},
    {0.016088, 1.701355},
    {0.016088, -0.530501},
    {0.016088, 2.245743},
    {0.016088, 1.668337},
    {0.016088, 2.806371},
    {0.016088, 0.000000},
    {0.016088, 1.101706},
    {0.016088, 0.545658},
    {0.048263, 1.668337},
    {0.016088, 1.652305},
    {0.048263, 0.545658},
    {0.016088, 1.668337},
    {0.016088, 2.245743},
    {0.016088, 0.000000},
    {0.048263, 1.668337},
    {0.016088, 2.245743},
    {0.048263, 1.080924},
    {-0.016088, 2.223961},
    {0.048263, 2.223961},
    {0.048263, 0.545658},
    {0.016088, 0.550904},
    {0.048263, 1.101706},
    {0.016088, 1.652305},
    {0.016088, 1.101706},
    {0.048263, 1.668337},
    {0.016088, 0.550904},
    {0.016088, 1.652305},
    {0.016088, 1.101706},
    {0.016088, 1.652305},
    {0.016088, 1.101706},
    {0.016088, 1.101706},
    {0.016088, 1.112400},
    {0.016088, 1.091216},
    {0.016088, 1.101706},
    {0.016088, 0.535459},
    {0.016088, 1.684684},
    {0.016088, 1.101706},
    {0.048263, 1.101706},
    {0.016088, 1.652305},
    {0.016088, 1.652305},
    {0.016088, 1.652305},
    {0.016088, 1.101706},
    {0.016088, 1.652305},
    {0.016088, 1.101706},
    {0.016088, 1.112400},
    {0.016088, 1.652305},
    {0.016088, 1.101706},
    {0.016088, 1.668337},
    {0.016088, 1.101706},
    {0.016088, 1.112400},
    {0.016088, 1.112400},
    {0.016088, 1.101706},
    {0.016088, 1.101706},
    {0.016088, 1.652305},
    {0.016088, 1.112400},
    {0.016088, 1.652305},
    {0.016088, 1.652305},
    {0.016088, 1.112400},
    {0.016088, 1.668337},
    {0.016088, 1.101706},
    {0.016088, 1.652305},
    {0.016088, 1.652305},
    {0.048263, 1.652305},
    {0.016088, 1.091216},
    {0.016088, 1.101706},
    {0.016088, 1.101706},
    {0.016088, 1.668337},
    {0.016088, 1.101706},
    {0.016088, 1.101706},
    {0.016088, 1.652305},
    {0.048263, 1.091216},
    {0.016088, 1.652305},
    {0.048263, 1.091216},
    {0.016088, 1.101706},
    {0.016088, 1.101706},
    {0.016088, 1.101706},
    {0.016088, 1.101706},
    {0.016088, 1.101706},
    {0.016088, 1.112400},
    {0.016088, 1.091216},
    {0.016088, 1.652305},
    {0.016088, 1.668337},
    {0.016088, 1.652305},
    {0.016088, 1.101706},
    {0.016088, 1.668337},
    {0.016088, 1.652305},
    {0.016088, 1.101706},
    {0.016088, 1.652305},
    {0.016088, 1.101706},
    {0.016088, 1.668337},
    {0.016088, 1.101706},
    {0.016088, 1.101706},
    {0.016088, 1.652305},
    {0.016088, 1.112400},
    {0.016088, 1.652305},
    {0.016088, 1.112400},
    {0.016088, 1.101706},
    {0.016088, 1.652305},
    {0.016088, 1.101706},
    {0.016088, 1.101706},
    {0.016088, 1.652305},
    {0.016088, 1.652305},
    {0.016088, 1.101706},
    {0.016088, 1.668337},
    {-0.016088, 1.668337},
    {0.016088, 1.101706},
    {0.016088, 1.668337},
    {0.016088, 1.101706},
    {0.016088, 1.101706},
    {0.016088, 1.652305},
    {0.048263, 1.652305},
    {0.016088, 1.652305},
    {0.016088, 1.101706},
    {0.016088, 1.668337},
    {0.016088, 1.091216},
    {0.016088, 1.652305},
    {0.016088, 1.636577},
    {0.016088, 1.652305},
    {0.016088, 1.652305},
    {0.016088, 1.668337},
    {0.016088, 1.652305},
    {0.016088, 1.091216},
    {0.048263, 1.112400},
    {0.016088, 1.091216},
    {0.016088, 1.101706},
    {0.016088, 1.091216},
    {0.016088, 1.668337},
    {0.016088, 1.101706},
    {0.016088, 1.652305},
    {0.016088, 1.652305},
    {0.016088, 1.101706},
    {0.016088, 1.112400},
    {0.016088, 1.101706},
    {0.016088, 1.101706},
    {0.016088, 2.202598},
    {0.016088, 2.245743},
    {-0.048263, 2.223961},
    {-0.016088, 0.545658},
    {0.048263, 0.000000},
    {0.016088, 1.668337},
    {0.016088, 3.366461},
    {-0.016088, 2.267955},
    {0.048263, 0.000000},
    {0.016088, 1.123303},
    {0.048263, 0.556252},
    {0.016088, 1.112400},
    {0.016088, 0.545658},
    {0.016088, 0.556252},
    {0.016088, 1.112400},
    {0.016088, 1.080924},
    {0.016088, 1.652305},
    {0.016088, 1.668337},
    {0.016088, 2.223961},
    {0.016088, 1.112400},
    {0.016088, 1.101706},
    {0.016088, 1.668337},
    {0.016088, 1.668337},
    {-0.016088, 1.668337},
    {0.016088, 1.652305},
    {-0.016088, 1.123303},
    {0.016088, 2.290610},
    {0.016088, 3.576334},
    {-0.016088, 2.313723},
    {-0.016088, 0.000000},
    {0.080438, 3.366461},
    {0.337838, 1.652305},
    {0.370013, -4.253836},
    {0.176963, -7.853313},
    {0.048263, 0.000000},
    {-0.048263, 0.000000},
    {-0.080438, -3.979382},
    {-0.080438, -2.556150},
    {0.048263, 3.044779},
    {0.209137, 1.273030},
    {0.048263, -4.899092},
    {-0.176963, -6.604836},
    {-0.337838, -8.130102},
    {-0.402187, -9.462322},
    {-0.305663, -10.905022},
    {-0.209137, -12.171458},
    {-0.144788, -14.036243},
    {-0.112612, -15.945396},
    {-0.080438, -16.238842},
    {-0.048263, -18.262890},
    {-0.144788, -16.858400},
    {-0.273488, -18.604965},
    {-0.273488, -24.200972},
    {-0.209137, -27.135139},
    {-0.337838, -27.783926},
    {-0.498712, -24.710798},
    {-0.402187, -28.679035},
    {-0.048263, -33.845341},
    {0.209137, -41.185921},
    {0.112612, -35.256359},
    {-0.209137, -35.272423},
    {-0.305663, -38.233829},
    {-0.209137, -39.999359},
    {-0.080438, -45.473511},
    {-0.080438, -49.236404},
    {-0.273488, -43.531200},
    {-0.498712, -42.414200},
    {-0.498712, -43.363422},
    {-0.402187, -46.824093},
    {-0.241313, -52.394009},
    {-0.144788, -52.352379},
    {-0.144788, -48.814075},
    {-0.112612, -51.423473},
    {-0.048263, -53.972630},
    {-0.112612, -58.120411},
    {-0.176963, -63.733364},
    {-0.273488, -63.741348},
    {-0.370013, -58.828651},
    {-0.402187, -63.434952},
    {-0.434363, -75.963760},
    {-0.691762, -78.231720},
    {-0.852637, -67.027283},
    {-0.563062, -62.949409},
    {-0.273488, -66.801407},
    {-0.176963, -70.497551},
    {-0.370013, -73.094841},
    {-0.434363, -75.203239},
    {-0.241313, -70.346184},
    {-0.048263, -65.979752},
    {0.209137, -69.918022},
    {0.241313, -80.819466},
    {-0.016088, -81.950943},
    {-0.209137, -76.526848},
    {-0.144788, -74.291367},
    {0.048263, -78.799202},
    {0.112612, -80.180702},
    {0.016088, -82.519119},
    {-0.016088, -79.957977},
    {-0.048263, -83.395172},
    {-0.080438, -85.186455},
    {-0.080438, -79.027763},
    {0.080438, -77.471199},
    {0.112612, -84.907883},
    {0.016088, -83.088776},
    {-0.144788, -77.347450},
    {-0.080438, -72.474434},
    {0.016088, -75.677284},
    {0.112612, -82.800774},
    {0.209137, -86.889160},
    {0.176963, -83.254425},
    {0.176963, -77.535072},
    {0.144788, -73.300758},
    {0.176963, -71.399933},
    {0.209137, -78.055824},
    {0.016088, -83.224350},
    {-0.273488, -85.872414},
    {-0.402187, -80.238228},
    {-0.370013, -69.376472},
    {-0.144788, -63.666920},
    {0.080438, -74.275253},
    {0.337838, -88.191269},
    {0.305663, -86.009087},
    {0.080438, -81.573029},
    {-0.080438, -75.510239},
    {-0.048263, -69.638405},
    {0.112612, -62.049030},
    {0.209137, -61.348171},
    {0.209137, -67.011284},
    {0.112612, -71.565056},
    {0.048263, -65.056099},
    {0.176963, -55.154270},
    {0.595237, -45.000000},
    {1.110038, -47.121101},
    {1.657013, -68.355568},
    {1.914412, -80.537682},
    {1.399613, -74.859024},
    {0.595237, -61.525799},
    {0.176963, -46.701359},
    {0.273488, -41.682220},
    {0.466538, -42.760525},
    {0.691762, -47.944050},
    {0.949163, -45.384537},
    {0.916988, -45.939194},
    {0.723938, -46.668339},
    {0.498712, -39.889584},
    {0.498712, -30.963758},
    {0.595237, -37.874989},
    {0.659587, -41.531769},
    {0.595237, -31.865978},
    {0.563062, -26.565052},
    {0.595237, -24.075500},
    {0.820463, -30.068586},
    {0.949163, -26.318090},
    {0.884813, -23.912319},
    {0.820463, -28.830971},
    {0.852637, -32.057377},
    {0.884813, -22.380136},
    {0.788288, -16.638805},
    {0.563062, -23.198591},
    {0.402187, -23.728392},
    {0.176963, -16.574007},
    {-0.016088, -18.257565},
    {-0.176963, -18.970407},
    {-0.144788, -10.784299},
    {-0.016088, -3.215484},
    {0.080438, -9.293308},
    {0.273488, -14.470294},
    {0.595237, -9.180542},
    {0.852637, -0.658543},
    {0.852637, -2.436648},
    {0.659587, -10.808230},
    {0.402187, -10.551811},
    {0.144788, -3.814075},
    {-0.144788, -6.037683},
    {-0.337838, -9.713251},
    {-0.370013, -8.447529},
    {-0.305663, 5.079608},
    {-0.176963, 3.576334},
    {-0.144788, -1.091216},
    {-0.080438, 2.313723},
    {0.080438, 3.366461},
    {0.273488, -1.507436},
    {0.337838, -1.080924},
    {0.209137, 0.603091},
    {-0.048263, 1.231977},
    {-0.144788, -3.366461},
    {-0.209137, -4.474896},
    {-0.209137, -3.423872},
    {-0.144788, 6.645278},
    {0.016088, -5.856013},
    {-0.112612, 7.829077},
    {0.370013, 8.011232},
    {0.241313, 7.480886},
    {0.209137, 6.404354},
    {0.144788, 4.214179},
    {-0.112612, 1.259045},
    {-0.498712, -0.622756},
    {-0.080438, 6.857073},
    {-0.080438, -0.795724},
    {0.016088, -5.617580},
    {0.176963, 4.398705},
    {0.241313, 8.130103},
    {-0.016088, -0.444144},
    {-0.048263, -1.032245},
    {-0.080438, 1.684684},
    {-0.048263, 2.161080},
    {-0.112612, -0.550904},
    {-0.080438, -3.209486},
    {-0.080438, -1.548158},
    {-0.048263, -1.652305},
    {-0.048263, -2.202598},
    {-0.048263, -1.123303},
    {-0.080438, 0.000000},
    {-0.144788, 0.000000},
    {-0.144788, -3.270488},
    {0.048263, -4.724452},
    {-0.048263, -0.550904},
    {-0.048263, -2.700630},
    {-0.016088, -4.993744},
    {-0.048263, -2.779167},
    {0.048263, -3.846030},
    {0.016088, -2.700630},
    {0.016088, -2.245743},
    {0.016088, -1.145763},
    {0.016088, -1.701355},
    {0.016088, -2.245743},
    {0.016088, -2.245743},
    {-0.016088, -2.245743},
    {0.016088, -2.223961},
    {-0.016088, -4.122298},
    {-0.016088, -3.209486},
    {-0.016088, -1.134422},
    {0.016088, -1.735705},
    {0.016088, -1.701355},
    {-0.016088, -2.245743},
    {-0.016088, -2.223961},
    {0.016088, -1.701355},
    {0.016088, -2.245743},
    {0.016088, -2.202598},
    {0.016088, -2.223961},
    {0.016088, -2.779167},
    {0.016088, -2.202598},
    {0.016088, -2.202598},
    {-0.016088, -2.202598},
    {0.016088, -2.223961},
    {0.016088, -2.223961},
    {0.016088, -2.181641},
    {0.016088, -2.752486},
    {0.016088, -2.752486},
    {0.016088, -2.752486},
    {0.016088, -2.202598},
    {-0.016088, -2.202598},
    {0.016088, -2.202598},
    {0.016088, -2.202598},
    {0.016088, -2.752486},
    {-0.016088, -2.752486},
    {0.016088, -2.202598},
    {0.016088, -2.202598},
    {0.016088, -2.223961},
    {0.016088, -2.752486},
    {0.016088, -2.223961},
    {0.016088, -2.202598},
    {0.016088, -2.223961},
    {0.016088, -2.202598},
    {0.016088, -2.202598},
    {0.016088, -2.223961},
    {0.016088, -2.202598},
    {0.016088, -2.223961},
    {0.016088, -2.202598},
    {0.016088, -2.223961},
    {0.016088, -2.202598},
    {0.016088, -1.652305},
    {0.016088, -2.202598},
    {0.016088, -2.202598},
    {0.016088, -2.202598},
    {-0.016088, -2.202598},
    {-0.016088, -2.223961},
    {-0.016088, -2.202598},
    {0.016088, -2.202598},
    {0.016088, -2.202598},
    {-0.016088, -2.223961},
    {0.016088, -2.202598},
    {0.016088, -2.202598},
    {0.016088, -2.223961},
    {0.016088, -2.202598},
    {0.016088, -2.752486},
    {0.016088, -2.202598},
    {0.016088, -2.223961},
    {0.016088, -2.202598},
    {0.016088, -2.223961},
    {0.016088, -2.752486},
    {0.016088, -2.202598},
    {0.016088, -2.202598},
    {0.016088, -2.223961},
    {-0.016088, -2.245743},
    {0.016088, -2.223961},
    {0.016088, -2.202598},
    {0.016088, -2.202598},
    {0.016088, -2.202598},
    {0.016088, -2.223961},
    {0.016088, -2.223961},
    {0.016088, -2.223961},
    {0.016088, -2.752486},
    {0.016088, -2.223961},
    {0.016088, -2.202598},
    {0.016088, -2.202598},
    {0.016088, -2.202598},
    {0.016088, -2.202598},
    {0.016088, -2.181641},
    {0.016088, -2.202598},
    {0.016088, -2.202598},
    {0.016088, -2.202598},
    {0.016088, -2.223961},
    {0.016088, -2.202598},
    {0.016088, -2.223961},
    {-0.016088, -2.223961},
    {0.016088, -2.202598},
    {0.016088, -2.202598}
};


float kalman_update(float gyro_rate, float accel_angle)
// Update the State Estimation and compute the Kalman Gain.
// The estimated angle is returned.
{
    // Inputs.
    float u = gyro_rate;
    float y = accel_angle;

    // Output.
    static float x_00 = 0.0;
    static float x_10 = 0.0;

    // Persistant states.
    static float P_00 = 0.001;
    static float P_01 = 0.003;
    static float P_10 = 0.003;
    static float P_11 = 0.003;

    // Constants.  

    // These are the delta in seconds between samples.
    const float A_01 = -0.019968;
    const float B_00 = 0.019968;

    // Data read from 1000 samples of the accelerometer had a variance of 0.07701688.
    const float Sz = 0.07701688;

    // Data read from 1000 samples of the gyroscope had a variance of 0.00025556.
    // XXX The values below were pulled from the autopilot site, but I'm not sure how to
    // XXX plug them into the process noise covariance matrix.  This needs to be
    // XXX further explored.
    const float Sw_00 = 0.001;
    const float Sw_01 = 0.003;
    const float Sw_10 = 0.003;
    const float Sw_11 = 0.003;

    // Temp.
    float s_00;
    float inn_00;
    float K_00;
    float K_10;
    float AP_00;
    float AP_01;
    float AP_10;
    float AP_11;
    float APAT_00;
    float APAT_01;
    float APAT_10;
    float APAT_11;
    float KCPAT_00;
    float KCPAT_01;
    float KCPAT_10;
    float KCPAT_11;

    // Update the state estimate by extrapolating current state estimate with input u.
    // x = A * x + B * u
    x_00 += (A_01 * x_10) + (B_00 * u);

    // Compute the innovation -- error between measured value and state.
    // inn = y - c * x
    inn_00 = y - x_00;

    // Compute the covariance of the innovation.
    // s = C * P * C' + Sz
    s_00 = P_00 + Sz;

    // Compute AP matrix for use below.
    // AP = A * P
    AP_00 = P_00 + A_01 * P_10;
    AP_01 = P_01 + A_01 * P_11;
    AP_10 = P_10;
    AP_11 = P_11;

    // Compute the kalman gain matrix.
    // K = A * P * C' * inv(s)
    K_00 = AP_00 / s_00;
    K_10 = AP_10 / s_00;
    
    // Update the state estimate.
    // x = x + K * inn
    x_00 += K_00 * inn_00;
    x_10 += K_10 * inn_00;

    // Compute the new covariance of the estimation error.
    // P = A * P * A' - K * C * P * A' + Sw
    APAT_00 = AP_00 + (AP_01 * A_01);
    APAT_01 = AP_01;
    APAT_10 = AP_10 + (AP_11 * A_01);
    APAT_11 = AP_11;
    KCPAT_00 = (K_00 * P_00) + (K_00 * P_01) * A_01;
    KCPAT_01 = (K_00 * P_01);
    KCPAT_10 = (K_10 * P_00) + (K_10 * P_01) * A_01;
    KCPAT_11 = (K_10 * P_01);
    P_00 = APAT_00 - KCPAT_00 + Sw_00;
    P_01 = APAT_01 - KCPAT_01 + Sw_01;
    P_10 = APAT_10 - KCPAT_10 + Sw_10;
    P_11 = APAT_11 - KCPAT_11 + Sw_11;

    // Return the estimate.
    return x_00;
}

int main(int argc, char **argv)
{
    int i;
    float gyro_input;
    float accel_input;
    float kalman_output;

    for (i = 0; i < SAMPLE_COUNT; ++i)
    {
        // Get the gyro and accelerometer input.
        gyro_input = sample_data[i][0];
        accel_input = sample_data[i][1];

        // Update the Kalman filter and get the output.
        kalman_output = kalman_update(gyro_input, accel_input);

        // Print out input data and the kalman output.
        printf("%d gyro=%7.3f deg/sec    accel=%7.3f deg    kalman_output=%5.1f deg\n", 
               i, gyro_input, accel_input, kalman_output);
    }

    return 0;
}
