#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define SAMPLE_COUNT 512
FILE *op;

// Sample data gathered directly from the SparkFun IMU 5 Degrees of Freedom
// IMU every 1/20th of a second using 10-bit ADC on an ATmega168.  The first
// column is the rate from the gyro (degrees/sec) and the second column is the 
// pitch attitude from horizontal in degrees.
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


static void matrix_multiply(float* A, float* B, int m, int p, int n, float* C)
// Matrix Multiplication Routine
{
    // A = input matrix (m x p)
    // B = input matrix (p x n)
    // m = number of rows in A
    // p = number of columns in A = number of rows in B
    // n = number of columns in B
    // C = output matrix = A*B (m x n)
    int i, j, k;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
        {
          C[n*i+j]=0;
          for (k=0;k<p;k++)
            C[n*i+j]= C[n*i+j]+A[p*i+k]*B[n*k+j];
        }
}

static void matrix_addition(float* A, float* B, int m, int n, float* C)
// Matrix Addition Routine
{
    // A = input matrix (m x n)
    // B = input matrix (m x n)
    // m = number of rows in A = number of rows in B
    // n = number of columns in A = number of columns in B
    // C = output matrix = A+B (m x n)
    int i, j;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
            C[n*i+j]=A[n*i+j]+B[n*i+j];
}

static void matrix_subtraction(float* A, float* B, int m, int n, float* C)
// Matrix Subtraction Routine
{
    // A = input matrix (m x n)
    // B = input matrix (m x n)
    // m = number of rows in A = number of rows in B
    // n = number of columns in A = number of columns in B
    // C = output matrix = A-B (m x n)
    int i, j;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
            C[n*i+j]=A[n*i+j]-B[n*i+j];
}

static void matrix_transpose(float* A, int m, int n, float* C)
// Matrix Transpose Routine
{
    // A = input matrix (m x n)
    // m = number of rows in A
    // n = number of columns in A
    // C = output matrix = the transpose of A (n x m)
    int i, j;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
            C[m*j+i]=A[n*i+j];
}


static int matrix_inversion(float* A, int n, float* AInverse)
// Matrix Inversion Routine
{
    // A = input matrix (n x n)
    // n = dimension of A
    // AInverse = inverted matrix (n x n)
    // This function inverts a matrix based on the Gauss Jordan method.
    // The function returns 1 on success, 0 on failure.
    int i, j, iPass, imx, icol, irow;
    float det, temp, pivot, factor;
    float* ac = (float*)calloc(n*n, sizeof(float));
    det = 1;
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            AInverse[n*i+j] = 0;
            ac[n*i+j] = A[n*i+j];
        }
        AInverse[n*i+i] = 1;
    }

    // The current pivot row is iPass.
    // For each pass, first find the maximum element in the pivot column.
    for (iPass = 0; iPass < n; iPass++)
    {
        imx = iPass;
        for (irow = iPass; irow < n; irow++)
        {
            if (fabs(A[n*irow+iPass]) > fabs(A[n*imx+iPass])) imx = irow;
        }

        // Interchange the elements of row iPass and row imx in both A and AInverse.
        if (imx != iPass)
        {
            for (icol = 0; icol < n; icol++)
            {
                temp = AInverse[n*iPass+icol];
                AInverse[n*iPass+icol] = AInverse[n*imx+icol];
                AInverse[n*imx+icol] = temp;
                if (icol >= iPass)
                {
                    temp = A[n*iPass+icol];
                    A[n*iPass+icol] = A[n*imx+icol];
                    A[n*imx+icol] = temp;
                }
            }
        }

        // The current pivot is now A[iPass][iPass].
        // The determinant is the product of the pivot elements.
        pivot = A[n*iPass+iPass];
        det = det * pivot;
        if (det == 0)
        {
            free(ac);
            return 0;
        }

        for (icol = 0; icol < n; icol++)
        {
            // Normalize the pivot row by dividing by the pivot element.
            AInverse[n*iPass+icol] = AInverse[n*iPass+icol] / pivot;
            if (icol >= iPass) A[n*iPass+icol] = A[n*iPass+icol] / pivot;
        }

        for (irow = 0; irow < n; irow++)
        {
            // Add a multiple of the pivot row to each row.  The multiple factor
            // is chosen so that the element of A on the pivot column is 0.
            if (irow != iPass) factor = A[n*irow+iPass];
            for (icol = 0; icol < n; icol++)
            {
                if (irow != iPass)
                {
                    AInverse[n*irow+icol] -= factor * AInverse[n*iPass+icol];
                    A[n*irow+icol] -= factor * A[n*iPass+icol];
                }
            }
        }
    }

    free(ac);
    return 1;
}

static void matrix_print(float* A, int m, int n)
// Matrix print.
{
    // A = input matrix (m x n)
    // m = number of rows in A
    // n = number of columns in A
    int i, j;
    for (i=0;i<m;i++)
    {
        printf("| ");
        for(j=0;j<n;j++)
        {
            printf("%7.3f ", A[n*i+j]);
        }
        printf("|\n");
    }
}


// n states
// m inputs
// r outputs
#define n 2
#define m 1
#define r 1

float kalman_update(float gyroscope_rate, float accelerometer_angle)
{
    // A is an n by n matrix
    // B is an n by m matrix
    // C is an r by n matrix
    // Sz is an r by r matrix
    // Sw is an n by n matrix
    // xhat is an n by 1 vector
    // P is an n by n matrix
    // y is an r by 1 vector
    // u is an m by 1 vector

    // Constants.
    static float A[n][n] = {{1.0, -0.019968}, {0.0, 1.0}};
    static float B[n][m] = {{0.019968}, {0.0}};
    static float C[r][n] = {{1.0, 0.0}};
    static float Sz[r][r] = {{17.2}};
    static float Sw[n][n] = {{0.005, 0.005}, {0.005, 0.005}};

    // Persistant states.
    static float xhat[n][1] = {{0.0}, {0.0}};
    static float P[n][n] = {{0.005, 0.005}, {0.005, 0.005}};

    // Inputs.
    float u[m][m];              // Gyroscope rate.
    float y[m][m];              // Accelerometer angle.

    // Temp values.
    float AP[n][n];             // This is the matrix A*P
    float CT[n][r];             // This is the matrix C'
    float APCT[n][r];           // This is the matrix A*P*C'
    float CP[r][n];             // This is the matrix C*P
    float CPCT[r][r];           // This is the matrix C*P*C'
    float CPCTSz[r][r];         // This is the matrix C*P*C'+Sz
    float CPCTSzInv[r][r];      // This is the matrix inv(C*P*C'+Sz)
    float K[n][r];              // This is the Kalman gain.
    float Cxhat[r][1];          // This is the vector C*xhat
    float yCxhat[r][1];         // This is the vector y-C*xhat
    float KyCxhat[n][1];        // This is the vector K*(y-C*xhat)
    float Axhat[n][1];          // This is the vector A*xhat 
    float Bu[n][1];             // This is the vector B*u
    float AxhatBu[n][1];        // This is the vector A*xhat+B*u
    float AT[n][n];             // This is the matrix A'
    float APAT[n][n];           // This is the matrix A*P*A'
    float APATSw[n][n];         // This is the matrix A*P*A'+Sw
    float KC[n][n];             // This is the matrix K*C
    float KCP[n][n];            // This is the matrix K*C*P
    float KCPAT[n][n];          // This is the matrix K*C*P*A'

    // Fill in inputs.
    u[0][0] = gyroscope_rate;
    y[0][0] = accelerometer_angle;

#if 0
    // Print various matrices.
    printf("u =\n");
    matrix_print((float*) u, m, m);
    printf("y =\n");
    matrix_print((float*) y, m, m);
    printf("A =\n");
    matrix_print((float*) A, n, n);
    printf("B =\n");
    matrix_print((float*) B, n, m);
    printf("State =\n");
    matrix_print((float*) xhat, n, 1);
#endif

    // Update the state estimate by extrapolating estimate with gyroscope input.
    // xhat_est = A * xhat + B * u
    matrix_multiply((float*) A, (float*) xhat, n, n, 1, (float*) Axhat);
    matrix_multiply((float*) B, (float*) u, n, r, 1, (float*) Bu);
    matrix_addition((float*) Axhat, (float*) Bu, n, 1, (float*) AxhatBu);

#if 0
    printf("State Estimate =\n");
    matrix_print((float*) AxhatBu, n, 1);
#endif

    // Compute the innovation.
    // Inn = y - c * xhat;
    matrix_multiply((float*) C, (float*) xhat, r, n, 1, (float*) Cxhat);
    matrix_subtraction((float*) y, (float*) Cxhat, r, 1, (float*) yCxhat);

#if 0
    printf("Innovation =\n");
    matrix_print((float*) yCxhat, r, 1);
#endif

    // Compute the covariance of the innovation.
    // s = C * P * C' + Sz
    matrix_transpose((float*) C, r, n, (float*) CT);
    matrix_multiply((float*) C, (float*) P, r, n, n, (float*) CP);
    matrix_multiply((float*) CP, (float*) CT, r, n, r, (float*) CPCT);
    matrix_addition((float*) CPCT, (float*) Sz, r, r, (float*) CPCTSz);

    // Compute the kalman gain matrix.
    // K = A * P * C' * inv(s)
    matrix_multiply((float*) A, (float*) P, n, n, n, (float*) AP);
    matrix_multiply((float*) AP, (float*) CT, n, n, r, (float*) APCT);
    matrix_inversion((float*) CPCTSz, r, (float*) CPCTSzInv);
    matrix_multiply((float*) APCT, (float*) CPCTSzInv, n, r, r, (float*) K);

    // Update the state estimate.
    // xhat = xhat_est + K * Inn;
    matrix_multiply((float*) K, (float*) yCxhat, n, r, 1, (float*) KyCxhat);
    matrix_addition((float*) AxhatBu, (float*) KyCxhat, n, 1, (float*) xhat);

    // Compute the new covariance of the estimation error.
    // P = A * P * A' - K * C * P * A' + Sw
    matrix_transpose((float*) A, n, n, (float*) AT);
    matrix_multiply((float*) AP, (float*) AT, n, n, n, (float*) APAT);
    matrix_addition((float*) APAT, (float*) Sw, n, n, (float*) APATSw);
    matrix_multiply((float*) K, (float*) C, n, r, n, (float*) KC);
    matrix_multiply((float*) KC, (float*) P, n, n, n, (float*) KCP);
    matrix_multiply((float*) KCP, (float*) AT, n, n, n, (float*) KCPAT);
    matrix_subtraction((float*) APATSw, (float*) KCPAT, n, n, (float*) P);

    // Return the estimate.
    return xhat[0][0];
}

int main(int argc, char **argv)
{
    int i;
    float gyro_input;
    float accel_input;
    float kalman_output;
	float gyro_angle=0;

	op = fopen("data1.dat", "w");
	fprintf(op, "Start\n");
    for (i = 0; i < SAMPLE_COUNT; ++i)
    {
        // Get the gyro and accelerometer input.
        gyro_input = sample_data[i][0];
        accel_input = sample_data[i][1];

        // Update the Kalman filter and get the output.
        kalman_output = kalman_update(gyro_input, accel_input);

		gyro_angle += (gyro_input)*0.05;
		//print to file
		fprintf(op,"%d,%f,%f,%f\n", i, gyro_angle, accel_input, kalman_output);
        // Print out input data and the kalman output.
        //printf("%d gyro=%7.3f deg/sec  accel=%7.3f degrees  kalman_output=%5.1f degrees\n", i, gyro_input, accel_input, kalman_output);
    }
	fprintf(op, "Done\n");
	fclose(op);
    return 0;
}
