#ifndef HARDWARE_H
#define HARDWARE_H

#include <p33fxxxx.h>
#include <stdlib.h>

#define INNER_LED	_LATC2
#define OUTER_LED	_LATB4
#define ACCEL       _LATC1
#define GYRO        _LATA1

#define NOKIA_RST       _LATC1
#define NOKIA_CMD       _LATA1
#define NOKIA_CS        _LATB3

#define SEL     0
#define DESEL   1

// Addresses in ADIS
#define XACCL	0x0400
#define YACCL	0x0600
#define XINCL	0x0C00
#define YINCL	0x0E00
#define GRATE	0x0400
#define GANGL   0x0E00

#define IN1         _LATC7
#define IN2         _LATC6
#define DI1         _LATB9
#define DI2         _LATB8

#define ENABLE_MOTOR    {DI1 = 0; DI2 = 1;}
#define DISABLE_MOTOR   {DI1 = 1; DI2 = 0;}
#define PRD             (500)

// set up the clock here
#define FIN	((unsigned long)7370000)

_FOSCSEL(FNOSC_FRCPLL);
_FOSC(FCKSM_CSDCMD & OSCIOFNC_ON & POSCMD_NONE);
_FWDT(FWDTEN_OFF);
_FPOR(PWMPIN_ON & HPOL_ON & LPOL_ON & FPWRT_PWR1);
_FGS(GSS_OFF & GCP_OFF & GWRP_OFF);
_FICD(JTAGEN_OFF & ICS_PGD2);

#define M	    43
#define N1	    2
#define N2	    2
#define FOSC	((unsigned long)((FIN/(N1*N2))*M))
#define FCY	    ((unsigned long)FOSC/2)

#define BAUDRATE	(9600)
#define BRGVAL		(((FCY/BAUDRATE)/16)-1)

#endif
