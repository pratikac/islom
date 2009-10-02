#ifndef _1DHOPPING_H
#define _1DHOPPING_H


#include <p33fxxxx.h>
#include <stdlib.h>
#include <string.h>
#include "common/macros.h"
#include "common/delay.h"
#include "common/uart.c"

typedef unsigned long   UINT32;
typedef signed long     INT32;
typedef unsigned int    UINT16;
typedef signed int      INT16;
typedef unsigned char   UINT8;
typedef signed char     INT8;
typedef unsigned char   bool;

#define INTERRUPT_PROTECT(x) {           \
    char saved_ipl;                      \
                                         \
    SET_AND_SAVE_CPU_IPL(saved_ipl,7);   \
    x;                                   \
    RESTORE_CPU_IPL(saved_ipl); } (void) 0;


#define M	43
#define N1	2
#define N2	2
#define FOSC	((FIN/(N1*N2))*M)
#define FCY	    FOSC/2


#define BAUDRATE	38400
#define BRGVAL		(((FCY/BAUDRATE)/16)-1)

_FOSCSEL(FNOSC_FRCPLL);
//_FOSCSEL(FNOSC_PRIPLL);     // Primary RC oscillator with PLL
_FOSC(FCKSM_CSDCMD & OSCIOFNC_ON & POSCMD_NONE);  // No primary oscillator
//_FOSC(FCKSM_CSDCMD & OSCIOFNC_ON & POSCMD_XT);      // Primary oscillator is XT
_FWDT(FWDTEN_OFF);
_FPOR(PWMPIN_ON & HPOL_ON & LPOL_ON & FPWRT_PWR1);
_FGS(GSS_OFF & GCP_OFF & GWRP_OFF);
_FICD(JTAGEN_OFF & COE_ON);

#define DEG2LSB    (((INT32)256*273)/20)
#define DEG2CNT     (((INT32)371*2048)/100/360)
#define CNT2LSB    ((INT32)9*273*100/371/4)

// Pin Assignments and Functions
#define INNER_LED	_LATC2
#define OUTER_LED	_LATB4
#define ACCEL       _LATC1
#define GYRO        _LATA1

#define IN1         _LATC7
#define IN2         _LATC6
#define DI1         _LATB9
#define DI2         _LATB8

#define SEL     0
#define DESEL   1

#define ON		1
#define OFF		0

#define INTERRUPT_SW_PRESSED    (_RB7==0)
#define SIGNED14(x) (((x & (1<<13))==0) ? (x & (0x1FFF)) : ((x & (0x1FFF)) | 0xE000))

void systemInit(void);
unsigned int spiXfer(unsigned int);
signed int accelRead(unsigned int addr);
signed int gyroRead(unsigned int addr);
unsigned int spiWriteByte(unsigned int inData);
unsigned int spiReadByte(void);
inline INT32 arctan2(INT16,INT16);
INT32 atanLookup[] = {3128, 6254, 9378, 12497, 15610, 18715, 21811, 24897, 27971, 31032, 34078, 37109, 40122, 43117, 46093, 49048, 51981, 54892, 57779, 60641, 63479, 66290, 69074, 71831, 74559, 77259, 79929, 82570, 85181, 87761, 90310, 92828, 95315, 97771, 100195, 102587, 104948, 107277, 109574, 111839, 114073, 116275, 118446, 120586, 122695, 124773, 126820, 128838, 130825, 132782, 134710, 136609, 138479, 140321, 142134, 143920, 145678, 147409, 149113, 150791, 152443, 154070, 155671, 157248};

// unsigned int spiRead(unsigned char device, unsigned int address);
// void spiWrite(unsigned char device, unsigned int address, unsigned int data);


// Addresses in ADIS
#define XACCL	0x0400
#define YACCL	0x0600
#define XINCL	0x0C00
#define YINCL	0x0E00
#define GRATE	0x0400
#define GANGL   0x0E00

#define FSAMP   100
#define Kp      3
#define Kd      0

#endif

