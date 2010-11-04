#ifndef MACROS_H
#define MACROS_H

#define BIT(x) (1<<(x))
#define sbi(x,y)	((x) |= (1<<(y)))
#define cbi(x,y)	((x) &= ~(1<<(y)))
#define tbi(x,y) 	((x) ^= (1<<(y)))

#define _NOPSV __attribute__((no_auto_psv))

#define TRUE	1
#define FALSE	0

#define ON      1
#define OFF     0

#define SGN(x)      ( ((x) >= 0) ? (1) : (-1) )
#define ABS(x)      ( ((x) >= 0) ? (x) : (-(x)) )

#define SIGNED14(x) (((x & (1<<13))==0) ? (x & (0x1FFF)) : ((x & (0x1FFF)) | 0xE000))
#define SIGNED12(x) (((x & (1<<11))==0) ? (x & (0x07FF)) : ((x & (0x7FFF)) | 0xF800))

typedef unsigned long UINT32;
typedef signed long INT32;
typedef unsigned int UINT16;
typedef signed int INT16;
typedef unsigned char UINT8;
typedef signed char INT8;
typedef unsigned char BOOL;

#endif
