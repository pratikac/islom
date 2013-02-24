/**********************************************************
*
*	fixed.h -	Performs the necessary definitions required 
*				for the fixed point variable usage. This 
*				file is not portable and may need to be 
*				changed for each compiler.
**********************************************************/

/**********************************************************
*
*	Includes
*
**********************************************************/
#define U8 unsigned char
#define S8 signed char
#define U16 unsigned short
#define S16 signed short
#define U32 unsigned int
#define S32 signed int
/**********************************************************
*
*	Fixed Point Type definitions
*
**********************************************************/
/***************
*
*	Range 0-1.9921875
*	Granularity 0.0078125
*
***************/
typedef union FIXED1_7tag{
		U8 full;
		struct part1_7tag{
			U8 fraction: 7;
			U8 integer: 1;
			}part;
		}FIXED1_7;

#define FIXED1_7CONST(A,B) (U8)((A<<7) + ((B + 0.00390625)*128))
#define MULT1_7(A,B) (U16)((A.full*B.full+64)>>7)
#define DIV1_7(A,B)(U16)( (((A.full<<8)/B.full)+1)/2)

/***************
*
*	Range 0-127.998046875
*	Granularity 0.001953125
*
***************/
typedef union FIXED7_9tag{
		U16 full;
		struct part7_9tag{
			U16 fraction: 9;
			U16 integer: 7;
			}part;
		}FIXED7_9;

#define FIXED7_9CONST(A,B) (U16)((A<<9) + ((B + 0.0009765625)*512))
#define MULT7_9(A,B) (U32)(A.full*B.full+256)>>9
#define DIV7_9(A,B)(U32)(((A.full<<10)/B.full)+1)/2

/***************
*
*	Range 0-127.998046875
*	Granularity 0.001953125
*
***************/
typedef union FIXED11_21tag{
		S32 full;
		struct part11_21tag{
			U32 fraction: 21;
			S32 integer: 11;
			}part;
		}FIXED11_21;

#define FIXED11_21CONST(A,B) (S32)((A<<21) + \
	((A<0?(B - 0.00000011920928955078125):\
	(B + 0.00000011920928955078125))*2097152))
S32 MULT11_21(FIXED11_21 A,FIXED11_21 B);
S32 DIV11_21(FIXED11_21 A,FIXED11_21 B);

/**********************************************************
*
*	Macros
*
**********************************************************/

/**********************************************************
*
*	Functions
*
**********************************************************/
