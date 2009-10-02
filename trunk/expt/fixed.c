#include "fixed.h"
#include "stdio.h"
void Test1_7(FIXED1_7 a, FIXED1_7 b);
void Test7_9(FIXED7_9 a, FIXED1_7 b);
void Test7_9_X(FIXED7_9 a, FIXED7_9 b);
void Test11_21(FIXED11_21 a, FIXED7_9 b);

int main()
{
	FIXED1_7 f1_7_variableA1;
	FIXED1_7 f1_7_variableA2;
	FIXED7_9 f7_9_variableB1;
	FIXED7_9 f7_9_variableB2;
	FIXED11_21 f11_21_variableC1,f11_21_variableC2;

	f1_7_variableA1.full = FIXED1_7CONST(0,0.25);
	f1_7_variableA2.full = FIXED1_7CONST(0,0.06);
	f7_9_variableB1.full = FIXED7_9CONST(25,0.6);
	f7_9_variableB2.full = FIXED7_9CONST(3,0.2);
	f11_21_variableC1.full = FIXED11_21CONST(-200,-0.75);
	f11_21_variableC2.full = FIXED11_21CONST(-5,-0.25);

	Test1_7(f1_7_variableA1,f1_7_variableA2);
	Test7_9(f7_9_variableB1,f1_7_variableA1);
	Test7_9_X(f7_9_variableB1,f7_9_variableB2);
	Test11_21(f11_21_variableC1,f7_9_variableB2);

	return 0;
}

void Test1_7(FIXED1_7 a, FIXED1_7 b)
{
	FIXED1_7 temp;

	printf(
		"Results of operations on 1_7 variables\n"
		);
	temp.full = a.full + b.full;
	printf("Addition result is %d.%2.2d\n",
		temp.part.integer,
		(temp.part.fraction*100+64)/128);
	if(a.full < b.full)
	{
		printf("a is less than b. Subtraction overflows.\n");
	}
	if(a.full == b.full)
	{
		printf("a is the same as b. Result = 0.\n");
	}
	if(a.full > b.full)
	{
		temp.full = a.full - b.full;
		printf("Subtraction result is %d.%2.2d\n",
			temp.part.integer,
			(temp.part.fraction*100+64)/128);
	}
}

void Test7_9(FIXED7_9 a, FIXED1_7 b)
{
	FIXED7_9 temp;

	printf(
		"\nResults of operations on 7_9 and 1_7 variables\n"
		);
	temp.full = a.full + (b.full<<2);
	printf("Addition result is %d.%1.1d\n",
		temp.part.integer,
		(temp.part.fraction*10+256)/512);
	if(a.full < (b.full<<2))
	{
		printf("a is less than b. Subtraction overflows.\n");
	}
	if(a.full == (b.full<<2))
	{
		printf("a is the same as b. Result = 0.\n");
	}
	if(a.full > (b.full<<2))
	{
		temp.full = a.full - (b.full<<2);
		printf("Subtraction result is %d.%1.1d\n",
			temp.part.integer,
			(temp.part.fraction*10+256)/512);
	}
}

void Test7_9_X(FIXED7_9 a, FIXED7_9 b)
{
	FIXED7_9 temp;

	printf(
		"\nResults of multiply and divide on 7_9 variables.\n"
		);
	temp.full = MULT7_9(a,b);
	printf("Multiply result is %d.%1.1d\n",
		temp.part.integer,
		(temp.part.fraction*10+256)/512);
	temp.full = DIV7_9(a,b);
	printf("Divide result is %d.%1.1d\n",
		temp.part.integer,
		(temp.part.fraction*10+256)/512);
}

void Test11_21(FIXED11_21 a, FIXED7_9 b)
{
	FIXED11_21 temp;

	printf(
		"\nResults of multiply and divide on 11_21 and 7_9 variables.\n"
		);
	temp.full = b.full << 12;
	temp.full = MULT11_21(a,temp);
	printf("Multiply result is %d.%2.2d\n",
		temp.part.integer,
		(temp.part.fraction*100+1048576)
		/2097152);
	temp.full = b.full << 12;
	temp.full = DIV11_21(a,temp);
	printf("Divide result is %d.%2.2d\n",
		temp.part.integer,
		(temp.part.fraction*100+1048576)
		/2097152);
}

S32 MULT11_21(FIXED11_21 a,FIXED11_21 b)
{
	S32 temp,result;
	S8 sign=-1;
	
	if(a.full < 0)
	{
		sign=1;
		a.full = -a.full;
	}
	if(b.full < 0)
	{
		sign^=1;
		b.full = -b.full;
	}

	result = (((a.full & 0x0000FFFF) * (b.full & 0x0000FFFF))+1048576)>>21;
	result = result + ((((a.full>>16) * (b.full & 0x0000FFFF))+16)>>5);
	result = result + ((((b.full>>16) * (a.full & 0x0000FFFF))+16)>>5);
	temp = (a.full>>16) * (b.full>>16);
	if(temp > 0xFFFF)
	{
		result = 0xFFFFFFFF;
	}
	else
	{
		result = result + (temp<<11);
	}
	return result * -sign;
}

S32 DIV11_21(FIXED11_21 a,FIXED11_21 b)
{
	double temp;
	FIXED11_21 result;
	U8 sign=0;

	temp = (double)a.full/(double)b.full;
	if(temp<0)
	{
		temp = -temp;
		sign = 1;
	}
	result.part.integer = temp;
	result.part.fraction = ((temp-result.part.integer)*4194304 + 1)/2;
	result.part.integer *= -sign;
	return result.full;
}

