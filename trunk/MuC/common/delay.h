void delay(unsigned int time);
void delay1ms(void);
void delay_us(unsigned int time);
void delay1us(void);
 
void delay(unsigned int time)
{
	unsigned int i=0;
	for(i=0;i<time;i++)
	{
		delay1ms();
	}
	return;
}
 void delay1ms(void)
 {
 	unsigned int i=0;
 	for(i=0;i<40000;i++)
 	{
 		Nop();
 	}
 	return;
 }
 
 void delay_us(unsigned int time)
{
	unsigned int i=0;
	for(i=0;i<time;i++)
	{
		delay1us();
	}
	return;
}
 void delay1us(void)
 {
 	unsigned int i=0;
 	for(i=0;i<40;i++)
 	{
 		Nop();
 	}
 	return;
 }

