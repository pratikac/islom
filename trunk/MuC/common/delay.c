void delay(unsigned int msecs)       // delay in ms
{
	unsigned int i=0;
	for(i=0; i<msecs; i++)
	{
		__delay_ms(1);
	}
	return;
}

void delayus(unsigned int usecs)
 {
 	unsigned int i=0;
 	for(i=0; i<usecs; i++)
 	{
 		__delay_us(1);
 	}
 	return;
 }
 