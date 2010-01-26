#define BAUDRATE	38400
#define BRGVAL		(((FCY/BAUDRATE)/16)-1)

char rxChar;

void TX(unsigned char data)
{
	U1TXREG = data;
	while(!U1STAbits.TRMT);
	return;
}

void TX_16(unsigned int data)
{
	TX((data & 0xFF00) >> 8);
	TX(data & 0x00FF);
	return;
}

void TX2(unsigned char data)
{
	U2TXREG = data;
	while(!U2STAbits.TRMT);
	return;
}

void TX2_16(unsigned int data)
{
	TX2((data & 0xFF00) >> 8);
	TX2(data & 0x00FF);
	return;
}

void TXI(unsigned char data)
{
	U1TXREG = data;
	return;
}

void TX_string(char * msg)
{
	while(*msg != 0x00)
		TX(*msg++);
}

void TX2_string(char * msg)
{
	while(*msg != 0x00)
		TX2(*msg++);
}

void TX_num2(unsigned int x) 
{
  unsigned int y;
  y=x/10;TX(y+0x30);x-=(y*10);
  TX(x+0x30);
}

void TX_num3(unsigned int x) 
{
  unsigned int y;
  y=x/100;TX(y+0x30);x-=(y*100);
  y=x/10;TX(y+0x30);x-=(y*10);
  TX(x+0x30);
}

void TX2_num3(unsigned int x) 
{
  unsigned int y;
  y=x/100;TX2(y+0x30);x-=(y*100);
  y=x/10;TX2(y+0x30);x-=(y*10);
  TX2(x+0x30);
}

void TX_num5(unsigned int x) 
{
  unsigned int y;
  y=x/10000;TX(y+0x30);x-=(y*10000);
  y=x/1000;TX(y+0x30);x-=(y*1000);
  y=x/100;TX(y+0x30);x-=(y*100);
  y=x/10;TX(y+0x30);x-=(y*10);
  TX(x+0x30);
}

void TX2_num5(unsigned int x) 
{
  unsigned int y;
  y=x/10000;TX2(y+0x30);x-=(y*10000);
  y=x/1000;TX2(y+0x30);x-=(y*1000);
  y=x/100;TX2(y+0x30);x-=(y*100);
  y=x/10;TX2(y+0x30);x-=(y*10);
  TX2(x+0x30);
}

void TX_snum5(signed int i) 
{
  unsigned int x,y;
  if(i<0)
  {
      x = -i;
      TX('-');
  }
  else 
    x = i;    
  
  y=x/10000;TX(y+0x30);x-=(y*10000);
  y=x/1000;TX(y+0x30);x-=(y*1000);
  y=x/100;TX(y+0x30);x-=(y*100);
  y=x/10;TX(y+0x30);x-=(y*10);
  TX(x+0x30);
}

void TX_snum7(signed int i) 
{
  unsigned int x,y;
  if(i<0)
  {
      x = -i;
      TX('-');
  }
  else 
    x = i;    
  
  y=x/1000000;TX(y+0x30);x-=(y*1000000);
  y=x/100000;TX(y+0x30);x-=(y*100000);
  y=x/10000;TX(y+0x30);x-=(y*10000);
  y=x/1000;TX(y+0x30);x-=(y*1000);
  y=x/100;TX(y+0x30);x-=(y*100);
  y=x/10;TX(y+0x30);x-=(y*10);
  TX(x+0x30);
}

void TX_num10(long int i) 
{
    unsigned long int x,y;
    if(i<0){
      x = -i;
      TX('-');
    }
    else 
    x = i;    
    
	y=x/1000000000;TX(y+0x30);x-=(y*1000000000);
	y=x/100000000;TX(y+0x30);x-=(y*100000000);
	y=x/10000000;TX(y+0x30);x-=(y*10000000);
	y=x/1000000;TX(y+0x30);x-=(y*1000000);
	y=x/100000;TX(y+0x30);x-=(y*100000);
	y=x/10000;TX(y+0x30);x-=(y*10000);
	y=x/1000;TX(y+0x30);x-=(y*1000);
    y=x/100;TX(y+0x30);x-=(y*100);
	y=x/10;TX(y+0x30);x-=(y*10);
	TX(x+0x30);
}

void RX(char * ptrChar)
{
	while(1)
	{
		/* check for receive errors */
		if(U1STAbits.FERR == 1)
		{
			continue;
		}
			
		/* must clear the overrun error to keep uart receiving */
		if(U1STAbits.OERR == 1)
		{
			U1STAbits.OERR = 0;
			continue;
		}

		/* get the data */
		if(U1STAbits.URXDA == 1)
		{
			* ptrChar = U1RXREG;
			break;
		}
	}
}

void RX2(char * ptrChar)
{
	while(1)
	{
		/* check for receive errors */
		if(U2STAbits.FERR == 1)
		{
			continue;
		}
			
		/* must clear the overrun error to keep uart receiving */
		if(U2STAbits.OERR == 1)
		{
			U2STAbits.OERR = 0;
			continue;
		}

		/* get the data */
		if(U2STAbits.URXDA == 1)
		{
			* ptrChar = U2RXREG;
			break;
		}
	}
}

unsigned char RXI(void)
{
    if(U1STAbits.FERR == 1){
        
    }
    
    if(U1STAbits.OERR == 1){
        U1STAbits.OERR = 0;
        
    }
    
    if(U1STAbits.URXDA == 1){
        rxChar = U1RXREG;
        return 1;
    }
    
    return 0;
}

unsigned char RX2I(void)
{
    if(U2STAbits.FERR == 1){
        
    }
    
    if(U2STAbits.OERR == 1){
        U2STAbits.OERR = 0;
        
    }
    
    if(U2STAbits.URXDA == 1){
        rxChar = U2RXREG;
        return 1;
    }
    
    return 0;
}
