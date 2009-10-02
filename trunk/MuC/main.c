#include <p33fxxxx.h>
#include <stdlib.h>

#include "common/delay.h"
#include "common/uart.c"
#include "common/macros.h"

typedef unsigned long UINT32;
typedef signed long INT32;
typedef unsigned int UINT16;
typedef signed int INT16;
typedef unsigned char UINT8;
typedef signed char INT8;
typedef unsigned char bool;

#define INNER_LED	_LATC2
#define OUTER_LED	_LATB4
#define ACCEL       _LATC1
#define GYRO        _LATA1

#define SEL     0
#define DESEL   1

//clock
_FOSCSEL(FNOSC_FRCPLL);
_FOSC(FCKSM_CSDCMD & OSCIOFNC_ON & POSCMD_NONE);
_FWDT(FWDTEN_OFF);
_FPOR(PWMPIN_ON & HPOL_ON & LPOL_ON & FPWRT_PWR1);
_FGS(GSS_OFF & GCP_OFF & GWRP_OFF);
_FICD(JTAGEN_OFF & COE_ON);

#define M	    43
#define N1	    2
#define N2	    2
#define FOSC	((FIN/(N1*N2))*M)
#define FCY	    FOSC/2

// Addresses in ADIS
#define XACCL	0x0400
#define YACCL	0x0600
#define XINCL	0x0C00
#define YINCL	0x0E00
#define GRATE	0x0400
#define GANGL   0x0E00

#define SIGNED14(x) (((x & (1<<13))==0) ? (x & (0x1FFF)) : ((x & (0x1FFF)) | 0xE000))

volatile INT8 dat;
void _ISR _NOPSV _U1RXInterrupt(void)
{
    INNER_LED = ON;
    dat = U1RXREG;
    TX_string("caught\r\n");
    INNER_LED = OFF;
    _U1RXIF=0;
}    

int init_pll()
{
    PLLFBD = M - 2;
	CLKDIVbits.PLLPOST = N1 - 2;	
	CLKDIVbits.PLLPRE = N2/2 - 1;
	__builtin_write_OSCCONH(0x01);
	__builtin_write_OSCCONL(0x01);
	// Wait for Clock switch to occur
	while(OSCCONbits.COSC != 0b001);
	while(OSCCONbits.LOCK!=1) {}; // Wait for PLL to lock

    return 0;
}    

void peripheral_pin_config()
{
    __builtin_write_OSCCONL(OSCCON & ~BIT(6));	    

	//Uart through PICkit
	RPINR18bits.U1RXR = 11;     // U1RX
	_RP10R = 3;                 // U1TX;
	
	//SPI
    RPINR20bits.SDI1R = 3;      // RP3 mapped to SDI
    _RP16R = 0b00111;           // RP16 mapped to SDO
    _RP2R = 0b01000;            // RP2 mapped to SCK

	__builtin_write_OSCCONL(OSCCON | BIT(6));
}   

void init_adc()
{
    //ADC Init
    AD1CON1bits.FORM   	= 0;		// Data Output Format: Integer
    AD1CON1bits.SSRC   	= 0b111;	// Conversion starts when SAMP = 0
    AD1CON1bits.ASAM   	= 0;		// ADC Sample Control: Manual Sampling start
    AD1CON1bits.AD12B  	= 1;		// 12-bit ADC operation
    AD1CON1bits.SIMSAM 	= 0;		// Samples multiple channels individually in sequence
    AD1CON2bits.BUFM   	= 0;
    AD1CON2bits.CSCNA  	= 0;		// No scanning
    AD1CON2bits.CHPS	= 0;		// Converts CH0
    AD1CON2bits.ALTS   	= 0;        // No alternate samplings
    AD1CON3bits.ADRC	= 0; 		// ADC Clock is derived from Systems Clock
    AD1CON3bits.ADCS	= 63; 		// ADC Conversion Clock. Tad = Tcy/64
    AD1CON3bits.SAMC 	= 14; 		// ADC Conversion Clock 
    AD1CHS0bits.CH0SA	= 7;		// MUXA +ve input selection (AIN7) for CH0
    AD1CHS0bits.CH0NA	= 0;		// MUXA -ve input selection (Vref-) for CH0	
    AD1CHS0bits.CH0SB	= 0;		// MUXA +ve input selection (AIN7) for CH0
    AD1CHS0bits.CH0NB	= 0;		// MUXA -ve input selection (Vref-) for CH0	
    AD1PCFGL  			= ~BIT(7);  // Put these pins on ADC

    AD1CSSL				= 7;        // Scan none of the inputs
    AD1CON2bits.SMPI   	= 0;

    IFS0bits.AD1IF  	= 0;		// Clear the A/D interrupt flag bit
    IEC0bits.AD1IE  	= 0;		// Do Not Enable A/D interrupt until checkBattery finishes
    AD1CON1bits.ADON	= 0;		// Turn on the A/D converter
    AD1CON1bits.SAMP	= 0;        // Start sampling
}   
 
void init_spi()
{
    // SPI Init
    SPI1CON1bits.DISSCK = 0;    // Internal Serial Clock is Enabled
    SPI1CON1bits.DISSDO = 0;    // SDOx pin is controlled by the module
    SPI1CON1bits.MODE16 = 1;    // Communication is word-wide (16 bits)
    SPI1CON1bits.SMP = 0;       // Input Data is sampled at the middle of data output time
    SPI1CON1bits.CKE = 0;       // Serial output data changes on transition from
                                //      Idle clock state to active clock state
    SPI1CON1bits.CKP = 1;       // Idle state for clock is a low level; 
                                //      active state is a high level
    SPI1CON1bits.MSTEN = 1;     // Master Mode Enabled
    SPI1CON1bits.PPRE = 0b00;   // Primary Prescaler 64:1
    SPI1CON1bits.SPRE = 0b000;  // Secondary Prescaler 8:1
    SPI1STATbits.SPIROV = 0;    // Clear the Receive Overflow bit
    SPI1CON2 = 0x0000;
    SPI1STATbits.SPIEN = 1;     // Enable SPI Module
    //SPI1BUF = 0x0000;           // Write data to be transmitted
    IFS0bits.SPI1IF = 0;        // Clear the Interrupt Flag
    IEC0bits.SPI1IE = 0;        // Enable the Interrupt

}     

unsigned int spiWriteByte(unsigned int inData)
{
    unsigned int outData;
    while (SPI1STATbits.SPIRBF) 
    {
        outData = SPI1BUF; // Dummy read to clear the SPIRBF flag
    }    
    SPI1STATbits.SPIROV = 0;
    SPI1BUF = inData;     	
    while (SPI1STATbits.SPITBF);
    while(!SPI1STATbits.SPIRBF);
    outData = SPI1BUF;
    return outData;    		
}	


unsigned int spiReadByte(void)
{
    unsigned int outData;
    while (SPI1STATbits.SPIRBF) {
        outData = SPI1BUF; // Dummy read to clear the SPIRBF flag
    }    
    SPI1STATbits.SPIROV = 0;
    SPI1BUF = 0x0000;    	 		
    while (SPI1STATbits.SPITBF);
    while (!SPI1STATbits.SPIRBF);
    outData = SPI1BUF;    
    return outData;
}

unsigned int spiXfer(unsigned int data)
{
    unsigned int spiReg=0;

    spiReg = SPI1BUF;
    _SPIROV = 0;

    SPI1BUF = data;
    while (SPI1STATbits.SPITBF);
    while (!SPI1STATbits.SPIRBF);
    spiReg = SPI1BUF;

    return spiReg;
}

signed int accelRead(unsigned int addr)
{
    unsigned int dataOut;
    signed int accelVal=0;

    ACCEL = SEL;
    dataOut = (spiXfer(addr) & 0x3FFF);
    ACCEL = DESEL;
    delay_us(5);
    ACCEL = SEL;
    dataOut = (spiXfer(0x0000) & 0x3FFF);
    ACCEL = DESEL;
    accelVal = SIGNED14(dataOut);
    delay_us(5);

    return (accelVal);
}

signed int gyroRead(unsigned int addr)
{
    unsigned int dataOut;
    signed int gyroVal=0;

    GYRO = SEL;
    dataOut = (spiXfer(addr) & 0x3FFF);
    GYRO = DESEL;
    delay_us(5);
    GYRO = SEL;
    dataOut = spiXfer(0x0000);
    GYRO = DESEL;
    gyroVal = SIGNED14(dataOut);
    delay_us(5);

    return gyroVal;
}

void init_uart1()
{
    // UART 1
	U1BRG = BRGVAL;

	U1MODEbits.BRGH = 0;	    // Low speed mode
	U1MODEbits.STSEL = 0;	    // 1-stop bit
	U1MODEbits.PDSEL = 0;	    // No Parity, 8-data bits
	U1MODEbits.ABAUD = 0;	    // Autobaud Disabled

	_U1RXIE = 1;    		    // TX Interrupt Enable
	U1STAbits.URXISEL=0x00;     // Interrupt after one byte
	_U1RXIF=0;

	U1MODEbits.UARTEN = 1;	    // Enable UART
	U1STAbits.UTXEN = 1;	    // Enable UART Tx
}    

int main()
{
    TRISA = 0;
    TRISB = BIT(0)|BIT(1)|BIT(3)|BIT(7)|BIT(12);
	TRISC = 0;
    OUTER_LED = OFF;
    INNER_LED = OFF;

    init_pll();
    peripheral_pin_config();
    init_uart1();
    init_spi();
    init_adc();
    
    INT16 xAccel;
    INT16 yAccel;
    INT16 gRead;
    while(1)
    {
        xAccel = accelRead(XACCL);
        yAccel = accelRead(YACCL);
        gRead = gyroRead(GRATE);
        OUTER_LED = ON;
        INNER_LED = ON;

        TX_snum5(xAccel);
        TX('\t');
        TX_snum5(yAccel);
        TX('\t');
        TX_snum5(gRead);
        TX_string("\r\n");
        OUTER_LED = OFF;
        INNER_LED = OFF;
        delay(1);             //delay in ms
    }  
    return 0;      
};
