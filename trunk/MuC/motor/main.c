#include <p33fxxxx.h>
#include <stdlib.h>

#include "../common/delay.h"
#include "../common/uart.c"
#include "../common/macros.h"

typedef unsigned long UINT32;
typedef signed long INT32;
typedef unsigned int UINT16;
typedef signed int INT16;
typedef unsigned char UINT8;
typedef signed char INT8;
typedef unsigned char bool;

#define INNER_LED	_LATC2
#define OUTER_LED	_LATB4

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

#define DI1         _LATB9
#define DI2         _LATB8

#define ENABLE_MOTOR    {DI1 = 0; DI2 = 1;}
#define DISABLE_MOTOR   {DI1 = 1; DI2 = 0;}

#define PRD     500

volatile int newData = FALSE;
volatile INT16 prevPos = 0;
volatile INT16 currPos = 0;
volatile INT16 vel = 0;     // count/s
volatile INT16 vel_profile = 0;
volatile INT16 accel = 100;           // add this to vel every interrupt
INT16 vel_error;
INT16 Kp=4, Kd=0;
INT16 motorCommand = 0;

void setMotorSpeed (INT16 speed);
volatile INT16 plateau_flag = 0;
volatile INT16 plateau_count;
void _ISR _NOPSV _T1Interrupt(void) //Running at 50Hz
{
    OUTER_LED = ON;
   
    prevPos = currPos;
    currPos = POS1CNT;
    vel = (INT32)((currPos - prevPos)*50);      // PID on 4*counts
    vel_profile += accel;
    if(vel_profile > 4000)
    {
        if(plateau_flag == 0)
            plateau_flag = 1;
        else
            plateau_count++;
        if(plateau_count == 300)
            accel = accel*-1;
        vel_profile = 4000;
    } 
    if(vel_profile < 0)
        vel_profile = 0;

    vel_error = vel_profile - vel;
    motorCommand += (INT32)(Kp*vel_error/256);
    setMotorSpeed(motorCommand);    
    
    newData = TRUE;
    _T1IF = 0;
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
	
  	//Quadrature decoding
	RPINR14bits.QEA1R = 0;
	RPINR14bits.QEB1R = 1;

	//Output Compare
	_RP22R = 0b10010;           // RP22 tied to OC1
	_RP23R = 0b10011;           // RP23 tied to OC2

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

void init_uart1()
{
    // UART 1
	U1BRG = BRGVAL;

	U1MODEbits.BRGH = 0;	    // Low speed mode
	U1MODEbits.STSEL = 0;	    // 1-stop bit
	U1MODEbits.PDSEL = 0;	    // No Parity, 8-data bits
	U1MODEbits.ABAUD = 0;	    // Autobaud Disabled

	_U1RXIE = 0;    		    // TX Interrupt Disable
	U1STAbits.URXISEL=0x00;     // Interrupt after one byte
	_U1RXIF=0;

	U1MODEbits.UARTEN = 1;	    // Enable UART
	U1STAbits.UTXEN = 1;	    // Enable UART Tx
}    

void init_timer1()
{
    // Initialize and enable Timer1 for Control Loop at 50Hz
    T1CONbits.TON = 1;          // Enable Timer
    T1CONbits.TCS = 0;          // Select internal instruction cycle clock
    T1CONbits.TGATE = 0;        // Disable Gated Timer mode
    T1CONbits.TCKPS = 0b11;     // Select 1:256 Prescaler
    TMR1 = 0x00;                // Clear timer register
    PR1 = 3094;                 // Load the period value for internal osc

    IPC0bits.T1IP = 0x01;       // Set Timer 1 Interrupt Priority Level
    IFS0bits.T1IF = 0;          // Clear Timer 1 Interrupt Flag
    IEC0bits.T1IE = 1;          // Timer 1 interrupt Enable
    T1CONbits.TON = 0;          // Timer Start Enable
}

// Motor PWM timer    
void init_timer2()
{
  	// Initialize and enable Timer2 for Output Compare
	OC1CONbits.OCM = 0b000;     // Disable Output Compare Module
	OC1CONbits.OCTSEL = 0;      // Select Timer 2 as output compare time base
	OC1R = 0;                   // Load the Compare Register Value
	OC1RS = 0;                // Write the duty cycle for the second PWM pulse
	OC1CONbits.OCM = 0b110;     // Select the Output Compare mode 
	OC2CONbits.OCM = 0b000;     // Disable Output Compare Module
	OC2CONbits.OCTSEL = 0;      // Select Timer 2 as output compare time base
	OC2R = 0;                   // Load the Compare Register Value
	OC2RS = 0;                // Write the duty cycle for the second PWM pulse
	OC2CONbits.OCM = 0b110;     // Select the Output Compare mode 

	T2CONbits.TON = 0;          // Disable Timer
	T2CONbits.TCS = 0;          // Select internal instruction cycle clock
	T2CONbits.TGATE = 0;        // Disable Gated Timer mode
	T2CONbits.TCKPS = 0b01;     // Select 1:8 Prescaler
	TMR2 = 0x00;                // Clear timer register
	PR2 = 500;                  // Load the period value

	IPC1bits.T2IP = 0x01;       // Set Timer 2 Interrupt Priority Level
	IFS0bits.T2IF = 0;          // Clear Timer 2 Interrupt Flag
	IEC0bits.T2IE = 0;          // Timer 2 interrupt enable
	T2CONbits.TON = 1;          // Start Timer
}

void setMotorSpeed(INT16 speed)
{
	if(speed >= 0)
	{
		OC2RS = 0;
		if(speed < PRD) OC1RS = speed;
		else            OC1RS = PRD;
	}
	else
	{
		OC1RS = 0;
		if(speed >-PRD) OC2RS = -speed;
		else            OC1RS = PRD;
	}
}

void init_qei1()
{
  	//QEI Interface
	QEI1CONbits.QEIM = 0b111;       // QEI enabled in x4 mode, POSCNT reset by match 
	DFLT1CONbits.QEOUT = 1;         // Digital filter outputs enabled
	DFLT1CONbits.QECK = 0b010;      //1:4 for digital filter
	DFLT1CONbits.CEID = 1;          //Count error interrupt disabled
	MAX1CNT = 0xFFFF;               //MAXCNT to 2^16-1
}    

int main()
{
    TRISA = 0;
    TRISB = BIT(0)|BIT(1)|BIT(3)|BIT(7)|BIT(12);
	TRISC = 0;
    OUTER_LED = OFF;
    INNER_LED = OFF;

    TX_string("Init peripherals...\r\n");
    init_pll();
    peripheral_pin_config();
    init_uart1();
    init_adc();
    init_timer1();
    init_timer2();
    init_qei1();
            
    TX_string("Start\r\n");
        
    T1CONbits.TON = 1;
    ENABLE_MOTOR;
    
    POS1CNT = 0;
   
    while(1)
    {
        if( newData == TRUE)
        {
            TX_snum5(vel_profile);
            TX('\t');
            TX_snum5(vel);
            TX('\t');
            TX_snum5(motorCommand);
            TX_string("\r\n");
                
            OUTER_LED = OFF;            
            newData = FALSE;
        }
    }  
    return 0;      
};
