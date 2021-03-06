#include <p33fxxxx.h>
#include <stdlib.h>

#include "../common/delay.h"
#include "../common/uart.c"
#include "../common/macros.h"

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
_FICD(JTAGEN_OFF & ICS_PGD2);

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
#define SIGNED12(x) (((x & (1<<11))==0) ? (x & (0x07FF)) : ((x & (0x7FFF)) | 0xF800))

#define IN1         _LATC7
#define IN2         _LATC6
#define DI1         _LATB9
#define DI2         _LATB8

#define ENABLE_MOTOR    {DI1 = 0; DI2 = 1;}
#define DISABLE_MOTOR   {DI1 = 1; DI2 = 0;}
#define PRD             (500)


signed int accelRead(unsigned int addr);
signed int gyroRead(unsigned int addr);

volatile int newData = FALSE;
volatile INT16 tiltX;
volatile INT16 tiltY;
volatile INT16 tilt;
volatile INT16 gRead;

// Kalman filter variables
#define GYRO_SCALE              (0.07326)
#define ACCEL_SCALE             (0.0004625)
#define gravity                 (9.806)
#define FSAMP                   (50)
#define RAD2DEG                 (180.0/3.1415)

// 10.6
volatile INT16 GYRO_MULT = GYRO_SCALE*64.0;
volatile INT16 TILT_MULT = 0.1*64.0;

// Persistant states (10.6) -- covaraince is < 1
volatile INT16 P_00 = 64.0;
volatile INT16 P_01 = 0;
volatile INT16 P_10 = 0;
volatile INT16 P_11 = 64.0;

// Constants (10.6)
volatile INT16 A_01 = 64.0/FSAMP;
volatile INT16 B_00 = 64.0/FSAMP;

// Accelerometer variance (10.6) = 22*ACCEL_SCALE*g
volatile INT16 Sz = 22*ACCEL_SCALE*gravity*64.0;

// Gyro variance (10.6) = 96E-6
volatile INT16 Sw_00 = 1;

// Output (10.6)
volatile INT16 x_00 = 0;
volatile INT16 x_10= 0;

// Filter vars (all 10.6)
volatile INT16 inn_00 = 0, s_00 = 0, AP_00 = 0, AP_01 = 0, AP_10 = 0, AP_11 = 0, K_00 = 0, K_10 = 0;
volatile INT16 to_send[7] = {0};

void do_kalman(INT16 gRead, INT16 tilt)
{
    // Update the state estimate by extrapolating current state estimate with input u.
    // x = A * x + B * u
    x_00 = x_00 + ((INT32)((INT32)A_01 * (INT32)x_10))/64 + ((INT32)((INT32)B_00 * (INT32)gRead))/64;
    to_send[0] = x_00;

    // Compute the innovation -- error between measured value and state.
    // inn = y - c * x
    inn_00 = tilt - x_00;
    to_send[1] = inn_00;    

    // Compute the covariance of the innovation.
    // s = C * P * C' + Sz
    s_00 = P_00 + Sz;
    to_send[2] = s_00;

    // Compute AP matrix for use below.
    // AP = A * P
    AP_00 = P_00 + ((INT32)((INT32)A_01 * (INT32)P_10))/64;
    AP_01 = P_01 + ((INT32)((INT32)A_01 * (INT32)P_11))/64;
    AP_10 = P_10;
    AP_11 = P_11;
    to_send[3] = AP_00;
    
    // Compute the kalman gain matrix.
    // K = A * P * C' * inv(s)
    K_00 = ((INT32)((INT32)AP_00*64)) / s_00;
    K_10 = ((INT32)((INT32)AP_10*64)) / s_00;
    to_send[4] = K_00;

    // Update the state estimate
    // x = x + K * inn
    x_00 = x_00 + ((INT32)((INT32)K_00 * (INT32)inn_00))/64;
    x_10 = x_10 + ((INT32)((INT32)K_10 * (INT32)inn_00))/64;
    to_send[5] = x_00;

    // Compute the new covariance of the estimation error
    // P = A * P * A' - K * C * P * A' + Sw
    P_00 = AP_00 + ((INT32)((INT32)AP_01 * (INT32)A_01) - (INT32)((INT32)K_00 * (INT32)P_00))/64 + (INT32)(((INT32)((INT32)K_00 * (INT32)P_01)/64) * (INT32)A_01)/64 + Sw_00;
    P_01 = AP_01 - (INT32)((INT32)K_00 * (INT32)P_01)/64;
    P_10 = AP_10 + ((INT32)((INT32)AP_11 * (INT32)A_01) - (INT32)((INT32)K_10 * (INT32)P_00))/64 + (INT32)(((INT32)((INT32)K_10 * (INT32)P_01)/64) * (INT32)A_01)/64;
    P_11 = AP_11 - (INT32)((INT32)K_10 * (INT32)P_01)/64;
}

void _ISR _NOPSV _T1Interrupt(void) //Running at 10Hz
{
    OUTER_LED = ON;
    tiltX = (INT32)(accelRead(XINCL)*TILT_MULT);
    gRead = (INT32)(gyroRead(GRATE)*GYRO_MULT);
    
    //convert from quadrants to pitch angle
    tilt = tiltX;
        
    //update filter
    do_kalman(gRead, tilt);

    newData = TRUE;
    _T1IF = 0;
}

// Main clk freq after PLL = 8291500 Hz
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
    if((addr == XINCL) || (addr == YINCL))
        accelVal = SIGNED12(dataOut);
    else
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

	_U1RXIE = 0;    		    // TX Interrupt Disable
	U1STAbits.URXISEL=0x00;     // Interrupt after one byte
	_U1RXIF=0;

	U1MODEbits.UARTEN = 1;	    // Enable UART
	U1STAbits.UTXEN = 1;	    // Enable UART Tx
}    

void init_timer1()
{
    // Initialize and enable Timer1 for Control Loop at 50 Hz
    T1CONbits.TON = 1;          // Enable Timer
    T1CONbits.TCS = 0;          // Select internal instruction cycle clock
    T1CONbits.TGATE = 0;        // Disable Gated Timer mode
    T1CONbits.TCKPS = 0b11;     // Select 1:256 Prescaler
    TMR1 = 0x00;                // Clear timer register
    PR1 = 3238;                // Load the period value for internal osc

    IPC0bits.T1IP = 0x01;       // Set Timer 1 Interrupt Priority Level
    IFS0bits.T1IF = 0;          // Clear Timer 1 Interrupt Flag
    IEC0bits.T1IE = 1;          // Timer 1 interrupt Enable
    T1CONbits.TON = 0;          // Timer Start Enable
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
    init_timer1();
            
    TX_string("Wait\r\n");
    delay(1000);
    TX_string("Start\r\n");
        
    T1CONbits.TON = 1; 
    
    //remove first trash reading
    tiltX = accelRead(XINCL);
    gRead = gyroRead(GRATE);    
    while(1)
    {
        if( newData == TRUE)
        {
            TX_snum5(gRead);
            TX('\t');
            TX_snum5(tilt);
            TX('\t');
            TX_snum5(x_00);
            TX_string("\r\n");
           
            OUTER_LED = OFF;            
            newData = FALSE;
        }
    }  
    return 0;      
};

