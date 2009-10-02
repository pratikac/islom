#include "imu.h"

volatile INT16  xAccl, yAccl, gRate;
volatile bool   printPending = 0;
volatile INT32  rateOffset = 0;
volatile bool   offsetPending = 1;
volatile bool   newData;
volatile INT32  gyroInteg = 0;       //
volatile UINT16 currIdx=0;
volatile INT32  rateInteg;
volatile INT32  angleEstimate, prevAngleEstimate, inclEstimate, prevInclEstimate, rateEstimate, prevRateEstimate;
volatile INT32  filteredIncl, filteredGyro;
volatile INT32  angleEnc;
volatile INT16  prevEnc, diffEnc, currEnc;

unsigned char axis;
signed int dataOut;
signed int accVal[3] = {0};

void _ISR _NOPSV _T1Interrupt(void) //Running at 100Hz
{
    OUTER_LED = ON;
    xAccl = accelRead(XACCL);
    yAccl = accelRead(YACCL);
    newData = TRUE;

    printPending = TRUE;

    OUTER_LED = OFF;
    _T1IF = 0;
}


int main()
{
    delay(100);
    TRISA = 0;          // For the LEDs to work before systemInit()
    INNER_LED = ON;
    OUTER_LED = ON;
    systemInit();
    delay(50);
    INNER_LED = OFF;
    TX_string("\r\nPeriherals Configured...\t[ OK ]\r\n");    
    OUTER_LED = OFF;

    delay(30);

    while(!INTERRUPT_SW_PRESSED);
    while(INTERRUPT_SW_PRESSED);
    
    TX_string("Starting Timer. Waiting for RXI to initialize...\r\n");
    
    T1CONbits.TON = 1;
    
    while(!RXI());
    
    delay(30);
    T1CONbits.TON = 1;
    
    while(1)
    {
        if(printPending)
        {
            //TX_snum5(((angleEstimate/256)*20)/273); TX_string("\r\n");
            //TX_num10(inclEstimate); TX_string("\r\n");
            TX_snum5(xAccl); TX('\t'); TX_snum5(yAccl); TX_string("\r\n");
            printPending=FALSE;
        }
    }
    return 0;
}

void systemInit()
{
    //PLL Init

    PLLFBD = M - 2;
    CLKDIVbits.PLLPOST = N1 - 2;	
    CLKDIVbits.PLLPRE = N2/2 - 1;
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(0x01);
    // Wait for Clock switch to occur
    while(OSCCONbits.COSC != 0b001);
    while(OSCCONbits.LOCK!=1) {}; // Wait for PLL to lock


    /******************************  UART Init  ************************************/

    // UART 1
    U1BRG = BRGVAL;

    U1MODEbits.BRGH = 0;	    // Low speed mode
    U1MODEbits.STSEL = 0;	    // 1-stop bit
    U1MODEbits.PDSEL = 0;	    // No Parity, 8-data bits
    U1MODEbits.ABAUD = 0;	    // Autobaud Disabled

    _U1RXIE = 0; 			    // RX Interrupt Enable
    U1STAbits.URXISEL=0x00;     // Interrupt after one byte
    _U1RXIF=0;

    U1MODEbits.UARTEN = 1;	    // Enable UART
    U1STAbits.UTXEN = 1;	    // Enable UART Tx
    
    // UART 2
    U2BRG = BRGVAL;

    U2MODEbits.BRGH = 0;	    // Low speed mode
    U2MODEbits.STSEL = 0;	    // 1-stop bit
    U2MODEbits.PDSEL = 0;	    // No Parity, 8-data bits
    U2MODEbits.ABAUD = 0;	    // Autobaud Disabled

    _U2RXIE = 0; 			    // RX Interrupt Enable
    U2STAbits.URXISEL=0x00;     // Interrupt after one byte
    _U2RXIF=0;

    U2MODEbits.UARTEN = 1;	    // Enable UART
    U2STAbits.UTXEN = 1;	    // Enable UART Tx



    /**********************  Assigning Peripheral pins  *****************************/

    __builtin_write_OSCCONL(OSCCON & ~BIT(6));	    

    //Uart through PICkit
    RPINR18bits.U1RXR = 11;     // U1RX
    _RP10R = 3;                 // U1TX;
    
    // SPI Pin Config
    RPINR20bits.SDI1R = 3;      // RP3 mapped to SDI
    _RP16R = 0b00111;           // RP16 mapped to SDO
    _RP2R = 0b01000;            // RP2 mapped to SCK
 
  
    __builtin_write_OSCCONL(OSCCON | BIT(6));


    //    TX_string("PLL Setup...\t\t\t[ OK ]\r\nUART Initialized...\t\t[ OK ]\r\nPeripheral pins assigned...\t[ OK ]\r\n");

    //    TX_string("Initializing I/O Pins...\t");

    /******************* Initialize I/O Pins***************************************/

    TRISA = 0;
    TRISB = BIT(0)|BIT(1)|BIT(3)|BIT(7)|BIT(12);
    TRISC = 0;

    //    TX_string("[ OK ]\r\n");


    //    TX_string("Initializing SPI...\t\t");

    // SPI Init
    SPI1CON1bits.DISSCK = 0;    // Internal Serial Clock is Enabled.
    SPI1CON1bits.DISSDO = 0;    // SDOx pin is controlled by the module.
    SPI1CON1bits.MODE16 = 1;    // Communication is byte-wide (8 bits).
    SPI1CON1bits.SMP = 0;       // Input Data is sampled at the middle of data output time.
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

    //    TX_string("[ OK ]\r\n");


    //    TX_string("Initializing Timers...\t\t");

    // Initialize and enable Timer1 for Control Loop at 100Hz
    T1CONbits.TON = 1;          // Enable Timer
    T1CONbits.TCS = 0;          // Select internal instruction cycle clock
    T1CONbits.TGATE = 0;        // Disable Gated Timer mode
    T1CONbits.TCKPS = 0b11;     // Select 1:256 Prescaler
    TMR1 = 0x00;                // Clear timer register
    PR1 = 1547;                 // Load the period value for internal osc

    IPC0bits.T1IP = 0x01;       // Set Timer 1 Interrupt Priority Level
    IFS0bits.T1IF = 0;          // Clear Timer 1 Interrupt Flag
    IEC0bits.T1IE = 1;          // Timer 1 interrupt Enable
    T1CONbits.TON = 0;          // Timer Start Enable

    //    TX_string("[ OK ]\r\n");

    
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
    
    
    return;
}

unsigned int spiWriteByte(unsigned int inData){

    unsigned int outData;
    while (SPI1STATbits.SPIRBF) {
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
    signed int accelVal;

    ACCEL = SEL;
    dataOut = (spiXfer(addr) & 0x3FFF);
    ACCEL = DESEL;
    delay_us(5);
    ACCEL = SEL;
    dataOut = (spiXfer(0x0000) & 0x3FFF);
    ACCEL = DESEL;
    accelVal = SIGNED14(dataOut);
    delay_us(5);

    return accelVal;
}
