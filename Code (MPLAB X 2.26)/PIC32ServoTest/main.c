/* PIC32 Model PIC32MX250F128B */
#include <p32xxxx.h>    // pic32 library functions
#include <plib.h>       // peripheral library functions

// Configuration Bits
#pragma config POSCMOD      = HS            // Primary oscillator using high speed crystal mode
// #pragma config FNOSC     = FRCPLL        // Internal Fast RC oscillator
#pragma config FNOSC        = PRIPLL        // Internal Fast RC oscillator (4 MHz) w/ PLL
#pragma config FPLLIDIV     = DIV_1         // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL      = MUL_20        // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV     = DIV_2         // Divide After PLL (now 40 MHz)
#pragma config FPBDIV       = DIV_2         // Divide core clock by 2 for peripheral bus (=20MHz Fpb)
#pragma config FWDTEN       = OFF           // Watchdog Timer Disabled
#pragma config ICESEL       = ICS_PGx1      // ICE/ICD Comm Channel Select
#pragma config JTAGEN       = OFF           // Disable JTAG
#pragma config FSOSCEN      = OFF           // Disable Secondary Oscillator

// main clock at 40MHz
#define SYS_FREQ    ( 40000000L )

// Common ADC setup definitions
#define ADC_CONFIG1 ( ADC_MODULE_ON | ADC_FORMAT_INTG32 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON )
#define ADC_CONFIG2 ( ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_1 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF )
#define ADC_CONFIG3 ( ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_15 )
// We're using AN5 for our analog input
#define ADC_CONFIGPORT ( ENABLE_AN5_ANA )
// skip all channels except the one we've enabled (AN5) for efficiency
#define ADC_CONFIGSCAN (                                \
    SKIP_SCAN_AN0 | SKIP_SCAN_AN1 | SKIP_SCAN_AN2 |     \
    SKIP_SCAN_AN3 | SKIP_SCAN_AN4 |                     \
    SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8 |     \
    SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN11 |   \
    SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14    \
    )

// PWM limits
#define PWM_MIN     2400
#define PWM_MAX     5800

volatile int pwmOut = PWM_MIN;
volatile int count;
int aiScaled = 0;
int posDelta = 0;
int mode = 0;
int countDir = 1;
int pbState = 0;
int dbCtr = 0;

// Timer 2 interrupt handler (4ms period)
void __ISR ( _TIMER_2_VECTOR, ipl7 ) TMR2IntHandler( void ) {
    // 5 interrupts @ 4ms = 20 ms update period between pulses

    // this ensures we only bring PWM output high once per update period
    SetDCOC1PWM( 0 );

    // counts number of interrupts since last output update
    if ( ++count >= 5 ) {
        count = 0;

        // if mode is 0 (continuous cycle -180 to 180), adjust position according to position delta
        // which is set by POT position
        if ( mode == 1 ) {
            if ( countDir ) {
                pwmOut += posDelta;
            } else {
                pwmOut -= posDelta;
            }
        // if mode is 1 (manual position update with POT), adjust position according to POT position
        } else {
            pwmOut = aiScaled;
        }
        
        // Bounds checking before we update OC1
        if ( pwmOut > PWM_MAX ) {
            pwmOut = PWM_MAX;
            countDir = 0;
        }

        if ( pwmOut < PWM_MIN ) {
            pwmOut = PWM_MIN;
            countDir = 1;
        }

        // Set OC1 
        SetDCOC1PWM( pwmOut );
    }

    mT2ClearIntFlag();
}

// initialize ADC10 peripheral
void I_InitADC( void ) {
    CloseADC10();
    // Use ground as negative reference for analog
    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF );
    OpenADC10( ADC_CONFIG1, ADC_CONFIG2, ADC_CONFIG3, ADC_CONFIGPORT, ADC_CONFIGSCAN );
    EnableADC10();
    while ( ! mAD1GetIntFlag() ) {}
}

int main() {
    // the usual optimization macro
    SYSTEMConfig( SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE );
    INTEnableSystemMultiVectoredInt();

    I_InitADC();    
    PPSOutput( 1, RPB7, OC1 );  // set OC1 to pin RB7 with peripheral pin select
    ANSELBbits.ANSB15 = 0;      // set RB15 to digital I/O mode
    TRISBbits.TRISB15 = 1;      // set RB15 as input
    TRISBbits.TRISB5 = 0;       // set RB5 as output
    // start with mode indicator LED output OFF
    LATBCLR = 0x0020;                           

    // Configure standard PWM mode for output compare module 1
    OpenOC1( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0 );
    CloseTimer2();

   /* Quick note & example of calculating required Timer period */
   // fire interrupt every 4 ms...update PWM every 5th interrupt for 20ms cycle time
   // We want to find required PR2 for given delay time...
   // x_ms = (PR2 + 1) * TMR_PS / Fpb ->
        // 4_ms = (PR2 + 1) * 8 / 20000000
        // -> PR2 = ( .004 * 20000000 / 8 ) - 1
        // -> PR2 = 9999

   // Set timer on, prescaler = 1:8, PR2 = 9999
   OpenTimer2( T2_ON | T2_PS_1_8 | T2_SOURCE_INT, 9999 );
   ConfigIntTimer2( T2_INT_ON | T2_INT_PRIOR_7 );
   mT2SetIntPriority( 7 );      // set timer2 int priority
   mT2ClearIntFlag();           // clear interrupt flag before startup
   mT2IntEnable( 1 );           // enable timer2 interrupts

    // Forever alone...
    while ( 1 ) {      

        // ... check the push button ...
        // starting with pbState off...start debounce
        if ( !pbState && !PORTBbits.RB15 ) {
            if ( dbCtr++ == 1000 ) {
                dbCtr = 1000;
                pbState = 1;
                // toggle mode state every time push button is pressed
                mode = mode ^ 0x01;
            }
        }

        // resets pushbutton state to ensure clean transition between presses
        if ( pbState && PORTBbits.RB15 ) {
            if ( dbCtr-- == 0 ) {
                pbState = 0;
            }
        }       

        // update mode indicator LED
        if ( mode == 1 ) {
              LATBSET = 0x0020;
        } else {
              LATBCLR = 0x0020;
        }

        // Disable T2 interrupts while reading from analog input       
        mT2IntEnable( 0 );

        /* scale raw counts of AI (0-1023) to PWM output scale */
        // m = (5800-1800)/(1023-0) = 3.91
        // x = raw counts (0-1023) from 10bit ADC
        // b = 1800
        aiScaled = ReadADC10( 0 ) * 3.91 + 1800;

        /* From our scaled analog input, now calculate an appropriate position delta to use for mode 1 */
        // aiScaled range = 2400 - 5800
        // min pos Delta per tic = 15 -> @ 0% POT (2400), 10/2400 ~ 0.004
        posDelta = aiScaled * 0.0125;
        mT2IntEnable( 1 );    
    }

    return ( EXIT_SUCCESS );
}