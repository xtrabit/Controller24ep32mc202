/*
 * File:   main5.c
 * Author: lexa
 *
 * Created on July 7, 2018, 12:08 AM
 */
#include <xc.h>
#include <stdint.h>
#include <p24EP32MC202.h>
//#include <stdbool.h>
//#include <stdlib.h>

// FICD
#pragma config ICS = PGD1    // ICD Communication Channel Select bits->Communicate on PGEC1 and PGED1
#pragma config JTAGEN = OFF    // JTAG Enable bit->JTAG is disabled

// FPOR
#pragma config ALTI2C1 = ON    // Alternate I2C1 pins->I2C1 mapped to ASDA1/ASCL1 pins
#pragma config ALTI2C2 = ON    // Alternate I2C2 pins->I2C2 mapped to ASDA2/ASCL2 pins
#pragma config WDTWIN = WIN25    // Watchdog Window Select bits->WDT Window is 25% of WDT period

// FWDT
#pragma config WDTPOST = PS32768    // Watchdog Timer Postscaler bits->1:32768
#pragma config WDTPRE = PR128    // Watchdog Timer Prescaler bit->1:128
#pragma config PLLKEN = ON    // PLL Lock Enable bit->Clock switch to PLL source will wait until the PLL lock signal is valid.
#pragma config WINDIS = OFF    // Watchdog Timer Window Enable bit->Watchdog Timer in Non-Window mode
#pragma config FWDTEN = OFF    // Watchdog Timer Enable bit->Watchdog timer enabled/disabled by user software

// FOSC
#pragma config POSCMD = NONE    // Primary Oscillator Mode Select bits->Primary Oscillator disabled
#pragma config OSCIOFNC = ON    // OSC2 Pin Function bit->OSC2 is general purpose digital I/O pin
#pragma config IOL1WAY = ON    // Peripheral pin select configuration->Allow only one reconfiguration
#pragma config FCKSM = CSECMD    // Clock Switching Mode bits->Clock switching is enabled,Fail-safe Clock Monitor is disabled

// FOSCSEL
#pragma config FNOSC = FRC    // Oscillator Source Selection->FRC
#pragma config PWMLOCK = OFF    // PWM Lock Enable bit->PWM registers may be written without key sequence
#pragma config IESO = ON    // Two-speed Oscillator Start-up Enable bit->Start up device with FRC, then switch to user-selected oscillator source

// FGS
#pragma config GWRP = OFF    // General Segment Write-Protect bit->General Segment may be written
#pragma config GCP = OFF    // General Segment Code-Protect bit->General Segment Code protect is Disabled

#define _XTAL_FREQ  63566250UL

#define LED_G LATAbits.LATA3
#define LED_R LATAbits.LATA4
#define MOTOR_L PDC1
#define MOTOR_R PDC2

void Display (unsigned int x);
void Display2 (unsigned int x, unsigned int y);
void Display3 (unsigned int x, unsigned int y, unsigned int z);
void Display4 (unsigned int x, unsigned int y, unsigned int z, unsigned int w);
void Gyro_LEDS (void);
int RCvalue_cond (int x);
void Motor_OUT(void);
void Signal_MIX(void);
void Test_MIN_MAX(unsigned int x);
void Buttons_thro(void);
void Display_buttons(void);

int count_t1;
char spi_digit1;
char spi_digit2;
char spi_digit3;
char spi_digit4;
int adc_gyro;
int adc_gyro0;
int adc_batt;
char gyro_deadband;
unsigned int aile_buf;
unsigned int elev_buf;
unsigned int thro_buf;
unsigned int gear_buf;
int temp;
int IC1BUF_temp;
int IC2BUF_temp;
int IC3BUF_temp;
int IC4BUF_temp;
int IC1BUF_temp1;
int IC2BUF_temp1;
int IC3BUF_temp1;
int IC4BUF_temp1;
unsigned int RCpulse_min;
unsigned int RCpulse_cen;
unsigned int RCpulse_max;
int RCswing;
int RCvalue;
int RCvalue_min;
int RCvalue_max;
int RCvalue_DB;
char SIGNAL_AILE;
char SIGNAL_ELEV;
char SIGNAL_THRO;
char SIGNAL_GEAR;
char SIGNAL_CHECK;
int aile_value;
int elev_value;
int thro_value;
int gear_value;
unsigned int PDC_cen;
int left_value;
int right_value;
char display_digit;
int test_min;
int test_max;
//char test_reset;
char sw_extend;
char sw_ontime;
char bt_flip;
char bt_extend;
char bt_retract;
//char thro_bt;
//char thro_bt_prev;
//char thro_bt_prev2;
//char thro_bt_prev3;
//char display_buttons;
//char buttons_update;
char sw_extend_old;
char sw_ontime_old;
char bt_flip_old;
char bt_extend_old;
char bt_retract_old;

// buttons new way
char bt_new_value;
char bt_change_dir;
char bt_change_dir_old;
unsigned int thro_buf_old;
unsigned int bt_min;
unsigned int bt_max;
int bt_timer;
void BT_new_way(void);
void Buttons(unsigned int x);
char wait;

int gyro_value;
void Test_Gyro_MIN_MAX(void);

void Test_spin(void);
char test_spin_on;

unsigned int test_timer_lsb;
unsigned int test_timer_msb;
unsigned long test_timer;
unsigned int test_timer_seconds;

unsigned int t1;
unsigned int t2;
unsigned int t3;
unsigned int t4;
unsigned int tt;

void STOP(void);
int wait_count;
void Test_spin2(void);
unsigned int test_timer2_lsb;
unsigned int test_timer2_msb;
unsigned long test_timer2;
unsigned int test_timer2_seconds;
unsigned int tt2;

//180 degree turn
unsigned int uturn_counter;
void Uturn(void);
int gyro[10];
char gyro_inc;
char gyro_update;
char gyro_break;
long turn_deg;
long turn_deg_10;
long turn_deg_100;
char turn_dir;
unsigned int brake_estimate;
void angle_tracker(void);
int gyro_value_ave;
int hextobcd (int x);
int y;
char final;



void __attribute__ ( ( interrupt, no_auto_psv ) ) _ISR _IC1Interrupt( void )
    {
        if(PORTBbits.RB5 == 0) 
        {
            IC1BUF_temp1 = IC1BUF;
            gear_buf = IC1BUF_temp1 - IC1BUF_temp;
            SIGNAL_GEAR = 1;
        }
        else
        {
            IC1BUF_temp = IC1BUF;
        }
        while (IC1CON1bits.ICBNE == 1)
        {
            temp = IC1BUF;
        }
        if (gear_buf > 0xef00 || gear_buf < 0x8620 )
        {
            IC1CON1bits.ICM = 0;
            Nop();
            Nop();
            Nop();
            Nop();
            Nop();
            IC1CON1bits.ICM = 1;
            SIGNAL_GEAR = 0;
            gear_buf = 0xeb00;
        }
        IFS0bits.IC1IF = 0;
    }

void __attribute__ ( ( interrupt, no_auto_psv ) ) _ISR _IC2Interrupt( void )
    {
        //if (IC2CON1bits.ICBNE == 1)
        //{
            if(PORTBbits.RB4 == 0)
            {
                IC2BUF_temp1 = IC2BUF;
                elev_buf = IC2BUF_temp1 - IC2BUF_temp;
                SIGNAL_ELEV = 1;
            }
            else
            {
                IC2BUF_temp = IC2BUF;
            }
        //}
        while (IC2CON1bits.ICBNE == 1)
        {
            temp = IC2BUF;
        }
        if (elev_buf > 0xef00 || elev_buf < 0x8620 )
        {
            IC2CON1bits.ICM = 0;
            Nop();
            Nop();
            Nop();
            Nop();
            Nop();
            IC2CON1bits.ICM = 1;
            SIGNAL_ELEV = 0;
            elev_buf = 0xba90;
        }
        IFS0bits.IC2IF = 0;
    }

void __attribute__ ( ( interrupt, no_auto_psv ) ) _ISR _IC3Interrupt( void )
    {
        //if (IC3CON1bits.ICBNE == 1)
        //{
            if(PORTBbits.RB10 == 0)
            {
                IC3BUF_temp1 = IC3BUF;
                aile_buf = IC3BUF_temp1 - IC3BUF_temp;
                SIGNAL_AILE = 1;
            }
            else
            {
                IC3BUF_temp = IC3BUF;
                
            }
        //}
        while (IC3CON1bits.ICBNE == 1)
        {
            temp = IC3BUF;
        }
        if (aile_buf > 0xef00 || aile_buf < 0x8620 )
        {
            IC3CON1bits.ICM = 0;
            Nop();
            Nop();
            Nop();
            Nop();
            Nop();
            IC3CON1bits.ICM = 1;
            SIGNAL_AILE = 0;
            aile_buf = 0xba90;
        }
            
        IFS2bits.IC3IF = 0;
    }

void __attribute__ ( ( interrupt, no_auto_psv ) ) _ISR _IC4Interrupt( void )
    {
        if(PORTBbits.RB6 == 0)
        {
            IC4BUF_temp1 = IC4BUF;
            thro_buf_old = thro_buf;
            thro_buf = IC4BUF_temp1 - IC4BUF_temp;
            SIGNAL_THRO = 1;
        }
        else
        {
            IC4BUF_temp = IC4BUF;
        }
        while (IC4CON1bits.ICBNE == 1)
        {
            temp = IC4BUF;
        }
        if (thro_buf > 0xef00 || thro_buf < 0x8620 )
        {
            IC4CON1bits.ICM = 0;
            Nop();
            Nop();
            Nop();
            Nop();
            Nop();
            IC4CON1bits.ICM = 1;
            SIGNAL_THRO = 0;
            thro_buf = 0x8c66;
        }
        bt_new_value = 1;
        IFS2bits.IC4IF = 0;
    }

void __attribute__ ( ( __interrupt__ , auto_psv ) ) _AD1Interrupt ( void )
    {
        adc_gyro0 = ADC1BUF0 - 7;
        adc_gyro = ADC1BUF1;
        gyro_value = adc_gyro - adc_gyro0;
        if (gyro_value > -gyro_deadband && gyro_value < gyro_deadband) gyro_value = 0;
            
        IFS0bits.AD1IF = 0;
    }

void __attribute__ ( ( interrupt, no_auto_psv ) ) _T1Interrupt (  )             // SPI output timer
    {
    if (count_t1 < 8)
    {
        count_t1++;
    }
    else
    {
        count_t1 = 0;
        if (display_digit < 1) display_digit++;
        else display_digit = 0;
    }
        SPI1BUF = spi_digit1;
        SPI1BUF = spi_digit2;
        SPI1BUF = spi_digit3;
        SPI1BUF = spi_digit4;
        
        if (wait_count < 16 )
        {
            if (wait != 0) wait_count++;
        }
        else
        {
            wait_count = 0;
            wait = 0;
        }
        IFS0bits.T1IF = 0;
    }

void __attribute__ ( ( interrupt, no_auto_psv ) ) _T2Interrupt (  )             // loss of signal output kill control timer
    {
    
    if (SIGNAL_AILE == 1) SIGNAL_AILE = 2;
    else SIGNAL_AILE = 0;
    
    if (SIGNAL_ELEV == 1) SIGNAL_ELEV = 2;
    else SIGNAL_ELEV = 0;
    
    if (SIGNAL_THRO == 1) SIGNAL_THRO = 2;
    else SIGNAL_THRO = 0;
    
    if (SIGNAL_GEAR == 1) SIGNAL_GEAR = 2;
    else SIGNAL_GEAR = 0;
    
    if (SIGNAL_AILE == 0 && SIGNAL_ELEV == 0)
    {
        MOTOR_L = PDC_cen;
        MOTOR_R = PDC_cen;
        SIGNAL_CHECK = 0;
        test_spin_on = 0;
    }
    else SIGNAL_CHECK = 1;
    
    _T2IF = 0;
    }

void __attribute__ ( ( interrupt, no_auto_psv ) ) _T3Interrupt (  )             // debouncing buttons
    {
    wait = 1;
    T3CONbits.TON = 0;
    _T3IF = 0;
    }

void __attribute__ ( ( interrupt, no_auto_psv ) ) _T4Interrupt (  )             // test timer
    {
    _T4IF = 0;
    if (gyro_inc < 9)
    {
        gyro[gyro_inc] = gyro_value;
        gyro_inc ++;
    }
    else
    {
        //if (LED_G == 1) LED_G = 0;
        //else LED_G = 1;
        //LED_R = 1;
        gyro[gyro_inc] = gyro_value;
        gyro_value_ave = ((long)gyro[0]+gyro[1]+gyro[2]+gyro[3]+gyro[4]+gyro[5]+gyro[6]+gyro[7]+gyro[8]+gyro[9])/10;// 19 us
        turn_deg_100 = (long)gyro_value_ave * 1465 / 1000;// together with above 38 us; _10==1465/10000
        if (turn_deg_100 > -gyro_deadband && turn_deg_100 < gyro_deadband) turn_deg_100 = 0;
        //LED_R = 0;
        gyro_inc = 0;
        gyro_update = 1;
    }
    //LED_G = 1;
    //test_timer_lsb = TMR2;
    //teat_timer_msb = TMR3HLD;
    
    //T4CONbits.TON = 0;
    //_T5IF = 0;
    //TMR5= 0;
    //TMR4= 0;
    
    //LED_G = 0;
    }

int main(void)
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      pins
    
    /****************************************************************************
     * Setting the Output Latch SFR(s)
     ***************************************************************************/
    LATA = 0x0000;
    LATB = 0x0000;
    /****************************************************************************
     * Setting the GPIO Direction SFR(s)
     ***************************************************************************/
    TRISA = 0x0003;
    TRISB = 0x047D;
    /****************************************************************************
     * Setting the Weak Pull Up and Weak Pull Down SFR(s)
     ***************************************************************************/
    CNPDA = 0x0000;
    CNPDB = 0x0470;
    CNPUA = 0x0000;
    CNPUB = 0x0000;
    /****************************************************************************
     * Setting the Open Drain SFR(s)
     ***************************************************************************/
    ODCA = 0x0000;
    ODCB = 0x0000;
    /****************************************************************************
     * Setting the Analog/Digital Configuration SFR(s)
     ***************************************************************************/
    ANSELA = 0x0003;
    ANSELB = 0x0001;
    /****************************************************************************
     * Set the PPS
     ***************************************************************************/
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPINR8bits.IC4R = 0x0026;   //RB6->IC4:IC4;
    RPINR8bits.IC3R = 0x002A;   //RB10->IC3:IC3;
    RPINR7bits.IC1R = 0x0025;   //RB5->IC1:IC1;
    RPINR7bits.IC2R = 0x0024;   //RB4->IC2:IC2;
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    
    
   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      oscillator 
    
    
    CLKDIV = 0x3040; // FRCDIV FRC/1; PLLPRE 2; DOZE 1:8; PLLPOST 1:4; DOZEN disabled; ROI disabled; 
    OSCTUN = 0x0000; // TUN Center frequency; 
    REFOCON = 0x0000; // ROON disabled; ROSEL disabled; RODIV Base clock value; ROSSLP disabled; 
    PLLFBD = 0x0043; //0x0043 0x001f lowest // PLLDIV 67; 
	CORCONbits.RND = 0;// RND disabled; SATB disabled; SATA disabled; ACCSAT disabled; 
	CORCONbits.SATB = 0;
	CORCONbits.SATA = 0;
	CORCONbits.ACCSAT = 0;
    // CF no clock failure; NOSC FRCPLL; CLKLOCK unlocked; OSWEN Switch is Complete; 
    __builtin_write_OSCCONH((uint8_t) ((0x0100 >> _OSCCON_NOSC_POSITION) & 0x00FF));
    __builtin_write_OSCCONL((uint8_t) ((0x0100 | _OSCCON_OSWEN_MASK) & 0xFF));
    
    while (OSCCONbits.OSWEN != 0);  // Wait for Clock switch to occur
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      interrupt
     
    IPC3bits.AD1IP = 1; //    ADI: ADC1 Convert Done; Priority: 1
    IPC0bits.IC1IP = 1; //    ICI_INT: Input Compare 1; Priority: 1
    IPC9bits.IC3IP = 1; //    ICI_INT: Input Capture 3; Priority: 1
    IPC1bits.IC2IP = 1; //    ICI_INT: Input Capture 2; Priority: 1   
    IPC9bits.IC4IP = 1; //    ICI_INT: Input Capture 4; Priority: 1
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      IC1
        
    // ICSIDL disabled; ICM Edge Detect Capture; ICTSEL FOSC/2; ICI Every; 
    IC1CON1 = 0x1C01;
    // SYNCSEL None; TRIGSTAT disabled; IC32 disabled; ICTRIG Sync; 
    IC1CON2 = 0x0000;
    IFS0bits.IC1IF = 0;
    IEC0bits.IC1IE = 1;
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      IC2    
    
    // ICSIDL disabled; ICM Edge Detect Capture; ICTSEL FOSC/2; ICI Every; 
    IC2CON1 = 0x1C01;
    // SYNCSEL None; TRIGSTAT disabled; IC32 disabled; ICTRIG Sync; 
    IC2CON2 = 0x0000;
    IFS0bits.IC2IF = 0;
    IEC0bits.IC2IE = 1;
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      IC3
    
    // ICSIDL disabled; ICM Edge Detect Capture; ICTSEL FOSC/2; ICI Every; 
    IC3CON1 = 0x1C01;
    // SYNCSEL None; TRIGSTAT disabled; IC32 disabled; ICTRIG Sync; 
    IC3CON2 = 0x0000;
    IFS2bits.IC3IF = 0;
    IEC2bits.IC3IE = 1;
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      IC4    
        
    // ICSIDL disabled; ICM Edge Detect Capture; ICTSEL FOSC/2; ICI Every; 
    IC4CON1 = 0x1C01; // SYNCSEL None; TRIGSTAT disabled; IC32 disabled; ICTRIG Sync; 
    IC4CON2 = 0x0000; //IC4CON1bits.ICTSEL = 1; //timer2 == 1;
    IFS2bits.IC4IF = 0;
    IEC2bits.IC4IE = 1; //IC4CON1bits.ICM = 1;
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      SPI    
    
    // MSTEN Master; DISSDO disabled; PPRE 4:1; SPRE 8:1; MODE16 disabled; SMP Middle; DISSCK disabled; 
    //CKP Idle:Low, Active:High; CKE Idle to Active; SSEN disabled; 
    SPI1CON1 = 0x0038;//was 0x0022; 0x0038 2us pulse
    // SPIFSD disabled; SPIBEN enabled; FRMPOL disabled; FRMDLY disabled; FRMEN disabled; 
    SPI1CON2 = 0x0001; //0x0001
    // SISEL SPI_INT_TRMT_COMPLETE; SPIROV disabled; SPIEN enabled; SPISIDL disabled; 
    SPI1STAT = 0x8014;
    //_FRMPOL = 0;
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      ADC
    
    // ASAM enabled; ADDMABM disabled; ADSIDL disabled; DONE disabled; SIMSAM Sequential; 
    // FORM Absolute decimal result, unsigned, right-justified; SAMP disabled; 
    // SSRC Internal counter ends sampling and starts conversion; AD12B 12-bit; ADON enabled; SSRCG disabled;
    AD1CON1 = 0x84E4;
    _ADON = 0;
    // CSCNA enabled; VCFG0 AVDD; VCFG1 AVSS; ALTS disabled; BUFM disabled; 
    // SMPI Generates interrupt after completion of every sample/conversion operation; CHPS 1 Channel;
    AD1CON2 = 0x0408; //was 040c // SAMC 31; ADRC FOSC/2; ADCS 3; 
    AD1CON3 = 0x1F03; // CH0SA AN0; CH0SB AN1; CH0NB AVSS; CH0NA AVSS; 
    AD1CHS0 = 0x0100; // CSS25 disabled; CSS24 disabled; CSS31 disabled; CSS30 disabled;
    AD1CSSH = 0x0000;  // CSS2 enabled; CSS1 enabled; CSS0 enabled; CSS5 disabled; CSS4 disabled; CSS3 disabled;
    AD1CSSL = 0x0007; // DMABL Allocates 1 word of buffer to each analog input; ADDMAEN disabled;
    AD1CON4 = 0x0000; // CH123SA disabled; CH123SB CH1=OA2/AN0,CH2=AN1,CH3=AN2; CH123NA disabled; CH123NB CH1=VREF-,CH2=VREF-,CH3=VREF-; 
    AD1CHS123 = 0x0000;
    _SAMC = 31;
    
    IEC0bits.AD1IE = 1;
    //_SAMC = 3;
    //_ADCS = 5;
    _ADON = 1;
   
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      TMR3
    //  Timer 3 for debouncing buttons
    TMR3= 0;
    T3CON = 0x0000;
    PR3 = 0xf4f4;
    // PR2 = 0xf4f4; T2CONbits.TCKPS = 3; period 500ms
    // PR2 = 0xc3f6; T2CONbits.TCKPS = 2; period 100ms
    // PR2 = 0xeb27; T2CONbits.TCKPS = 1; period 15ms
    // PR2 = 0x9cc5; T2CONbits.TCKPS = 1; period 10ms
    T3CONbits.TCKPS = 3;
    _T3IF = 0;
    _T3IE = 1;
    T3CONbits.TON = 1;
    
    
    wait = 0;
    while (wait == 0)
    {
        
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      PWM
    
    // PCLKDIV 2; 
    PTCON2 = 0x0004;
    PTPER = 0xFFFF;
    SEVTCMP = 0x0000;
    MDC = 0x0000;
    CHOP = 0x0000;
    PWMKEY = 0x0000;
    // MDCS Primary; FLTIEN disabled; CAM Edge Aligned; DTC Dead-time function is disabled; 
    // TRGIEN disabled; XPRES disabled; ITB Master; IUE disabled; CLIEN disabled; MTBS disabled; DTCP disabled; 
    PWMCON1 = 0x0080;
    // MDCS Primary; FLTIEN disabled; CAM Edge Aligned; DTC Dead-time function is disabled; 
    // TRGIEN disabled; XPRES disabled; ITB Master; IUE disabled; CLIEN disabled; MTBS disabled; DTCP disabled; 
    PWMCON2 = 0x0080;
    // MDCS Primary; FLTIEN disabled; CAM Edge Aligned; DTC Dead-time function is disabled; 
    // TRGIEN disabled; XPRES disabled; ITB Master; IUE disabled; CLIEN disabled; MTBS disabled; DTCP disabled; 
    PWMCON3 = 0x0080;
    //FLTDAT PWM1L Low, PWM1H Low; SWAP disabled; OVRENH disabled; PENL enabled; PMOD Redundant Output Mode; 
    // OVRENL disabled; OSYNC disabled; POLL disabled; PENH disabled; CLDAT PWM1L Low, PWM1H Low; OVRDAT PWM1L Low, PWM1H Low; POLH disabled; 
    __builtin_write_PWMSFR(&IOCON1, 0x7400, &PWMKEY);
    //FLTDAT PWM2L Low, PWM2H Low; SWAP disabled; OVRENH disabled; PENL enabled; PMOD Redundant Output Mode; 
    // OVRENL disabled; OSYNC disabled; POLL disabled; PENH disabled; CLDAT PWM2L Low, PWM2H Low; OVRDAT PWM2L Low, PWM2H Low; POLH disabled; 
    __builtin_write_PWMSFR(&IOCON2, 0x7400, &PWMKEY);
    //FLTDAT PWM3L Low, PWM3H Low; SWAP disabled; OVRENH disabled; PENL disabled; PMOD Complementary Output Mode; 
    // OVRENL disabled; OSYNC disabled; POLL disabled; PENH disabled; CLDAT PWM3L Low, PWM3H Low; OVRDAT PWM3L Low, PWM3H Low; POLH disabled; 
    __builtin_write_PWMSFR(&IOCON3, 0x0000, &PWMKEY);
    //FLTPOL disabled; CLPOL disabled; CLSRC FLT1; CLMOD disabled; FLTMOD Fault input is disabled; IFLTMOD disabled; FLTSRC FLT32; 
    __builtin_write_PWMSFR(&FCLCON1, 0x00FB, &PWMKEY);
    //FLTPOL disabled; CLPOL disabled; CLSRC FLT1; CLMOD disabled; FLTMOD Fault input is disabled; IFLTMOD disabled; FLTSRC FLT32; 
    __builtin_write_PWMSFR(&FCLCON2, 0x00FB, &PWMKEY);
    //FLTPOL disabled; CLPOL disabled; CLSRC FLT1; CLMOD disabled; FLTMOD Fault input is disabled; IFLTMOD disabled; FLTSRC FLT32; 
    __builtin_write_PWMSFR(&FCLCON3, 0x00FB, &PWMKEY);
    PDC1 = 0x0000; // 1784
    PDC2 = 0x0000; // 1f5a
    PDC3 = 0x0000;
    PHASE1 = 0x0000;
    PHASE2 = 0x0000;
    PHASE3 = 0x0000;
    DTR1 = 0x0000;
    DTR2 = 0x0000;
    DTR3 = 0x0000;
    ALTDTR1 = 0x0000;
    ALTDTR2 = 0x0000;
    ALTDTR3 = 0x0000;
    TRIG1 = 0x0000;
    TRIG2 = 0x0000;
    TRIG3 = 0x0000;
    TRGCON1 = 0x0000;
    TRGCON2 = 0x0000;
    TRGCON3 = 0x0000;
    LEBCON1 = 0x0000;
    LEBCON2 = 0x0000;
    LEBCON3 = 0x0000;
    LEBDLY1 = 0x0000;
    LEBDLY2 = 0x0000;
    LEBDLY3 = 0x0000;
    AUXCON1 = 0x0000;
    AUXCON2 = 0x0000;
    AUXCON3 = 0x0000;
    

    // SYNCOEN disabled; SEIEN disabled; SESTAT disabled; SEVTPS 1; SYNCSRC SYNCI1;
    // SYNCEN disabled; PTSIDL disabled; PTEN enabled; EIPU disabled; SYNCPOL disabled; 
    //PTCON = 0x8000;
    _PTEN = 1;
    
   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      TMR1
   //   Timer 1 controls SPI output
    TMR1 = 0x0000;      // period is about 250 ms 
    PR1 = 0x793e;
    T1CON = 0x8030;
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
    T1CONbits.TON = 1;
	
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      TMR2
    //  Timer 2 monitors RC signal presence, if not clears SIGNAL_CHECK
    TMR2= 0;
    T2CON = 0x0000;
    PR2 = 0xc3f6;
    // PR2 = 0xf4f4; T2CONbits.TCKPS = 3; period 500ms
    // PR2 = 0xc3f6; T2CONbits.TCKPS = 2; period 100ms
    // PR2 = 0xeb27; T2CONbits.TCKPS = 1; period 15ms
    // PR2 = 0x9cc5; T2CONbits.TCKPS = 1; period 10ms
    T2CONbits.TCKPS = 2;
    _T2IF = 0;
    _T2IE = 1;
    T2CONbits.TON = 1;
    
    
    
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      TMR4
    //  Timer 4 for test timing 32 bit
    
    //TMR5= 0; //msb
    TMR4= 0; //lsb
    //T4CON = 0x0008; // 0008 - 32 bit with 5
    T4CON = 0x0000;
    //PR5 = 0xffff; //msb
    PR4 = 0xf876; //lsb; 0xf876 + tckps=0 == .002 sec period; tckps=1 == .016 sec period
    //PR5 = 0xf4; //0.5 sec
    //PR4 = 0xf40b;
    //PR5 = 0x1e9; //1 sec
    //PR4 = 0xe816;
    T4CONbits.TCKPS = 0; // 1== x8
    //_T5IF = 0;
    //_T5IE = 0;
    _T4IF = 0;
    _T4IE = 1;
    T4CONbits.TON = 1;
    
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    count_t1 = 0;
    LED_G = 0;
    LED_R = 0;
    gyro_deadband = 10;//was 23
    RCpulse_min = 0x8a20;   //35360
    RCpulse_cen = 0xba90;   //47760
    RCpulse_max = 0xeb00;   //60160
    RCswing = 0x60e0;       //24800
    RCvalue_max = 0x3070;
    RCvalue_min = -RCvalue_max;
    RCvalue_DB = 256;
    SIGNAL_AILE = 0;
    SIGNAL_ELEV = 0;
    SIGNAL_THRO = 0;
    SIGNAL_GEAR = 0;
    SIGNAL_CHECK = 0;
    PDC_cen = 0x1758;
    display_digit = 0;
    sw_extend = 0;
    sw_ontime = 0;
    bt_flip = 0;
    bt_extend = 0;
    bt_retract = 0;
    // buttons new way
    bt_new_value = 0;
    bt_change_dir = 0;
    bt_change_dir_old = 0;
    thro_buf = 0;
    thro_buf_old = 0;
    bt_timer = 0;
    bt_min = 0xffff;
    bt_max = 0x0000;
    
    aile_value = 0;
    elev_value = 0;
    test_spin_on = 0;
    test_timer_seconds = 0;
    wait_count = 0;
    uturn_counter = 0;
    gyro_inc = 0;
    gyro_update = 0;
    gyro_break = 0;
    turn_dir = 0;
    gyro_value_ave = 0;
    final = 0;
    
    
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
    
    while(1)
    {
        
        STOP();
        Uturn();
        //Signal_MIX();
        //Motor_OUT();
        
        BT_new_way();
        //Test_spin2();
        
        //angle_tracker();
        y = turn_deg/10;
        Display(hextobcd(turn_deg/10));
        //Display(hextobcd(turn_deg));
        //Display(gyro_value);
        //Test_MIN_MAX(adc_gyro);
        //Display2 (test_min, test_max);
        
        //Display2 (left_value, right_value);
        //Display2 (aile_value, elev_value);
        //Display2 (aile_buf, elev_buf);
        //Test_Gyro_MIN_MAX();
        //Display4 (-test_min, test_max, tt, tt2);
        //Display2 (-test_min, test_max);                 // 1.4 us
        //Gyro_LEDS();                        // 1 us
    }
}

void BT_new_way(void)
{
    if (bt_new_value == 1)
        {
            int step;
            step = 300;
            int timer_max;
            timer_max = 25;
            
            if (bt_timer == 0)   // if there was no change before
            {
                bt_max = 0x0000;
                bt_min = 0xffff;
                if (thro_buf_old == 0)   //check if new power-on condition or after error
                    {
                        bt_timer = 0;
                        return;
                    }
                else    // if buffer has valid value
                {
                    if ((thro_buf + step) > thro_buf_old || (thro_buf - step) < thro_buf_old)   // check for significant change
                    {
                        bt_change_dir_old = bt_change_dir;
                        if (thro_buf > thro_buf_old) bt_change_dir = 1; // assign direction
                        else bt_change_dir = -1;
                        if (bt_change_dir == 1)
                        {
                            if (thro_buf > bt_max) bt_max = thro_buf;    // if positive, set max
                        }
                        else
                        {
                            if (thro_buf < bt_min) bt_min = thro_buf;   // if negative, set min
                        }
                        bt_timer ++;
                    }
                    else
                    {
                        bt_change_dir_old = bt_change_dir;
                        bt_change_dir = 0;  // no significant change
                        bt_timer = 0;
                    }
                }
                
            }
            else    // if the change in progress
            {
                if (bt_timer < timer_max)   // not yet timeout
                {
                    bt_timer++;
                    if ((thro_buf + step) > thro_buf_old || (thro_buf - step) < thro_buf_old)   // check for significant change
                    {
                        bt_change_dir_old = bt_change_dir;
                        if (thro_buf > thro_buf_old) bt_change_dir = 1; // assign direction
                        else bt_change_dir = -1;
                        if (bt_change_dir == 1)
                        {
                            if (thro_buf > bt_max) bt_max = thro_buf;    // if positive, set max
                        }
                        else
                        {
                            if (thro_buf < bt_min) bt_min = thro_buf;   // if negative, set min
                        }
                    }
                    else
                    {
                        //bt_change_dir = bt_change_dir_old;
                        //bt_change_dir = 0;  // no significant change
                    }
                }
                else    // timeout has occurred
                {
                    bt_timer = 0;
                    //bt_max+=150;
                    //bt_min-=150;
                    if (bt_change_dir == 1) Buttons(bt_max);
                    else Buttons(bt_min);
                    //Display_buttons();
                }
            }
            bt_new_value = 0;
        }
}

void Buttons(unsigned int x)
{
    char count;
    count = 0;
    sw_extend_old = sw_extend;
    sw_ontime_old = sw_ontime;
    bt_flip_old = bt_flip;
    bt_extend_old = bt_extend;
    bt_retract_old = bt_retract;
            
    if (x < 49800)
    {
        if (x > 49284)
        {
            sw_extend = 1;
            sw_ontime = 1;
            bt_flip = 1;
            bt_extend = 0;
            bt_retract = 0;
        }
        else
        {
            if (x > 48496)
            {
                sw_extend = 1;
                sw_ontime = 0;
                bt_flip = 1;
                bt_extend = 0;
                bt_retract = 0;
            }
            else
            {
                if (x > 47637)
                {
                    sw_extend = 0;
                    sw_ontime = 1;
                    bt_flip = 1;
                    bt_extend = 0;
                    bt_retract = 0;
                }
                else
                {
                    if (x > 47072)
                    {
                        sw_extend = 0;
                        sw_ontime = 0;
                        bt_flip = 1;
                        bt_extend = 0;
                        bt_retract = 0;
                    }
                    else
                    {
                        if (x > 43224)
                        {
                            sw_extend = 1;
                            sw_ontime = 1;
                            bt_flip = 0;
                            bt_extend = 1;
                            bt_retract = 0;
                        }
                        else
                        {
                            if (x > 42629)
                            {
                                sw_extend = 1;
                                sw_ontime = 0;
                                bt_flip = 0;
                                bt_extend = 1;
                                bt_retract = 0;
                            }
                            else
                            {
                                if (x > 41871)
                                {
                                    sw_extend = 0;
                                    sw_ontime = 1;
                                    bt_flip = 0;
                                    bt_extend = 1;
                                    bt_retract = 0;
                                }
                                else
                                {
                                    if (x > 41265)
                                    {
                                        sw_extend = 0;
                                        sw_ontime = 0;
                                        bt_flip = 0;
                                        bt_extend = 1;
                                        bt_retract = 0;
                                    }
                                    else
                                    {
                                        if (x > 40551)
                                        {
                                            sw_extend = 1;
                                            sw_ontime = 1;
                                            bt_flip = 0;
                                            bt_extend = 0;
                                            bt_retract = 1;
                                        }
                                        else
                                        {
                                            if (x > 39894)
                                            {
                                                sw_extend = 1;
                                                sw_ontime = 0;
                                                bt_flip = 0;
                                                bt_extend = 0;
                                                bt_retract = 1;
                                            }
                                            else
                                            {
                                                if (x > 39129)
                                                {
                                                    sw_extend = 0;
                                                    sw_ontime = 1;
                                                    bt_flip = 0;
                                                    bt_extend = 0;
                                                    bt_retract = 1;
                                                }
                                                else
                                                {
                                                    if (x > 38470)
                                                    {
                                                        sw_extend = 0;
                                                        sw_ontime = 0;
                                                        bt_flip = 0;
                                                        bt_extend = 0;
                                                        bt_retract = 1;
                                                    }
                                                    else
                                                    {
                                                        if (x > 37746)
                                                        {
                                                            sw_extend = 1;
                                                            sw_ontime = 1;
                                                            bt_flip = 0;
                                                            bt_extend = 0;
                                                            bt_retract = 0;
                                                        }
                                                        else
                                                        {
                                                            if (x > 37027)
                                                            {
                                                                sw_extend = 1;
                                                                sw_ontime = 0;
                                                                bt_flip = 0;
                                                                bt_extend = 0;
                                                                bt_retract = 0;
                                                            }
                                                            else
                                                            {
                                                                if (x > 36241)
                                                                {
                                                                    sw_extend = 0;
                                                                    sw_ontime = 1;
                                                                    bt_flip = 0;
                                                                    bt_extend = 0;
                                                                    bt_retract = 0;
                                                                }
                                                                else
                                                                {
                                                                    if (x > 35600)
                                                                    {
                                                                        sw_extend = 0;
                                                                        sw_ontime = 0;
                                                                        bt_flip = 0;
                                                                        bt_extend = 0;
                                                                        bt_retract = 0;
                                                                    }
                                                                    else
                                                                    {
                                                                        sw_extend = 0;
                                                                        sw_ontime = 0;
                                                                        bt_flip = 0;
                                                                        bt_extend = 0;
                                                                        bt_retract = 0;
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else
    {
        sw_extend = 0;
        sw_ontime = 0;
        bt_flip = 0;
        bt_extend = 0;
        bt_retract = 0;
    }
    if (sw_extend != sw_extend_old) count++;
    if (sw_ontime != sw_ontime_old) count++;
    if (bt_flip != bt_flip_old) count++;
    if (bt_extend != bt_extend_old) count++;
    if (bt_retract != bt_retract_old) count++;
    if (count > 1)
    {
        sw_extend = sw_extend_old;
        sw_ontime = sw_ontime_old;
        bt_flip = bt_flip_old;
        bt_extend = bt_extend_old;
        bt_retract = bt_retract_old;
    }
    
}


void Display (unsigned int x)
{
    spi_digit1 = 0x10 + (x & 0x000f);
    spi_digit2 = 0x20 + ((x & 0x00f0) >> 4);
    spi_digit3 = 0x30 + ((x & 0x0f00) >> 8);
    spi_digit4 = 0x40 + ((x & 0xf000) >> 12);
    
}

void Display2 (unsigned int x, unsigned int y)
{
    if (display_digit == 0)
    {
        spi_digit1 = 0x90 + (x & 0x000f);
        spi_digit2 = 0x20 + ((x & 0x00f0) >> 4);
        spi_digit3 = 0x30 + ((x & 0x0f00) >> 8);
        spi_digit4 = 0x40 + ((x & 0xf000) >> 12);
    }
    else
    {
        spi_digit1 = 0x10 + (y & 0x000f);
        spi_digit2 = 0xa0 + ((y & 0x00f0) >> 4);
        spi_digit3 = 0x30 + ((y & 0x0f00) >> 8);
        spi_digit4 = 0x40 + ((y & 0xf000) >> 12);
    }
    
}

void Display3 (unsigned int x, unsigned int y, unsigned int z)
{
    
    switch (display_digit)
    {
            case 0:
                spi_digit1 = 0x90 + (x & 0x000f);
                spi_digit2 = 0x20 + ((x & 0x00f0) >> 4);
                spi_digit3 = 0x30 + ((x & 0x0f00) >> 8);
                spi_digit4 = 0x40 + ((x & 0xf000) >> 12);
                break;
            case 1:
                spi_digit1 = 0x10 + (y & 0x000f);
                spi_digit2 = 0xa0 + ((y & 0x00f0) >> 4);
                spi_digit3 = 0x30 + ((y & 0x0f00) >> 8);
                spi_digit4 = 0x40 + ((y & 0xf000) >> 12);
                break;
            case 2:
                spi_digit1 = 0x10 + (z & 0x000f);
                spi_digit2 = 0x20 + ((z & 0x00f0) >> 4);
                spi_digit3 = 0xb0 + ((z & 0x0f00) >> 8);
                spi_digit4 = 0x40 + ((z & 0xf000) >> 12);
                break;
    }
}

void Display4 (unsigned int x, unsigned int y, unsigned int z, unsigned int w)
{
    
    switch (display_digit)
    {
            case 0:
                spi_digit1 = 0x90 + (x & 0x000f);
                spi_digit2 = 0x20 + ((x & 0x00f0) >> 4);
                spi_digit3 = 0x30 + ((x & 0x0f00) >> 8);
                spi_digit4 = 0x40 + ((x & 0xf000) >> 12);
                break;
            case 1:
                spi_digit1 = 0x10 + (y & 0x000f);
                spi_digit2 = 0xa0 + ((y & 0x00f0) >> 4);
                spi_digit3 = 0x30 + ((y & 0x0f00) >> 8);
                spi_digit4 = 0x40 + ((y & 0xf000) >> 12);
                break;
            case 2:
                spi_digit1 = 0x10 + (z & 0x000f);
                spi_digit2 = 0x20 + ((z & 0x00f0) >> 4);
                spi_digit3 = 0xb0 + ((z & 0x0f00) >> 8);
                spi_digit4 = 0x40 + ((z & 0xf000) >> 12);
                break;
            case 3:
                spi_digit1 = 0x10 + (w & 0x000f);
                spi_digit2 = 0x20 + ((w & 0x00f0) >> 4);
                spi_digit3 = 0x30 + ((w & 0x0f00) >> 8);
                spi_digit4 = 0xc0 + ((w & 0xf000) >> 12);
                break;
    }
}

void Gyro_LEDS (void)
{
    if (adc_gyro > (adc_gyro0 + gyro_deadband)) //positive counterclockwise
        {
            LED_G = 1;
            LED_R = 0;
        }
        else
        {
            if (adc_gyro < (adc_gyro0 - gyro_deadband))
            {
                LED_G = 0;
                LED_R = 1;
            }
            else
            {
                LED_G = 0;
                LED_R = 0;
            }  
        }
}

int RCvalue_cond (int x)
{
    if (x > -RCvalue_DB && x < RCvalue_DB)
        {
            x = 0;
        }
        else
        {
            if (x > 0)
            {
                if (x > RCvalue_max)
                {
                    x = RCvalue_max;
                }
            }
            else
            {
                if (x < RCvalue_min)
                {
                    x = RCvalue_min;
                }
            }
        }
    return(x);
}

void Motor_OUT(void)
{
    if (SIGNAL_CHECK == 1)
    {
        //RCvalue = aile_buf - RCpulse_cen;   // 250 ns
        //RCvalue = RCvalue_cond(RCvalue);    // 1.2us
        MOTOR_L = PDC_cen + left_value / 8;     // 500ns //PDC1 = 0x1758 + RCvalue / 7.98;     //12.5us //PDC1 = 0x1758 + ((long)RCvalue * 100) / 798;    // 18us
        
        //RCvalue = elev_buf - RCpulse_cen;   // 250 ns
        //RCvalue = RCvalue_cond(RCvalue);    // 1.2us
        MOTOR_R = PDC_cen - right_value / 8;
    }
    else
    {
        MOTOR_L = PDC_cen;
        MOTOR_R = PDC_cen;
    }
}

void Signal_MIX(void)
{
    aile_value = aile_buf - RCpulse_cen;
    aile_value = RCvalue_cond(aile_value);
    
    elev_value = elev_buf - RCpulse_cen;
    elev_value = RCvalue_cond(elev_value);
    
    thro_value = thro_buf - RCpulse_cen;
    thro_value = RCvalue_cond(thro_value);
    
    gear_value = gear_buf - RCpulse_cen;
    gear_value = RCvalue_cond(gear_value);
    
    if (gear_value < 0)
    {
        left_value = -elev_value + aile_value;
        right_value = -elev_value - aile_value;
    }
    else
    {
        left_value = elev_value + aile_value;
        right_value = elev_value - aile_value;
    }
    //if (SIGNAL_CHECK == 1)
    //{
    left_value = RCvalue_cond(left_value);
    right_value = RCvalue_cond(right_value);    
    //}
    //else
    //{
    //left_value = 0;
    //right_value = 0;    
    //}    
}

void Test_MIN_MAX(unsigned int x)
{
    if (bt_retract == 1)
    {
        test_min = 0xffff;
        test_max = 0x0000;
    }
    
    if (x > test_max)
    {
        test_max = x;
    }
    if (x < test_min)
    {
        test_min = x;
    }
}

void Display_buttons(void)
{
    if (display_digit == 0)
    {
        if (sw_extend == 1) spi_digit1 = 0x11;
        else spi_digit1 = 0x10;
        if (sw_ontime == 1) spi_digit2 = 0x21;
        else spi_digit2 = 0x20;
        if (bt_flip == 1) spi_digit3 = 0x31;
        else spi_digit3 = 0x30;
        if (bt_extend == 1) spi_digit4 = 0x41;
        else spi_digit4 = 0x40;
    }
    else
    {
        if (sw_extend == 1) spi_digit1 = 0x11;
        else spi_digit1 = 0x10;
        if (sw_ontime == 1) spi_digit2 = 0x21;
        else spi_digit2 = 0x20;
        if (bt_flip == 1) spi_digit3 = 0x31;
        else spi_digit3 = 0x30;
        if (bt_retract == 1) spi_digit4 = 0xc1;
        else spi_digit4 = 0xc0;
    }
    //display_buttons = 0;
}

void Test_Gyro_MIN_MAX(void)
{
    if (bt_retract == 1)
    {
        test_min = 0x0000;
        test_max = 0x0000;
    }
    
    if (gyro_value > test_max)
    {
        test_max = gyro_value;
    }
    if (gyro_value < test_min)
    {
        test_min = gyro_value;
    }
}

/*void Test_spin(void)
{
    if (test_spin_on == 0)
    {
        if (bt_flip == 1 && wait != 1)
        {
            test_min = 0x0000;
            test_max = 0x0000;
            test_spin_on = 1;
            wait = 1;
            if (gear_value < 0)
            {
                left_value = RCvalue_max/2;
                right_value = RCvalue_max/2;
            }
            else
            {
                left_value = RCvalue_min/2;
                right_value = RCvalue_min/2;
            }
            //left_value = RCvalue_max;
            //right_value = RCvalue_max;
            MOTOR_L = PDC_cen + left_value / 8;
            MOTOR_R = PDC_cen + right_value / 8;
            TMR5 = 0x0000;
            TMR4 = 0x0000;
            T4CONbits.TON = 1;
        }
        else
        {
            Signal_MIX();
            Motor_OUT();
        }
    }
    else
    {
        if (gyro_value <= -0x64 || gyro_value >= 0x64)
        {
            test_timer_lsb = TMR4;
            test_timer_msb = TMR5HLD;
            T4CONbits.TON = 0;
            TMR5HLD = 0;
            TMR5 = 0x0000;
            TMR4 = 0x0000;
            test_timer = 0;
            test_timer = test_timer + test_timer_msb << 16;
            test_timer = test_timer + test_timer_lsb;
            test_timer_seconds = test_timer * 0.0000312;//1557316
            t4 = test_timer_seconds / 1000;
            tt = test_timer_seconds % 1000;
            t3 = tt / 100;
            tt = tt % 100;
            t2 = tt / 10;
            t1 = tt % 10;
            t4 = t4 << 12;
            t3 = t3 << 8;
            t2 = t2 << 4;
            tt = t1 + t4 + t3 + t2;
            test_spin_on = 0;
            MOTOR_L = PDC_cen;
            MOTOR_R = PDC_cen;
            wait = 1;
            wait_count = 0;
        }
        else
        {
            
        }
    }
}*/

void STOP(void)
{
    if (bt_extend == 1)
    {
        //T4CONbits.TON = 0;
        //TMR5HLD = 0;
        //TMR5 = 0x0000;
        //TMR4 = 0x0000;
        test_spin_on = 0;
        MOTOR_L = PDC_cen;
        MOTOR_R = PDC_cen;
    }
    
    if (bt_retract == 1)
    {
        turn_deg = 0;
    }
}

void Test_spin2(void)
{
    if (test_spin_on == 0)
    {
        if (bt_flip == 1 && wait != 1)
        {
            final = 0;
            test_min = 0x0000;
            test_max = 0x0000;
            test_spin_on = 1;
            wait = 1;
            if (gear_value < 0)
            {
                left_value = RCvalue_max/3;
                right_value = RCvalue_max/3;
            }
            else
            {
                left_value = RCvalue_min/3;
                right_value = RCvalue_min/3;
            }
            //left_value = RCvalue_max;
            //right_value = RCvalue_max;
            MOTOR_L = PDC_cen + left_value / 8;
            MOTOR_R = PDC_cen + right_value / 8;
            TMR5HLD = 0x0000;
            TMR5 = 0x0000;
            TMR4 = 0x0000;
            T4CONbits.TON = 1;
        }
        else
        {
            Signal_MIX();
            Motor_OUT();
        }
    }
    else
    {
        if (test_spin_on == 1)
        {
            if (gyro_value <= -0xfa || gyro_value >= 0xfa)
            {
                test_timer_lsb = TMR4;
                test_timer_msb = TMR5HLD;
                T4CONbits.TON = 0;
                TMR5HLD = 0;
                TMR5 = 0x0000;
                TMR4 = 0x0000;
                T4CONbits.TON = 1;
                test_timer = 0;
                test_timer = test_timer + test_timer_msb << 16;
                test_timer = test_timer + test_timer_lsb;
                test_timer_seconds = test_timer * 0.0000312;//1557316
                t4 = test_timer_seconds / 1000;
                tt = test_timer_seconds % 1000;
                t3 = tt / 100;
                tt = tt % 100;
                t2 = tt / 10;
                t1 = tt % 10;
                t4 = t4 << 12;
                t3 = t3 << 8;
                t2 = t2 << 4;
                tt = t1 + t4 + t3 + t2;
                test_spin_on = 2;
                MOTOR_L = PDC_cen;
                MOTOR_R = PDC_cen;
                wait = 1;
                wait_count = 0;
                //return;
            }
        }
        else
        {
            if (gyro_value >= -gyro_deadband && gyro_value <= gyro_deadband)
            {
                test_timer2_lsb = TMR4;
                test_timer2_msb = TMR5HLD;
                T4CONbits.TON = 0;
                TMR5HLD = 0;
                TMR5 = 0x0000;
                TMR4 = 0x0000;
                test_timer2 = 0;
                test_timer2 = test_timer2 + test_timer2_msb << 16;
                test_timer2 = test_timer2 + test_timer2_lsb;
                //test_timer2 = test_timer2 - test_timer;
                test_timer2_seconds = test_timer2 * 0.0000312;//1557316
                t4 = test_timer2_seconds / 1000;
                tt2 = test_timer2_seconds % 1000;
                t3 = tt2 / 100;
                tt2 = tt2 % 100;
                t2 = tt2 / 10;
                t1 = tt2 % 10;
                t4 = t4 << 12;
                t3 = t3 << 8;
                t2 = t2 << 4;
                tt2 = t1 + t2 + t3 + t4;
                test_spin_on = 0;
                MOTOR_L = PDC_cen;
                MOTOR_R = PDC_cen;
                wait = 1;
                wait_count = 0;
            }
        }
    }
}

void Uturn(void)
{
    if (test_spin_on == 0)
    {
        if (bt_flip == 1 && wait != 1)
        {
            LED_G = 1;
            LED_R = 1;
            test_spin_on = 1;
            wait = 1;
            uturn_counter = 0;
            //T4CONbits.TON = 1;
            //gyro_inc = 0;
            turn_deg = 0;
            MOTOR_L = PDC_cen;
            MOTOR_R = PDC_cen;
            //gyro[gyro_inc] = gyro_value;
            //gyro_inc ++;
            gyro_update = 0;
            gyro_break = 1;
        }
        else
        {
            Signal_MIX();
            Motor_OUT();
        }
    }
    else
    {
        
        if (gyro_update == 1)
        {
            
            turn_deg = turn_deg + turn_deg_100;
            //gyro[gyro_inc] = gyro_value;
            if (gyro_break == 1)
            {
                //turn_deg = turn_deg + (gyro[gyro_inc] - gyro[gyro_inc - 1]) * 0.732;
                if (uturn_counter > 4)
                {
                    gyro_break = 0;
                    //gyro_inc = 0;
                    if (turn_deg > 0) turn_dir = 1; // positive is ccw left
                    else turn_dir = 0;
                    turn_deg = 0;
                }
                else
                {
                    //gyro_inc ++;
                }
            }
            else
            {
                if (uturn_counter < 16)
                {
                    //LED_G = 0;
                    if (turn_dir == 1)
                    {
                        if (LED_G == 1) LED_G = 0;
                        else LED_G = 1;
                        MOTOR_L = PDC_cen + (RCvalue_min / 80) * (uturn_counter - 5);
                        MOTOR_R = PDC_cen + (RCvalue_min / 80) * (uturn_counter - 5);
                    }
                    else
                    {
                        if (LED_R == 1) LED_R = 0;
                        else LED_R = 1;
                        MOTOR_L = PDC_cen + (RCvalue_max / 80) * (uturn_counter - 5);
                        MOTOR_R = PDC_cen + (RCvalue_max / 80) * (uturn_counter - 5);
                    }
                    //turn_deg = turn_deg + gyro[gyro_inc] * 0.732;
                    //gyro_inc ++;
                }
                else
                {
                    //gyro_inc = 0;
                    brake_estimate = (long)gyro_value * gyro_value * 894 / 10000000;// 819;//6;
                    if (turn_deg >= 0)
                    {
                        if ((turn_deg/10 + brake_estimate) > 1750)
                        {
                            LED_G = 0;
                            MOTOR_L = PDC_cen;
                            MOTOR_R = PDC_cen;
                            final = 1;
                            //test_spin_on = 0;
                            //turn_deg = turn_deg + brake_estimate;
                        }
                    }
                    else
                    {
                        if ((turn_deg/10 - brake_estimate) < -1750)
                        {
                            LED_R = 0;
                            MOTOR_L = PDC_cen;
                            MOTOR_R = PDC_cen;
                            final = 1;
                            //test_spin_on = 0;
                            //turn_deg = turn_deg - brake_estimate;
                        }
                    }
                }
            }
            uturn_counter ++;
            gyro_update = 0;
            wait = 1;
            if (final == 1)
            {
                //turn_deg = turn_deg + turn_deg_100;
                if (gyro_value > -gyro_deadband && gyro_value < gyro_deadband)
                {
                    test_spin_on = 0;
                    final = 0;
                }
                else
                {
                    
                }
            }
        }
    }
}

void angle_tracker(void)
{
    if (gyro_update == 1)
    {
        //if (gyro_value >= -gyro_deadband*2 && gyro_value <= gyro_deadband*2) gyro_value = 0;
        
        turn_deg = turn_deg + turn_deg_100;
        gyro_update = 0;
    }
}
int hextobcd (int x)
{
    unsigned int td1;
    unsigned int td2;
    unsigned int td3;
    unsigned int td4;
    unsigned int tdt;
    unsigned int y;
    if (x >= 0) x = x;
    else x = -x;
    
    
    td1=0;
    td2=0;
    td3=0;
    td4=0;
    tdt=0;
    td4 = x / 1000;
    tdt = x % 1000;
    td3 = tdt / 100;
    tdt = tdt % 100;
    td2 = tdt / 10;
    td1 = tdt % 10;
    //td4 = td4 << 12;
    //td3 = td3 << 8;
    //td2 = td2 << 4;
    tdt = td1 + (td4<< 12) + (td3<< 8) + (td2<< 4);
    return (tdt);
}