/*
 * File:   main.c
 * Author: lexa
 *
 * Modified September 29 2019
 */
#include <xc.h>
#include <stdint.h>
#include <p24EP32MC202.h>
//#include <stdbool.h>
#include <string.h>
// #include <stdlib.h>

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

#define NEWLINE 0xA
#define ADC_GYRO_AVE 10

void Gyro_LEDS(void);
int RCvalue_cond(int x);
void Motor_OUT(void);
void Signal_MIX(void);
void Uturn(void);
void angle_tracker(void);
int hextobcd(int x);
void Rudder_con(void);
//void STOP(void);
void UART_send(void);
char value_to_uart(char value[], unsigned char *pos, unsigned char len);
char *hex_to_bcd_str(unsigned long value, char sign, char frac_dig);

char UART_done = 1;
char mcu_on;
//int adc_batt;
int adc_gyro_input;
int adc_gyro_ref;
unsigned int aile_buf;
unsigned int elev_buf;
unsigned int thro_buf;
unsigned int gear_buf;
int aile_value;
int elev_value;
int thro_value;
int gear_value;
int adc_gyro_value;
int gyro_value_ave;
char gyro_deadband;
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
int RCvalue_min;
int RCvalue_max;
int RCvalue_DB;
char SIGNAL_AILE;
char SIGNAL_ELEV;
char SIGNAL_THRO;
char SIGNAL_GEAR;
char SIGNAL_CHECK;
unsigned int PDC_cen;
int left_value;
int right_value;

int test_min;
int test_max;

//180 degree turn
unsigned int uturn_counter;
int gyro[10];
unsigned char gyro_inc;
char gyro_update;
char gyro_break;
int turn_deg_x_10;
long turn_deg_sum;
int deg_per_interrupt;
char turn_dir;
unsigned int brake_estimate;
char wait;
char test_spin_on;
int wait_count;

char final;
char rudd_left;
char rudd_right;
unsigned char adc_gyro_inc;
int adc_gyro_value_old;
int adc_gyro[ADC_GYRO_AVE];
long adc_gyro_sum;
int gyro_value_comp;
int display_min = 0x7fff;
int display_max = 0xffff;
unsigned int TMR_DIFF = 0;
long gyro_value_sum = 0;


void __attribute__((interrupt, no_auto_psv)) _ISR _IC1Interrupt(void) {  // 1.3 us
    //LED_R = 1;
    if (PORTBbits.RB5 == 0) {
        IC1BUF_temp1 = IC1BUF;
        gear_buf = IC1BUF_temp1 - IC1BUF_temp;
        SIGNAL_GEAR = 1;
    } else {
        IC1BUF_temp = IC1BUF;
    }
    while (IC1CON1bits.ICBNE == 1) {
        temp = IC1BUF;
    }
    if (gear_buf > 0xef00 || gear_buf < 0x8620) {
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
    //LED_R = 0;
}

void __attribute__((interrupt, no_auto_psv)) _ISR _IC2Interrupt(void) {
    if (PORTBbits.RB4 == 0) {
        IC2BUF_temp1 = IC2BUF;
        elev_buf = IC2BUF_temp1 - IC2BUF_temp;
        SIGNAL_ELEV = 1;
    } else {
        IC2BUF_temp = IC2BUF;
    }
    while (IC2CON1bits.ICBNE == 1) {
        temp = IC2BUF;
    }
    if (elev_buf > 0xef00 || elev_buf < 0x8620) {
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

void __attribute__((interrupt, no_auto_psv)) _ISR _IC3Interrupt(void) {
    if (PORTBbits.RB10 == 0) {
        IC3BUF_temp1 = IC3BUF;
        aile_buf = IC3BUF_temp1 - IC3BUF_temp;
        SIGNAL_AILE = 1;
    } else {
        IC3BUF_temp = IC3BUF;
    }
    while (IC3CON1bits.ICBNE == 1) {
        temp = IC3BUF;
    }
    if (aile_buf > 0xef00 || aile_buf < 0x8620) {
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

void __attribute__((interrupt, no_auto_psv)) _ISR _IC4Interrupt(void) {
    if (PORTBbits.RB6 == 0) {
        IC4BUF_temp1 = IC4BUF;
        thro_buf = IC4BUF_temp1 - IC4BUF_temp;
        SIGNAL_THRO = 1;
    } else {
        IC4BUF_temp = IC4BUF;
    }
    while (IC4CON1bits.ICBNE == 1) {
        temp = IC4BUF;
    }
    if (thro_buf > 0xef00 || thro_buf < 0x8620) {
        IC4CON1bits.ICM = 0;
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
        IC4CON1bits.ICM = 1;
        SIGNAL_THRO = 0;
        thro_buf = 0xba90; //8c66;
    }
    IFS2bits.IC4IF = 0;
}

void __attribute__((__interrupt__, auto_psv)) _AD1Interrupt(void) {  // every 30us; takes 2us
    adc_gyro_ref = ADC1BUF0;
    adc_gyro_input = ADC1BUF1;
    adc_gyro_value = adc_gyro_input - adc_gyro_ref;

    adc_gyro_inc = adc_gyro_inc == ADC_GYRO_AVE ? 0 : adc_gyro_inc;
    adc_gyro_value_old = adc_gyro[adc_gyro_inc];
    adc_gyro[adc_gyro_inc] = adc_gyro_value;
    adc_gyro_sum = adc_gyro_sum - adc_gyro_value_old + adc_gyro_value;
    adc_gyro_inc ++;

    IFS0bits.AD1IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt() { // SPI output timer 250ms period
    if (mcu_on == 0) {
        mcu_on = 1;
    }
    // LED_G = LED_G ? 0 : 1;
    UART_done = 0;
    IFS0bits.T1IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _T2Interrupt() { // 100ms loss of signal output kill control timer
    if (SIGNAL_AILE == 1) SIGNAL_AILE = 2;
    else SIGNAL_AILE = 0;

    if (SIGNAL_ELEV == 1) SIGNAL_ELEV = 2;
    else SIGNAL_ELEV = 0;

    if (SIGNAL_THRO == 1) SIGNAL_THRO = 2;
    else SIGNAL_THRO = 0;

    if (SIGNAL_GEAR == 1) SIGNAL_GEAR = 2;
    else SIGNAL_GEAR = 0;

    if (SIGNAL_AILE == 0 && SIGNAL_ELEV == 0) {
        MOTOR_L = PDC_cen;
        MOTOR_R = PDC_cen;
        SIGNAL_CHECK = 0;
        test_spin_on = 0;
        rudd_left = 0;
        rudd_right = 0;
    }
    else SIGNAL_CHECK = 1;

    _T2IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _T3Interrupt() {
    T3CONbits.TON = 0;
    _T3IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _T4Interrupt() {// Gyro_value calcs and value update timer; 2ms period; update every 20ms; 20us, 70us all parts
    TMR_DIFF = TMR4;
    _T4IF = 0;
    gyro_value_comp = adc_gyro_sum + 63;// / ADC_GYRO_AVE; //was (adc_gyro_sum + 625) / 100; - smaller makes zero creep CCW : 0 - deg grow negative
    // if (gyro_value_comp > -150 && gyro_value_comp < 100) gyro_value_comp = 0;
    gyro_inc = gyro_inc == 10 ? 0 : gyro_inc;
    static int gyro_value_comp_old = 0;
    gyro_value_comp_old = gyro[gyro_inc];
    gyro[gyro_inc] = gyro_value_comp;
    gyro_value_sum = gyro_value_sum - gyro_value_comp_old + gyro_value_comp;
    gyro_value_ave = gyro_value_sum / 10;
    if (gyro_value_ave > -50 && gyro_value_ave < 50) gyro_value_ave = 0;
    gyro_inc++;

    if (gyro_value_ave > -500 && gyro_value_ave < 500) deg_per_interrupt = gyro_value_ave * 1.54;
    else if (gyro_value_ave > -1000 && gyro_value_ave < 1000) deg_per_interrupt = gyro_value_ave * 1.53;
    else deg_per_interrupt = gyro_value_ave * 1.52; // 203
    // if (deg_per_interrupt > -100 && deg_per_interrupt < 100) deg_per_interrupt = 0;
    turn_deg_sum = turn_deg_sum + deg_per_interrupt;
    turn_deg_x_10 = turn_deg_sum / 1000;

    gyro_update = 1;
    TMR_DIFF = TMR4 - TMR_DIFF;
}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt() {

    IFS0bits.U1TXIF = 0;
}

int main(void) {
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      pins
    LATA = 0x0000;  //Setting the Output Latch SFR(s)
    LATB = 0x0000;
    TRISA = 0x0003; //Setting the GPIO Direction SFR(s)
    // TRISB = 0x047D; // SPI RB7 - SCK output
    TRISB = 0x04FD; // UART RB7 - U1RX input
    CNPDA = 0x0000; //Setting the Weak Pull Up and Weak Pull Down SFR(s)
    CNPDB = 0x0470;
    CNPUA = 0x0000;
    CNPUB = 0x0000;
    ODCA = 0x0000;  //Setting the Open Drain SFR(s)
    ODCB = 0x0000;
    ANSELA = 0x0003;    //Setting the Analog/Digital Configuration SFR(s)
    ANSELB = 0x0001;
    __builtin_write_OSCCONL(OSCCON & 0xbf); //Set the PPS; unlock PPS
    RPINR8bits.IC4R = 0x0026; //RB6->IC4:IC4;
    RPINR8bits.IC3R = 0x002A; //RB10->IC3:IC3;
    RPINR7bits.IC1R = 0x0025; //RB5->IC1:IC1;
    RPINR7bits.IC2R = 0x0024; //RB4->IC2:IC2;
    RPINR18bits.U1RXR = 0x0027; //RB7
    // RPOR3 = 0x0001; //RB8->U1TX
    RPOR3bits.RP40R = 0x01; //RB8->U1TX
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      oscillator
    CLKDIV = 0x3040; // FRCDIV FRC/1; PLLPRE 2; DOZE 1:8; PLLPOST 1:4; DOZEN disabled; ROI disabled;
    OSCTUN = 0x0000; // TUN Center frequency;
    REFOCON = 0x0000; // ROON disabled; ROSEL disabled; RODIV Base clock value; ROSSLP disabled;
    PLLFBD = 0x0043; //0x0043 0x001f lowest // PLLDIV 67;
    CORCONbits.RND = 0; // RND disabled; SATB disabled; SATA disabled; ACCSAT disabled;
    CORCONbits.SATB = 0;
    CORCONbits.SATA = 0;
    CORCONbits.ACCSAT = 0;
    // CF no clock failure; NOSC FRCPLL; CLKLOCK unlocked; OSWEN Switch is Complete;
    __builtin_write_OSCCONH((uint8_t) ((0x0100 >> _OSCCON_NOSC_POSITION) & 0x00FF));
    __builtin_write_OSCCONL((uint8_t) ((0x0100 | _OSCCON_OSWEN_MASK) & 0xFF));

    while (OSCCONbits.OSWEN != 0); // Wait for Clock switch to occur

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
    //SPI1CON1 = 0x0038; //was 0x0022; 0x0038 2us pulse
    // SPIFSD disabled; SPIBEN enabled; FRMPOL disabled; FRMDLY disabled; FRMEN disabled;
    //SPI1CON2 = 0x0001; //0x0001
    // SISEL SPI_INT_TRMT_COMPLETE; SPIROV disabled; SPIEN enabled; SPISIDL disabled;
    //SPI1STAT = 0x8014;
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
    AD1CSSH = 0x0000; // CSS2 enabled; CSS1 enabled; CSS0 enabled; CSS5 disabled; CSS4 disabled; CSS3 disabled;
    AD1CSSL = 0x0007; // DMABL Allocates 1 word of buffer to each analog input; ADDMAEN disabled;
    AD1CON4 = 0x0000; // CH123SA disabled; CH123SB CH1=OA2/AN0,CH2=AN1,CH3=AN2; CH123NA disabled; CH123NB CH1=VREF-,CH2=VREF-,CH3=VREF-;
    AD1CHS123 = 0x0000;
    _ADCS = 6;  //3 = 17us; 4 = 21.5 us; 5 = 26us period; 6 = 30us; 7 = 34us; 8 = 38us;
    _SAMC = 31;

    IEC0bits.AD1IE = 1;
    //_SAMC = 3;
    //_ADCS = 5;
    _ADON = 1;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      TMR1
    //   Timer 1 controls SPI output
    TMR1 = 0x0000; // period is about 250 ms
    PR1 = 0x793e;
    T1CON = 0x8030;
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
    T1CONbits.TON = 1;

    mcu_on = 0;
    while (mcu_on == 0) {}

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

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      TMR2
    //  Timer 2 monitors RC signal presence, if not clears SIGNAL_CHECK
    TMR2 = 0;
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

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      TMR3
    //  Timer 3 for
    TMR3 = 0;
    T3CON = 0x0000;
    PR3 = 0xf4f4;
    // PR2 = 0xf4f4; T2CONbits.TCKPS = 3; period 500ms
    // PR2 = 0xc3f6; T2CONbits.TCKPS = 2; period 100ms
    // PR2 = 0xeb27; T2CONbits.TCKPS = 1; period 15ms
    // PR2 = 0x9cc5; T2CONbits.TCKPS = 1; period 10ms
    T3CONbits.TCKPS = 3;
    _T3IF = 0;
    _T3IE = 1;
    T3CONbits.TON = 0;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      TMR4
    // Gyro_value calcs and value update timer
    TMR4 = 0; //lsb
    T4CON = 0x0000;
    PR4 = 0xf876; //lsb; 0xf876 + tckps=0 == .002 sec period; tckps=1 == .016 sec period
    T4CONbits.TCKPS = 0; // 1== x8
    _T4IF = 0;
    _T4IE = 1;
    T4CONbits.TON = 1;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      UART1
    LED_R = 1;
    U1BRG = 0x0010;
    U1MODEbits.ABAUD = 0;
    U1MODEbits.PDSEL = 0x00;
    U1MODEbits.STSEL = 0;
    U1MODEbits.RTSMD = 0;
    U1MODEbits.BRGH = 0;
    U1STAbits.UTXINV = 0;
    // IFS0bits.U1TXIF = 0;
    // IEC0bits.U1TXIE = 1;
    U1MODEbits.UARTEN = 1;

    long i;
    for (i = 0; i < 1000000; i++) {
        Nop();
    }
    U1STAbits.UTXEN = 1;

    for (i = 0; i < 1000000; i++) {
        Nop();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    LED_G = 0;
    LED_R = 0;
    gyro_deadband = 10; //was 23
    RCpulse_min = 0x8a20; //35360
    RCpulse_cen = 0xba90; //47760
    RCpulse_max = 0xeb00; //60160
    RCswing = 0x60e0; //24800
    RCvalue_max = 0x3070;
    RCvalue_min = -RCvalue_max;
    RCvalue_DB = 256;
    SIGNAL_AILE = 0;
    SIGNAL_ELEV = 0;
    SIGNAL_THRO = 0;
    SIGNAL_GEAR = 0;
    SIGNAL_CHECK = 0;
    PDC_cen = 0x1758;

    aile_value = 0;
    elev_value = 0;

    test_spin_on = 0;
    wait_count = 0;
    uturn_counter = 0;
    gyro_inc = 0;
    gyro_update = 0;
    gyro_break = 0;
    turn_dir = 0;
    gyro_value_ave = 0;
    final = 0;
    rudd_left = 0;
    rudd_right = 0;
    adc_gyro_inc = 0;

    deg_per_interrupt = 0;
    turn_deg_sum = 0;
    turn_deg_x_10 = 0;

    while (1) {

        // if (adc_gyro_input < display_min) {
        //     display_min = adc_gyro_input;
        // }
        // if (adc_gyro_input > display_max) {
        //     display_max = adc_gyro_input;
        // }

        // if (gyro_value_ave < display_min) {
        //     display_min = gyro_value_ave;
        // }
        // if (gyro_value_ave > display_max) {
        //     display_max = gyro_value_ave;
        // }

        // if (adc_gyro_sum < display_min) {
        //     display_min = adc_gyro_sum;
        // }
        // if (adc_gyro_sum > display_max) {
        //     display_max = adc_gyro_sum;
        // }

        if (gyro_value_comp < display_min) {
            display_min = gyro_value_comp;
        }
        if (gyro_value_comp > display_max) {
            display_max = gyro_value_comp;
        }

        // if (gyro_value_ave < display_min) {
        //     display_min = gyro_value_ave;
        // }
        // if (gyro_value_ave > display_max) {
        //     display_max = gyro_value_ave;
        // }


        UART_send();

        if(U1STAbits.URXDA == 1) {
            char buff = U1RXREG;
            if (buff == 'r') { // 'r' = 0x72
                turn_deg_sum = 0;
                display_min = 0x7fff;
                display_max = 0xffff;
            }
        }


        //STOP();
        //Uturn();
        //Rudder_con();

        // if (U1STAbits.PERR == 1) {
        //     LED_R = 1;
        //     // U1TXREG = 0x01;
        // } else {
        //     LED_R = 0;
        // }
        // if (U1STAbits.FERR == 1) {
        //     LED_R = 1;
        //     // U1TXREG = 0x01;
        // } else {
        //     LED_R = 0;
        // }
        // if (IFS0bits.U1TXIF == 1) {
        //     LED_G = 0;
        // }
        // Must clear the overrun error to keep UART receiving
        // if(U1STAbits.OERR == 1)
        // {
        //     U1STAbits.OERR = 0;
        // }

        //Signal_MIX();
        //Motor_OUT();
        //angle_tracker();

        Gyro_LEDS();
        //LED_R = 0;
    }
}

void Gyro_LEDS(void) {
    if (gyro_value_ave > 0) { //positive counterclockwise
        LED_G = 0;
        LED_R = 1;
    } else {
        if (gyro_value_ave < 0) {
            LED_G = 1;
            LED_R = 0;
        } else {
            LED_G = 0;
            LED_R = 0;
        }
    }
}

int RCvalue_cond(int x) {
    if (x > -RCvalue_DB && x < RCvalue_DB) {
        x = 0;
    } else {
        if (x > 0) {
            if (x > RCvalue_max) {
                x = RCvalue_max;
            }
        } else {
            if (x < RCvalue_min) {
                x = RCvalue_min;
            }
        }
    }
    return (x);
}

void Motor_OUT(void) {
    if (SIGNAL_CHECK == 1) {
        //RCvalue = aile_buf - RCpulse_cen;   // 250 ns
        //RCvalue = RCvalue_cond(RCvalue);    // 1.2us
        MOTOR_L = PDC_cen + left_value / 8; // 500ns //PDC1 = 0x1758 + RCvalue / 7.98;

        //RCvalue = elev_buf - RCpulse_cen;   // 250 ns
        //RCvalue = RCvalue_cond(RCvalue);    // 1.2us
        MOTOR_R = PDC_cen - right_value / 8;
    } else {
        MOTOR_L = PDC_cen;
        MOTOR_R = PDC_cen;
    }
}

void Signal_MIX(void) {
    aile_value = aile_buf - RCpulse_cen;
    aile_value = RCvalue_cond(aile_value);

    elev_value = elev_buf - RCpulse_cen;
    elev_value = RCvalue_cond(elev_value);

    thro_value = thro_buf - RCpulse_cen;
    thro_value = RCvalue_cond(thro_value);

    gear_value = gear_buf - RCpulse_cen;
    gear_value = RCvalue_cond(gear_value);

    if (gear_value < 0) {
        left_value = -elev_value + aile_value;
        right_value = -elev_value - aile_value;
    } else {
        left_value = elev_value + aile_value;
        right_value = elev_value - aile_value;
    }

    left_value = RCvalue_cond(left_value);
    right_value = RCvalue_cond(right_value);
}

/*void STOP(void) {
    if (bt_extend == 1) {
        test_spin_on = 0;
        MOTOR_L = PDC_cen;
        MOTOR_R = PDC_cen;
    }
    if (bt_retract == 1) {
        turn_deg_x_10 = 0;
    }
}*/

void Uturn(void) {
    if (test_spin_on == 0) {
        if ((rudd_left == 1 || rudd_right == 1) && wait != 1) {
            //LED_G = 1;
            //LED_R = 1;
            test_spin_on = 1;
            wait = 1;
            uturn_counter = 0;
            //T4CONbits.TON = 1;
            //gyro_inc = 0;
            turn_deg_sum = 0;
            MOTOR_L = PDC_cen;
            MOTOR_R = PDC_cen;
            //gyro[gyro_inc] = adc_gyro_value;
            //gyro_inc ++;
            gyro_update = 0;
            gyro_break = 0;
        } else {
            Signal_MIX();
            Motor_OUT();
        }
    } else {
        if (gyro_update == 1) {
            //turn_deg_x_10 = turn_deg_x_10 + deg_per_interrupt;
            //gyro[gyro_inc] = adc_gyro_value;
            if (gyro_break == 1) {
                //turn_deg_x_10 = turn_deg_x_10 + (gyro[gyro_inc] - gyro[gyro_inc - 1]) * 0.732;
                if (uturn_counter >= 0) {
                    gyro_break = 0;
                    //gyro_inc = 0;
                    if (turn_deg_x_10 > 0) turn_dir = 1; // positive is ccw left
                    else turn_dir = 0;
                    turn_deg_x_10 = 0;
                } else {
                    //gyro_inc ++;
                }
            } else {
                if (uturn_counter < 2) {
                    //LED_G = 0;
                    if (rudd_left == 1) {
                        //if (LED_G == 1) LED_G = 0;
                        //else LED_G = 1;
                        MOTOR_L = PDC_cen + RCvalue_min / 8; //Cvalue_min / 80) * (uturn_counter - 5);
                        MOTOR_R = PDC_cen + RCvalue_min / 8; //(RCvalue_min / 80) * (uturn_counter - 5);
                    } else {
                        //if (LED_R == 1) LED_R = 0;
                        //else LED_R = 1;
                        MOTOR_L = PDC_cen + RCvalue_max / 8; //(RCvalue_max / 80) * (uturn_counter - 5);
                        MOTOR_R = PDC_cen + RCvalue_max / 8; //(RCvalue_max / 80) * (uturn_counter - 5);
                    }
                    //turn_deg_x_10 = turn_deg_x_10 + gyro[gyro_inc] * 0.732;
                    //gyro_inc ++;
                } else {
                    //gyro_inc = 0;
                    brake_estimate = (long) adc_gyro_value * adc_gyro_value * 16 / 10000; // 122/100000
                    if (turn_deg_x_10 >= 0) {
                        if ((turn_deg_x_10 + brake_estimate) > 1800) {
                            //LED_G = 0;
                            MOTOR_L = PDC_cen;
                            MOTOR_R = PDC_cen;
                            final = 1;
                            //test_spin_on = 0;
                            //turn_deg_x_10 = turn_deg_x_10 + brake_estimate;
                        }
                    } else {
                        if ((turn_deg_x_10 - brake_estimate) < -1800) {
                            //LED_R = 0;
                            MOTOR_L = PDC_cen;
                            MOTOR_R = PDC_cen;
                            final = 1;
                            //test_spin_on = 0;
                            //turn_deg_x_10 = turn_deg_x_10 - brake_estimate;
                        }
                    }
                }
            }
            uturn_counter++;
            gyro_update = 0;
            wait = 1;
            if (final == 1) {
                //turn_deg_x_10 = turn_deg_x_10 + deg_per_interrupt;
                if (adc_gyro_value > -gyro_deadband && adc_gyro_value < gyro_deadband) {
                    test_spin_on = 0;
                    final = 0;
                } else {

                }
            }
        }
    }
}

void angle_tracker(void) {
    if (gyro_update == 1) {
        //if (adc_gyro_value >= -gyro_deadband*2 && adc_gyro_value <= gyro_deadband*2) adc_gyro_value = 0;
        turn_deg_x_10 = turn_deg_x_10 + deg_per_interrupt;
        gyro_update = 0;

    }
}

void Rudder_con(void) { // positive is actually left
    if (thro_value > 10000) {
        rudd_left = 0;
        rudd_right = 1;
    } else if (thro_value < -10000) {
        rudd_left = 1;
        rudd_right = 0;
    } else {
        rudd_left = 0;
        rudd_right = 0;
    }
    //if (rudd_right == 1) turn_deg_sum = 0;
}

void UART_send(void) {
    #define key_len 11
    #define val_len 12
    #define headers_len 10
    #define init_default "xxxxxxxxxxxx", 0, 0, 0
    typedef struct {
        char key[key_len];
        char value[val_len];
        unsigned char key_pos;
        unsigned char val_pos;
        char done;
    } str_header;
    static str_header headers[headers_len] = {
        {"adc_val    ", init_default},
        {"adc_sum    ", init_default},
        {"gyro_v_a_a ", init_default},
        {"gyro_val_a ", init_default},
        {"       min ", init_default},
        {"       max ", init_default},
        {"timer4_buf ", init_default},
        {"deg_per_int", init_default},
        {"deg_sum    ", init_default},
        {"turn_deg   ", init_default}
    };
    void update_header_value(long value, str_header *header, char frac_dig) {
        char sign = value >= 0;
        unsigned int temp_value = value < 0 ? -value : value;
        char *converted = hex_to_bcd_str(temp_value, sign, frac_dig);
        unsigned char i;
        for (i = 0; i < val_len; i++, converted++) {
            header->value[i] = *converted;
        }
    }

    static unsigned char h_pos = 0;

    if (U1STAbits.UTXEN && U1STAbits.TRMT && UART_done == 0) {
        if (h_pos == 0 && headers[0].key_pos == 0) {
            U1TXREG = NEWLINE;
        }
        for (;h_pos < headers_len; h_pos++) {
            if (value_to_uart(headers[h_pos].key, &headers[h_pos].key_pos, key_len)) break;
            if (value_to_uart(headers[h_pos].value, &headers[h_pos].val_pos, val_len)) break;
            if (headers[h_pos].val_pos == val_len) {
                if (!U1STAbits.UTXBF) {
                    U1TXREG = NEWLINE;
                    headers[h_pos].key_pos = 0;
                    headers[h_pos].val_pos = 0;
                } else break;
            }
        }
        if (h_pos == headers_len) {
            h_pos = 0;
            UART_done = 1;
            // update_header_value((long)adc_gyro_ref, &headers[0], 0);
            // update_header_value((long)adc_gyro_input, &headers[0], 0);
            update_header_value((long)adc_gyro_value, &headers[0], 0);
            update_header_value(adc_gyro_sum, &headers[1], 0);
            update_header_value((long)gyro_value_comp, &headers[2], 0);
            update_header_value((long)gyro_value_ave, &headers[3], 0);
            update_header_value((long)display_min, &headers[4], 0);
            update_header_value((long)display_max, &headers[5], 0);
            update_header_value((long)TMR_DIFF, &headers[6], 0);
            update_header_value((long)deg_per_interrupt, &headers[7], 4);
            update_header_value(turn_deg_sum, &headers[8], 4);
            update_header_value(turn_deg_x_10, &headers[9], 1);
        }
    }
}

char value_to_uart(char value[], unsigned char *pos, unsigned char len) {
    for (; *pos < len; (*pos)++) {
        if (!U1STAbits.UTXBF) {
            U1TXREG = value[*pos];
        } else break;
    }
    return *pos < len ? 1 : 0;
}

char *hex_to_bcd_str(unsigned long value, char sign, char frac_dig) {
    #define frac_comp (frac_dig > 0 ? 1 : 0)
    #define num_len 10
    #define offset (val_len - num_len)
    #define str_position (i + offset - frac_comp)
    //4 292 967 295
    char digit;
    unsigned long factor = 1000000000;
    char first_non_zero_number = 0;
    static char result[val_len];
    result[0] = sign ? '+' : '-';
    unsigned char i;
    for (i = 0; i < offset; i++) {
        result[i + 1] = ' ';
    }
    for (i = 0; i < num_len + frac_comp; i++) {
        if (i == num_len - frac_dig) {
            result[i + offset - 1] = '.';
            continue;
        }
        digit = value / factor;
        if (digit == 0) {
            if (!first_non_zero_number) {
                if (i < num_len - 1 - frac_dig) {
                    result[str_position] = ' ';
                } else {
                    first_non_zero_number = 1;
                    result[str_position] = 48;
                }
            } else {
                result[str_position] = 48;
            }
        } else {
            first_non_zero_number = 1;
            result[str_position] = digit + 48;
        }
        value = value % factor;
        factor = factor / 10;
    }
    return result;
}
