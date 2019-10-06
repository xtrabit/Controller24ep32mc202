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
#define DELAY_105uS asm volatile ("REPEAT, #4201"); Nop(); // 105uS delay
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
void Test_Gyro_MIN_MAX(void);
void Test_MIN_MAX(int x);
void Test_MIN_MAX_unsigned(unsigned int x);
void UART_send(void);
char value_to_uart(char value[], unsigned char *pos, unsigned char len);
char *int_to_bcd_str(unsigned int value, char sign, char frac_dig);
char *long_to_bcd_str(unsigned long value, char sign, char frac_dig);

char UART_send_trigger = 0;
char UART_done = 1;
char mcu_on;
int spi_count_t1;   //SPI timer
char spi_digit1;
char spi_digit2;
char spi_digit3;
char spi_digit4;
char spi_digit[4];
int spi_digit_inc;
char display_digit;
char values_to_display;
//int adc_batt;
int adc_gyro;
int adc_gyro0;
unsigned int aile_buf;
unsigned int elev_buf;
unsigned int thro_buf;
unsigned int gear_buf;
int aile_value;
int elev_value;
int thro_value;
int gear_value;
int gyro_value;
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
int turn_deg;
long turn_deg_10;
int turn_deg_100;
// long turn_deg_100;
char turn_dir;
unsigned int brake_estimate;
char wait;
char test_spin_on;
int wait_count;

char final;
char rudd_left;
char rudd_right;
unsigned char gyro_inc_adc;
int gyro_value_adc_old;
int gyro_adc[ADC_GYRO_AVE];
long gyro_adc_sum;
//unsigned char gyro0_inc_adc;
//unsigned int gyro0_value_adc_ave;
//int adc_gyro0_old;
//int gyro0_adc[100];
//long gyro0_adc_sum;
int gyro_value_adc_ave;
int gyro_value_ave_min = 0;
int gyro_value_ave_max = 0;
unsigned int TMR_DIFF = 0;


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
    //LED_G = 1;
    adc_gyro0 = ADC1BUF0;
    adc_gyro = ADC1BUF1;
    gyro_value = adc_gyro - adc_gyro0;
    gyro_inc_adc = gyro_inc_adc == ADC_GYRO_AVE ? 0 : gyro_inc_adc;

    gyro_value_adc_old = gyro_adc[gyro_inc_adc];
    gyro_adc[gyro_inc_adc] = gyro_value;
    gyro_adc_sum = gyro_adc_sum - gyro_value_adc_old + gyro_value;

    // if (gyro_inc_adc < 100) { //was 100
    //     gyro_value_adc_old = gyro_adc[(gyro_inc_adc)];
    //     gyro_adc[(gyro_inc_adc)] = gyro_value;
    //     gyro_adc_sum = gyro_adc_sum - gyro_value_adc_old + gyro_value;
    // } else {
    //     gyro_inc_adc = 0;
    //     gyro_value_adc_old = gyro_adc[(gyro_inc_adc)];
    //     gyro_adc[(gyro_inc_adc)] = gyro_value;
    //     gyro_adc_sum = gyro_adc_sum - gyro_value_adc_old + gyro_value;
    // }
    //gyro_value_adc_ave = (gyro_adc_sum - 28) / 100; // - more negative = red less
    gyro_inc_adc ++;
    IFS0bits.AD1IF = 0;
    //LED_G = 0;
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt() { // SPI output timer 250ms period
    if (mcu_on == 0) {
        mcu_on = 1;
    }
    LED_G = LED_G ? 0 : 1;
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
    gyro_value_adc_ave = (gyro_adc_sum + 62) / ADC_GYRO_AVE; //was (gyro_adc_sum + 625) / 100; - smaller makes zero creep CCW : 0 - deg grow negative
    gyro_inc = gyro_inc == 10 ? 0 : gyro_inc;
    static int gyro_value_adc_ave_old = 0;
    static int gyro_value_adc_ave_sum = 0;
    gyro_value_adc_ave_old = gyro[gyro_inc];
    gyro[gyro_inc] = gyro_value_adc_ave;
    gyro_value_adc_ave_sum = gyro_value_adc_ave_sum - gyro_value_adc_ave_old + gyro_value_adc_ave;
    gyro_value_ave = gyro_value_adc_ave_sum / 10;
    // if (gyro_value_ave > -2 && gyro_value_ave < 2) gyro_value_ave = 0;
    gyro_inc++;

    // if (gyro_value_ave > -25 && gyro_value_ave < 25) turn_deg_100 = gyro_value_ave * 40;
    // if (gyro_value_ave > -40 && gyro_value_ave < 40) turn_deg_100 = gyro_value_ave * 30;
    // if (gyro_value_ave > -65 && gyro_value_ave < 65) turn_deg_100 = gyro_value_ave * 25;
    // if (gyro_value_ave > -90 && gyro_value_ave < 90) turn_deg_100 = gyro_value_ave * 20;
    if (gyro_value_ave > -100 && gyro_value_ave < 100) turn_deg_100 = gyro_value_ave * 15.4;
    else turn_deg_100 = gyro_value_ave * 15.2; // 203
    // turn_deg_100 = gyro_value_ave * 15.5; // 203
    // turn_deg_100 = (long) gyro_value_ave * 1510 / 1000; // 565
    if (turn_deg_100 > -100 && turn_deg_100 < 100) turn_deg_100 = 0;
    turn_deg_10 = turn_deg_10 + turn_deg_100;
    turn_deg = turn_deg_10 / 1000;
    gyro_update = 1;
    TMR_DIFF = TMR4 - TMR_DIFF;

    // if (gyro_inc < 9) {  // was 9
    //     gyro[gyro_inc] = gyro_value_adc_ave;
    //     gyro_inc++;
    // } else {   //55 us
    //     //LED_G = 1;
    //     gyro[gyro_inc] = gyro_value_adc_ave;
    //     gyro_value_ave = ((long) gyro[0] + gyro[1] + gyro[2] + gyro[3] + gyro[4] + gyro[5] + gyro[6] + gyro[7] + gyro[8] + gyro[9]) / 10; // 19 us
    //     //gyro_value_ave = ((long) gyro[0] + gyro[1] + gyro[2] + gyro[3] + gyro[4]) / 5;
    //     turn_deg_100 = (long) gyro_value_ave * 1510 / 1000; // together with above 38 us; _10==1465/10000 ; 1524 works well slow, adds 1 or 0.5 at quick 180; 760 at 5 steps
    //     if (turn_deg_100 > -2 && turn_deg_100 < 2) turn_deg_100 = 0;
    //     turn_deg_10 = turn_deg_10 + turn_deg_100;
    //     turn_deg = turn_deg_10 / 10;
    //     gyro_inc = 0;
    //     gyro_update = 1;
    // }
    //LED_G = 0;
    //LED_R = 0;
}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt() {
    if (UART_send_trigger || !UART_done) {
        UART_send_trigger = 0;
        UART_send();

    }
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
    // DELAY_105uS
    long i;
    for (i = 0; i < 1000000; i++) {
        Nop();
    }
    U1STAbits.UTXEN = 1;

    for (i = 0; i < 1000000; i++) {
        Nop();
    }
    // U1TXREG = 'a';

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    spi_count_t1 = 0;
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

    display_digit = 0;
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
    gyro_inc_adc = 0;

    turn_deg_100 = 0;
    turn_deg_10 = 0;
    turn_deg = 0;

    values_to_display = 1;
    spi_digit_inc = 3;

    while (1) {

        if (turn_deg_100 < gyro_value_ave_min) {
            gyro_value_ave_min = turn_deg_100;
        }
        if (turn_deg_100 > gyro_value_ave_max) {
            gyro_value_ave_max = turn_deg_100;
        }

        // if (gyro_value_ave < gyro_value_ave_min) {
        //     gyro_value_ave_min = gyro_value_ave;
        // }
        // if (gyro_value_ave > gyro_value_ave_max) {
        //     gyro_value_ave_max = gyro_value_ave;
        // }


        // if (TMR4 > TMR_DIFF) {
        //     TMR_DIFF = TMR4;
        // }
        //LED_R = LED_R ? 0 : 1;
        // if (UART_send_trigger || !UART_done) {
        //     UART_send_trigger = 0;
        //     UART_send();

        // }
        UART_send();

        if(U1STAbits.URXDA == 1) {
            char buff = U1RXREG;
            if (buff == 'r') { // 'r'
            // if (buff == 0x72) { // 'r'
                turn_deg_10 = 0;
                gyro_value_ave_min = 0;
                gyro_value_ave_max = 0;
            }
        }


        //STOP();
        //Uturn();
        //Rudder_con();


        // U1TXREG = "1";
        // U1TXREG = 0x0A;
        // if (U1STAbits.UTXBF == 0 && received) {
        //     LED_R = 0;
        //     // U1TXREG = 'a';
        //     char roger[] = "roger";
        //     char j = 0;
        //     for (j = 0; j < 5; j++) {
        //         while (U1STAbits.UTXBF == 1) {}
        //         U1TXREG = roger[j];
        //     }
        //     while (U1STAbits.UTXBF == 1) {}
        //     U1TXREG = 0x0A;
        //     received = 0;
        // } else {
        //     LED_R = 1;
        // }
        // if(U1STAbits.URXDA == 1)
        // {
        //     // int i;

        //     LED_G = 1;
        //     // for (i = 0; i < 1000; i++) {
        //     //     Nop();
        //     // }
        //     // U1TXREG = U1RXREG;
        //     temp_read = U1RXREG;
        //     received = 1;
        //     // for (i = 0; i < 1000; i++) {
        //     //     Nop();
        //     // }
        //     // U1TXREG = temp_read;
        // }




        // if (U1STAbits.UTXBF == 1) {
        //     LED_R = 1;
        //     // U1TXREG = 0x01;
        // }
        // if (U1STAbits.TRMT == 0) {
        //     LED_R = 1;
        //     // U1TXREG = 0x01;
        // } else {
        //     LED_R = 0;
        // }
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

        // if(U1STAbits.FERR == 1)
        // {
        // continue;
        // }
        /* Must clear the overrun error to keep UART receiving */
        // if(U1STAbits.OERR == 1)
        // {
        //     U1STAbits.OERR = 0;
        // }
        /* Get the data */
        // if(U1STAbits.URXDA == 1)
        // {
        //     int i;

        //     LED_G = 1;
        //     // for (i = 0; i < 1000; i++) {
        //     //     Nop();
        //     // }
        //     // U1TXREG = U1RXREG;
        //     temp_read = U1RXREG;
        //     // for (i = 0; i < 1000; i++) {
        //     //     Nop();
        //     // }
        //     // U1TXREG = temp_read;
        // }
        // U1TXREG = 'a';
        // if (U1STAbits.UTXBF == 0) {
        //     U1TXREG = spi_digit1;
        // }
        // if (U1STAbits.UTXBF == 0) {
        //     U1TXREG = spi_digit2;
        // }
        // if (U1STAbits.UTXBF == 0) {
        //     U1TXREG = spi_digit3;
        // }
        // if (U1STAbits.UTXBF == 0) {
        //     U1TXREG = spi_digit4;
        // }

        //Signal_MIX();
        //Motor_OUT();
        //angle_tracker();
        //Display(adc_gyro0);
        //Display(hextobcd(turn_deg));
        //Display(hextobcd(brake_estimate));
        //Display(hextobcd(gyro_value));
        //Display(hextobcd(gyro0_value_adc_ave));
        //Display(hextobcd(gyro_value_adc_ave));
        //Display(gyro_value_adc_ave);
        //Display(hextobcd(gyro_value_ave));
        //Test_MIN_MAX_unsigned(adc_gyro0);
        //Test_MIN_MAX(gyro_value);
        //Display2 (test_min, test_max); // for unsigned
        //Display2 (-test_min, test_max);
        //Display2 (left_value, right_value);
        //Display2 (aile_value, elev_value);
        //Display2 (aile_buf, elev_buf);
        //Display4 (-test_min, test_max, tt, tt2);
        //Test_Gyro_MIN_MAX();

        //Gyro_LEDS();
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
        MOTOR_L = PDC_cen + left_value / 8; // 500ns //PDC1 = 0x1758 + RCvalue / 7.98;     //12.5us //PDC1 = 0x1758 + ((long)RCvalue * 100) / 798;    // 18us

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

void Test_Gyro_MIN_MAX(void) {
    if (rudd_left == 1) {
        test_min = 0x0000;
        test_max = 0x0000;
    }
    if (gyro_value > test_max) {
        test_max = gyro_value;
    }
    if (gyro_value < test_min) {
        test_min = gyro_value;
    }
}

void Test_MIN_MAX(int x) {
    if (rudd_left == 1) {
        test_min = 0x0000;
        test_max = 0x0000;
    }
    if (x > test_max) {
        test_max = x;
    }
    if (x < test_min) {
        test_min = x;
    }
}

void Test_MIN_MAX_unsigned(unsigned int x) {
    if (rudd_left == 1) {
        test_min = 0xffff;
        test_max = 0x0000;
    }
    if (x > test_max) {
        test_max = x;
    }
    if (x < test_min) {
        test_min = x;
    }
}

/*void STOP(void) {
    if (bt_extend == 1) {
        //T4CONbits.TON = 0;
        //TMR5HLD = 0;
        //TMR5 = 0x0000;
        //TMR4 = 0x0000;
        test_spin_on = 0;
        MOTOR_L = PDC_cen;
        MOTOR_R = PDC_cen;
    }
    if (bt_retract == 1) {
        turn_deg = 0;
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
            turn_deg_10 = 0;
            MOTOR_L = PDC_cen;
            MOTOR_R = PDC_cen;
            //gyro[gyro_inc] = gyro_value;
            //gyro_inc ++;
            gyro_update = 0;
            gyro_break = 0;
        } else {
            Signal_MIX();
            Motor_OUT();
        }
    } else {
        if (gyro_update == 1) {
            //turn_deg = turn_deg + turn_deg_100;
            //gyro[gyro_inc] = gyro_value;
            if (gyro_break == 1) {
                //turn_deg = turn_deg + (gyro[gyro_inc] - gyro[gyro_inc - 1]) * 0.732;
                if (uturn_counter >= 0) {
                    gyro_break = 0;
                    //gyro_inc = 0;
                    if (turn_deg > 0) turn_dir = 1; // positive is ccw left
                    else turn_dir = 0;
                    turn_deg = 0;
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
                    //turn_deg = turn_deg + gyro[gyro_inc] * 0.732;
                    //gyro_inc ++;
                } else {
                    //gyro_inc = 0;
                    brake_estimate = (long) gyro_value * gyro_value * 16 / 10000; // 122/100000
                    if (turn_deg >= 0) {
                        if ((turn_deg + brake_estimate) > 1800) {
                            //LED_G = 0;
                            MOTOR_L = PDC_cen;
                            MOTOR_R = PDC_cen;
                            final = 1;
                            //test_spin_on = 0;
                            //turn_deg = turn_deg + brake_estimate;
                        }
                    } else {
                        if ((turn_deg - brake_estimate) < -1800) {
                            //LED_R = 0;
                            MOTOR_L = PDC_cen;
                            MOTOR_R = PDC_cen;
                            final = 1;
                            //test_spin_on = 0;
                            //turn_deg = turn_deg - brake_estimate;
                        }
                    }
                }
            }
            uturn_counter++;
            gyro_update = 0;
            wait = 1;
            if (final == 1) {
                //turn_deg = turn_deg + turn_deg_100;
                if (gyro_value > -gyro_deadband && gyro_value < gyro_deadband) {
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
        //if (gyro_value >= -gyro_deadband*2 && gyro_value <= gyro_deadband*2) gyro_value = 0;
        turn_deg = turn_deg + turn_deg_100;
        gyro_update = 0;

    }
}

int hextobcd(int x) {
    unsigned int td1;
    unsigned int td2;
    unsigned int td3;
    unsigned int td4;
    unsigned int tdt;

    if (x >= 0) x = x;
    else x = -x;

    td1 = 0;
    td2 = 0;
    td3 = 0;
    td4 = 0;
    tdt = 0;
    td4 = x / 1000;
    tdt = x % 1000;
    td3 = tdt / 100;
    tdt = tdt % 100;
    td2 = tdt / 10;
    td1 = tdt % 10;
    tdt = td1 + (td4 << 12) + (td3 << 8) + (td2 << 4);
    return (tdt);
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
    //if (rudd_right == 1) turn_deg_10 = 0;
}

void UART_send(void) {
    #define key_len 11
    #define val_len 12
    #define headers_len 12
    #define blank "xxxxxxxxxxxx"
    typedef struct {
        char key[key_len];
        char value[val_len];
        unsigned char key_pos;
        unsigned char val_pos;
        char done;
    } str_header;
    static str_header headers[headers_len] = {
        {
            "adc_buf0   ",
            blank,
            0,
            0,
            0
        },
        {
            "adc_buf1   ",
            blank,
            0,
            0,
            0
        },
        {
            "adc_val    ",
            blank,
            0,
            0,
            0
        },
        {
            "adc_sum    ",
            blank,
            0,
            0,
            0
        },
        {
            "gyro_v_a_a ",
            blank,
            0,
            0,
            0
        },
        {
            "gyro_val_a ",
            blank,
            0,
            0,
            0
        },
        {
            "gyro_v_min ",
            blank,
            0,
            0,
            0
        },
        {
            "gyro_v_max ",
            blank,
            0,
            0,
            0
        },
        {
            "timer4_buf ",
            blank,
            0,
            0,
            0
        },
        {
            "deg_100    ",
            blank,
            0,
            0,
            0
        },
        {
            "deg_10     ",
            blank,
            0,
            0,
            0
        },
        {
            "turn_deg   ",
            blank,
            0,
            0,
            0
        }
    };
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
            char sign;
            unsigned int temp_value;
            int temp_value_int;
            long temp_value_long;
            temp_value_int = adc_gyro0;
            sign = temp_value_int >= 0;
            temp_value = temp_value_int < 0 ? -temp_value_int : temp_value_int;
            unsigned char i;
            char *converted = int_to_bcd_str(temp_value, sign, 1);
            for (i = 0; i < val_len; i++, converted++) {
                headers[0].value[i] = *converted;
            }
            temp_value_int = adc_gyro;
            sign = temp_value_int >= 0;
            temp_value = temp_value_int < 0 ? -temp_value_int : temp_value_int;
            converted = int_to_bcd_str(temp_value, sign, 2);
            for (i = 0; i < val_len; i++, converted++) {
                headers[1].value[i] = *converted;
            }
            temp_value_int = gyro_value;
            sign = temp_value_int >= 0;
            temp_value = temp_value_int < 0 ? -temp_value_int : temp_value_int;
            converted = int_to_bcd_str(temp_value, sign, 0);
            for (i = 0; i < val_len; i++, converted++) {
                headers[2].value[i] = *converted;
            }
            temp_value_long = gyro_adc_sum;
            sign = temp_value_long >= 0;
            unsigned long temp_value_u_long;
            temp_value_u_long = temp_value_long < 0 ? -temp_value_long : temp_value_long;
            converted = long_to_bcd_str(temp_value_u_long, sign, 0);
            for (i = 0; i < val_len; i++, converted++) {
                headers[3].value[i] = *converted;
            }
            temp_value_int = gyro_value_adc_ave;
            sign = temp_value_int >= 0;
            temp_value = temp_value_int < 0 ? -temp_value_int : temp_value_int;
            converted = int_to_bcd_str(temp_value, sign, 0);
            for (i = 0; i < val_len; i++, converted++) {
                headers[4].value[i] = *converted;
            }
            temp_value_int = gyro_value_ave;
            sign = temp_value_int >= 0;
            temp_value = temp_value_int < 0 ? -temp_value_int : temp_value_int;
            converted = int_to_bcd_str(temp_value, sign, 0);
            for (i = 0; i < val_len; i++, converted++) {
                headers[5].value[i] = *converted;
            }
            temp_value_int = gyro_value_ave_min;
            sign = temp_value_int >= 0;
            temp_value = temp_value_int < 0 ? -temp_value_int : temp_value_int;
            converted = int_to_bcd_str(temp_value, sign, 0);
            for (i = 0; i < val_len; i++, converted++) {
                headers[6].value[i] = *converted;
            }
            temp_value_int = gyro_value_ave_max;
            sign = temp_value_int >= 0;
            temp_value = temp_value_int < 0 ? -temp_value_int : temp_value_int;
            converted = int_to_bcd_str(temp_value, sign, 0);
            for (i = 0; i < val_len; i++, converted++) {
                headers[7].value[i] = *converted;
            }
            converted = int_to_bcd_str(TMR_DIFF, 1, 0);
            for (i = 0; i < val_len; i++, converted++) {
                headers[8].value[i] = *converted;
            }
            temp_value_int = turn_deg_100;
            sign = temp_value_int >= 0;
            temp_value = temp_value_int < 0 ? -temp_value_int : temp_value_int;
            converted = int_to_bcd_str(temp_value, sign, 0);
            for (i = 0; i < val_len; i++, converted++) {
                headers[9].value[i] = *converted;
            }
            temp_value_long = turn_deg_10;
            sign = temp_value_long >= 0;
            temp_value_u_long = temp_value_long < 0 ? -temp_value_long : temp_value_long;
            converted = long_to_bcd_str(temp_value_u_long, sign, 4);
            for (i = 0; i < val_len; i++, converted++) {
                headers[10].value[i] = *converted;
            }
            temp_value_int = turn_deg;
            sign = temp_value_int >= 0;
            temp_value = temp_value_int < 0 ? -temp_value_int : temp_value_int;
            converted = int_to_bcd_str(temp_value, sign, 1);
            for (i = 0; i < val_len; i++, converted++) {
                headers[11].value[i] = *converted;
            }
            // temp_value = turn_deg < 0 ? -turn_deg : turn_deg;
            // unsigned char i;
            // char *converted = int_to_bcd_str(temp_value, sign, 1);
            // for (i = 0; i < val_len; i++, converted++) {
            //     headers[0].value[i] = *converted;
            // }
            // converted = int_to_bcd_str(temp_value, sign, 0);
            // for (i = 0; i < val_len; i++, converted++) {
            //     headers[1].value[i] = *converted;
            // }
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

char *int_to_bcd_str(unsigned int value, char sign, char frac_dig) {
    #define offset (val_len - 5)
    #define frac_comp (frac_dig > 0 ? 1 : 0)
    char digit;
    int factor = 10000;
    char first_non_zero_number = 0;
    static char result[val_len];
    result[0] = sign ? '+' : '-';
    unsigned char i;
    for (i = 0; i < offset; i++) {
        result[i + 1] = ' ';
    }
    for (i = 0; i < val_len - offset + frac_comp; i++) {
        if (i == val_len - offset - frac_dig) {
            result[i + offset - 1] = '.';
            continue;
        }
        digit = value / factor;
        if (digit == 0) {
            if (!first_non_zero_number) {
                if (i < val_len - offset - 1 - frac_dig) {
                    result[i + offset - frac_comp] = ' ';
                } else {
                    first_non_zero_number = 1;
                    result[i + offset - frac_comp] = 48;
                }
            } else {
                result[i + offset - frac_comp] = 48;
            }
        } else {
            first_non_zero_number = 1;
            result[i + offset - frac_comp] = digit + 48;
        }
        value = value % factor;
        factor = factor / 10;
    }
    return result;
}

char *long_to_bcd_str(unsigned long value, char sign, char frac_dig) {
    #define offset_l (val_len - 10)
    //4 292 967 295
    char digit;
    unsigned long factor = 1000000000;
    char first_non_zero_number = 0;
    static char result[val_len];
    result[0] = sign ? '+' : '-';
    unsigned char i;
    for (i = 0; i < offset_l; i++) {
        result[i + 1] = ' ';
    }
    for (i = 0; i < val_len - offset_l + frac_comp; i++) {
        if (i == val_len - offset_l - frac_dig) {
            result[i + offset_l - 1] = '.';
            continue;
        }
        digit = value / factor;
        if (digit == 0) {
            if (!first_non_zero_number) {
                if (i < val_len - offset_l - 1 - frac_dig) {
                    result[i + offset_l - frac_comp] = ' ';
                } else {
                    first_non_zero_number = 1;
                    result[i + offset_l - frac_comp] = 48;
                }
            } else {
                result[i + offset_l - frac_comp] = 48;
            }
        } else {
            first_non_zero_number = 1;
            result[i + offset_l - frac_comp] = digit + 48;
        }
        value = value % factor;
        factor = factor / 10;
    }
    return result;
}
