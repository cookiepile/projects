// Hardware Target//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// Blue LED:
//   PF2 drives an NPN transistor that powers the green LED
// Pushbutton:
//   SW1 pulls pin PF4 low (internal pull-up is used)
//On board:
//red led PA6
//GREEN LED PA7

// Red Backlight LED:
//   PB5 drives an NPN transistor that powers the red LED

// ST7565R Graphics LCD Display Interface:
//   MOSI (SSI2Tx) on PB7
//   MISO (SSI2Rx) is not used by the LCD display but the pin is used for GPIO for A0
//   SCLK (SSI2Clk) on PB4
//   A0 connected to PB6
//   ~CS connected to PB1



//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include <ctype.h>

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
/////onboard LEDs
#define redLed       (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))
#define greenLed     (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))
//LCD Backlight
#define RED_BL_LED   (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 5*4)))

#define CS_NOT       (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 1*4)))
#define A0           (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))


#define MAX_CHARS 80
#define MAX_MSGS 25
#define MAX_DATA 25
#define Broadcast_Address 255
#define Cmd_Set 0
#define Cmd_Reset 0x7F
#define Cmd_Get 0x20
#define Cmd_Data_Report 0x21
#define Cmd_Acknowledge 0x70
#define Cmd_Poll_Request 0x78
#define Cmd_Poll_Response 0x79
#define Cmd_Set_Address 0x7A
#define Cmd_RGB 0x48
#define Cmd_Pulse 0x02
#define Cmd_Square 0x03
#define Cmd_LCDtext 0xC0
#define MAX_RETRIES 5

uint8_t var_i = 0,var_n=0, cycleU=0, cycleL=0;
uint8_t s,lcdTextSize;
int16_t set=1,clear=0, T_ON1=0;
uint16_t T0=100, T=500, var_t=0, cycle=0, var_t1=0, var_t2=0;
int32_t T_ON=-1; // T_ON_ms=0, T_OFF_ms1=0, T_OFF1=0;
uint8_t T_ON_ms1=0, T_ON_ms2=0, T1_ON_ms1=0, T1_ON_ms2=0, T2_ON_ms1=0, T2_ON_ms2=0;

uint8_t msgSizeRx; //this can be maximum as MAX_DATA (25)

uint8_t sizeTx; //this can be maximum as MAX_DATA+7
// dst_adr, src_adr, seqid, command, channel, size, checksum

int16_t address,channel,value,newAddress,validity;
int16_t rgbData[3];
int16_t squareData[8];
//uint8_t address,channel,value,newAddress,validity;
uint8_t field;
uint8_t position[10];
bool csEnable,randomRetrans,ackon;
bool inProgress= false;
bool transmitError=false;
bool printErrorTxMsg=false;
bool printTxAttemptMsg=false;
uint8_t tempInt, tempInt1;
char str[MAX_CHARS+1],str2[MAX_CHARS+1];

uint8_t count, checksum, checksumRx;
uint8_t currentIndex, currentPhase=0, rxPhase=0, oldRxPhase=0, oldTxPhase=0;
uint16_t sequenceId, ledTimeout,  deadlockTimeoutRx, deadlockTimeoutTx;
uint16_t retransTimeOut[MAX_MSGS];
uint8_t sourceAddress=70;

int16_t sourceAdress_8bit;

int16_t rxData[MAX_DATA+7]; //apart from data there are 7 overhead bytes
                            // dst_adr, src_adr, seqid, command, channel, size, checksum

int16_t destAdd[MAX_MSGS],chan[MAX_MSGS];
int16_t data[MAX_MSGS][MAX_DATA];
bool valid[MAX_MSGS],ackRequired[MAX_MSGS];
uint8_t seqId[MAX_MSGS],cmd[MAX_MSGS],size[MAX_MSGS],checkSum[MAX_MSGS],retransCount[MAX_MSGS];
char LCDdata[30];
char lcdText;

//char str3[80]={};

/////////LCD global variables
uint8_t  pixelMap[1024];
uint16_t txtIndex = 0;

// 96 character 5x7 bitmaps based on ISO-646 (BCT IRV extensions)
const uint8_t charGen[100][5] = {
    // Codes 32-127
    // Space ! " % $ % & ' ( ) * + , - . /
    {0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x4F, 0x00, 0x00},
    {0x00, 0x07, 0x00, 0x07, 0x00},
    {0x14, 0x7F, 0x14, 0x7F, 0x14},
    {0x24, 0x2A, 0x7F, 0x2A, 0x12},
    {0x23, 0x13, 0x08, 0x64, 0x62},
    {0x36, 0x49, 0x55, 0x22, 0x40},
    {0x00, 0x05, 0x03, 0x00, 0x00},
    {0x00, 0x1C, 0x22, 0x41, 0x00},
    {0x00, 0x41, 0x22, 0x1C, 0x00},
    {0x14, 0x08, 0x3E, 0x08, 0x14},
    {0x08, 0x08, 0x3E, 0x08, 0x08},
    {0x00, 0x50, 0x30, 0x00, 0x00},
    {0x08, 0x08, 0x08, 0x08, 0x08},
    {0x00, 0x60, 0x60, 0x00, 0x00},
    {0x20, 0x10, 0x08, 0x04, 0x02},
    // 0-9
    {0x3E, 0x51, 0x49, 0x45, 0x3E},
    {0x00, 0x42, 0x7F, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46},
    {0x21, 0x41, 0x45, 0x4B, 0x31},
    {0x18, 0x14, 0x12, 0x7F, 0x10},
    {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3C, 0x4A, 0x49, 0x49, 0x30},
    {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36},
    {0x06, 0x49, 0x49, 0x29, 0x1E},
    // : ; < = > ? @
    {0x00, 0x36, 0x36, 0x00, 0x00},
    {0x00, 0x56, 0x36, 0x00, 0x00},
    {0x08, 0x14, 0x22, 0x41, 0x00},
    {0x14, 0x14, 0x14, 0x14, 0x14},
    {0x00, 0x41, 0x22, 0x14, 0x08},
    {0x02, 0x01, 0x51, 0x09, 0x3E},
    {0x32, 0x49, 0x79, 0x41, 0x3E},
    // A-Z
    {0x7E, 0x11, 0x11, 0x11, 0x7E},
    {0x7F, 0x49, 0x49, 0x49, 0x36},
    {0x3E, 0x41, 0x41, 0x41, 0x22},
    {0x7F, 0x41, 0x41, 0x22, 0x1C},
    {0x7F, 0x49, 0x49, 0x49, 0x41},
    {0x7F, 0x09, 0x09, 0x09, 0x01},
    {0x3E, 0x41, 0x49, 0x49, 0x3A},
    {0x7F, 0x08, 0x08, 0x08, 0x7F},
    {0x00, 0x41, 0x7F, 0x41, 0x00},
    {0x20, 0x40, 0x41, 0x3F, 0x01},
    {0x7F, 0x08, 0x14, 0x22, 0x41},
    {0x7F, 0x40, 0x40, 0x40, 0x40},
    {0x7F, 0x02, 0x0C, 0x02, 0x7F},
    {0x7F, 0x04, 0x08, 0x10, 0x7F},
    {0x3E, 0x41, 0x41, 0x41, 0x3E},
    {0x7F, 0x09, 0x09, 0x09, 0x06},
    {0x3E, 0x41, 0x51, 0x21, 0x5E},
    {0x7F, 0x09, 0x19, 0x29, 0x46},
    {0x46, 0x49, 0x49, 0x49, 0x31},
    {0x01, 0x01, 0x7F, 0x01, 0x01},
    {0x3F, 0x40, 0x40, 0x40, 0x3F},
    {0x1F, 0x20, 0x40, 0x20, 0x1F},
    {0x3F, 0x40, 0x70, 0x40, 0x3F},
    {0x63, 0x14, 0x08, 0x14, 0x63},
    {0x07, 0x08, 0x70, 0x08, 0x07},
    {0x61, 0x51, 0x49, 0x45, 0x43},
    // [ \ ] ^ _ `
    {0x00, 0x7F, 0x41, 0x41, 0x00},
    {0x02, 0x04, 0x08, 0x10, 0x20},
    {0x00, 0x41, 0x41, 0x7F, 0x00},
    {0x04, 0x02, 0x01, 0x02, 0x04},
    {0x40, 0x40, 0x40, 0x40, 0x40},
    {0x00, 0x01, 0x02, 0x04, 0x00},
    // a-z
    {0x20, 0x54, 0x54, 0x54, 0x78},
    {0x7F, 0x44, 0x44, 0x44, 0x38},
    {0x38, 0x44, 0x44, 0x44, 0x20},
    {0x38, 0x44, 0x44, 0x48, 0x7F},
    {0x38, 0x54, 0x54, 0x54, 0x18},
    {0x08, 0x7E, 0x09, 0x01, 0x02},
    {0x0C, 0x52, 0x52, 0x52, 0x3E},
    {0x7F, 0x08, 0x04, 0x04, 0x78},
    {0x00, 0x44, 0x7D, 0x40, 0x00},
    {0x20, 0x40, 0x44, 0x3D, 0x00},
    {0x7F, 0x10, 0x28, 0x44, 0x00},
    {0x00, 0x41, 0x7F, 0x40, 0x00},
    {0x7C, 0x04, 0x18, 0x04, 0x78},
    {0x7C, 0x08, 0x04, 0x04, 0x78},
    {0x38, 0x44, 0x44, 0x44, 0x38},
    {0x7C, 0x14, 0x14, 0x14, 0x08},
    {0x08, 0x14, 0x14, 0x18, 0x7C},
    {0x7C, 0x08, 0x04, 0x04, 0x08},
    {0x48, 0x54, 0x54, 0x54, 0x20},
    {0x04, 0x3F, 0x44, 0x40, 0x20},
    {0x3C, 0x40, 0x40, 0x20, 0x7C},
    {0x1C, 0x20, 0x40, 0x20, 0x1C},
    {0x3C, 0x40, 0x20, 0x40, 0x3C},
    {0x44, 0x28, 0x10, 0x28, 0x44},
    {0x0C, 0x50, 0x50, 0x50, 0x3C},
    {0x44, 0x64, 0x54, 0x4C, 0x44},
    // { | } ~ cc
    {0x00, 0x08, 0x36, 0x41, 0x00},
    {0x00, 0x00, 0x7F, 0x00, 0x00},
    {0x00, 0x41, 0x36, 0x08, 0x00},
    {0x0C, 0x04, 0x1C, 0x10, 0x18},
    {0x00, 0x00, 0x00, 0x00, 0x00},
    // Custom assignments beyond ISO646
    // Codes 128+: right arrow, left arrow, degree sign
    {0x08, 0x08, 0x2A, 0x1C, 0x08},
    {0x08, 0x1C, 0x2A, 0x08, 0x08},
    {0x07, 0x05, 0x07, 0x00, 0x00},
};

initPWM()
    {
    //Configure GPIO FOR PWM
    GPIO_PORTF_AFSEL_R |= 0x0E; // select auxilary function for bits 1,2 and 3
    GPIO_PORTF_PCTL_R = GPIO_PCTL_PF1_M1PWM5|GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7; // enable PWM on bits 1,2 and 3
    // Configure PWM module1 to drive RGB
    // RED   on M0PWM5 (PF1), M0PWM2b
    // BLUE  on M0PWM6 (PF2), M0PWM3a
    // GREEN on M0PWM7 (PF3), M0PWM3b
    SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM0;             // turn-on PWM module
    SYSCTL_RCGCPWM_R |= 0x02;
    __asm(" NOP");                                   // wait 3 clocks
    __asm(" NOP");
    __asm(" NOP");
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM0 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM1_2_CTL_R = 0;                                // turn-off PWM1 generator 2
    PWM1_3_CTL_R = 0;                                // turn-off PWM1 generator 3
    PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;
                                                     // output 1 on PWM1, gen 2b, cmpb
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ZERO | PWM_1_GENA_ACTLOAD_ONE;
                                                     // output 2 on PWM1, gen 3a, cmpa
    PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;
                                                     // output 3 on PWM1, gen 3b, cmpb
    PWM1_2_LOAD_R = 1024;                            // set period to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM1_3_LOAD_R = 1024;
    PWM1_INVERT_R = PWM_INVERT_PWM5INV | PWM_INVERT_PWM6INV | PWM_INVERT_PWM7INV;
                                                     // invert outputs for duty cycle increases with increasing compare values
    PWM1_2_CMPB_R = 0;                               // red off (0=always low, 1023=always high)
    PWM1_3_CMPB_R = 0;                               // green off
    PWM1_3_CMPA_R = 0;                               // blue off

    PWM1_2_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 2
    PWM1_3_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 3
    PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
                                                     // enable outputs
    }

void clearPWM()
{
    GPIO_PORTF_AFSEL_R &= ~(0x0E);
    GPIO_PORTF_PCTL_R &=~(GPIO_PCTL_PF1_M1PWM5|GPIO_PCTL_PF2_M1PWM6|GPIO_PCTL_PF3_M1PWM7);
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM0 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM1_2_CTL_R = 0;                                // turn-off PWM0 generator 1
    PWM1_3_CTL_R = 0;                                // turn-off PWM0 generator 2
    PWM1_ENABLE_R &=~( PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN);
}

void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S) | SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_2;;

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A, C and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF |SYSCTL_RCGC2_GPIOC |SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOE;

    //Enable and check EEPROM
    SYSCTL_RCGCEEPROM_R= SYSCTL_RCGCEEPROM_R0;
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);


    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0E;  // bit 1,2,3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0E; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x1E;  // enable LED and push button
    GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button
    GPIO_PORTA_DIR_R = 0xC0;  // bits 6 and 7 are outputs, other pins are inputs
    GPIO_PORTA_DR2R_R = 0xC0; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R = 0xC0;

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    //Configure UART1 pins using portC
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;         // turn-on UART1, leave other uarts in same status
    GPIO_PORTC_DIR_R |=0x40;
    GPIO_PORTC_DEN_R |= 0x70;                           // default, added for clarity
    GPIO_PORTC_AFSEL_R |= 0x30;                         // default, added for clarity
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;


    // Configure UART1 to 38400 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART1_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART1_IBRD_R = 65;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART1_FBRD_R = 7;                               // round(fract(r)*64)=45
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN| UART_LCRH_PEN| UART_LCRH_SPS| UART_LCRH_EPS; // configure for 8N1 w/ 16-level FIFO
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // Configure Timer 1 as the time base
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 0x9C40;                      // set load value to 40e6 for 1k Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

    // Configure three backlight LEDs
    GPIO_PORTB_DIR_R |= 0x20;  // make bit5 an output
    GPIO_PORTB_DR2R_R |= 0x20; // set drive strength to 2mA
    GPIO_PORTB_DEN_R |= 0x20;  // enable bit5 for digital
    GPIO_PORTE_DIR_R |= 0x30;  // make bits 4 and 5 outputs
    GPIO_PORTE_DR2R_R |= 0x30; // set drive strength to 2mA
    GPIO_PORTE_DEN_R |= 0x30;  // enable bits 4 and 5 for digital

    // Configure A0 and ~CS for graphics LCD
    GPIO_PORTB_DIR_R |= 0x42;  // make bits 1 and 6 outputs
    GPIO_PORTB_DR2R_R |= 0x42; // set drive strength to 2mA
    GPIO_PORTB_DEN_R |= 0x42;  // enable bits 1 and 6 for digital

    // Configure SSI2 pins for SPI configuration
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;           // turn-on SSI2 clocking
    GPIO_PORTB_DIR_R |= 0x90;                        // make bits 4 and 7 outputs
    GPIO_PORTB_DR2R_R |= 0x90;                       // set drive strength to 2mA
    GPIO_PORTB_AFSEL_R |= 0x90;                      // select alternative functions for MOSI, SCLK pins
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB4_SSI2CLK; // map alt fns to SSI2
    GPIO_PORTB_DEN_R |= 0x90;                        // enable digital operation on TX, CLK pins

    // Configure the SSI2 as a SPI master, mode 3, 8bit operation, 1 MHz bit rate
    SSI2_CR1_R &= ~SSI_CR1_SSE;                      // turn off SSI2 to allow re-configuration
    SSI2_CR1_R = 0;                                  // select master mode
    SSI2_CC_R = 0;                                   // select system clock as the clock source
    SSI2_CPSR_R = 40;                                // set bit rate to 1 MHz (if SR=0 in CR0)
    SSI2_CR0_R = SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_8; // set SR=0, mode 3 (SPH=1, SPO=1), 8-bit
    SSI2_CR1_R |= SSI_CR1_SSE;                       // turn on SSI2
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

///LCD Functions

// Blocking function that writes data to the SPI bus and waits for the data to complete transmission
void sendGraphicsLcdCommand(uint8_t command)
{
    CS_NOT = 0;                        // assert chip select
    __asm (" NOP");                    // allow line to settle
    __asm (" NOP");
    __asm (" NOP");
    __asm (" NOP");
    A0 = 0;                            // clear A0 for commands
    SSI2_DR_R = command;               // write command
    while (SSI2_SR_R & SSI_SR_BSY);
    CS_NOT = 1;                        // de-assert chip select
}

// Blocking function that writes data to the SPI bus and waits for the data to complete transmission
void sendGraphicsLcdData(uint8_t data)
{
    CS_NOT = 0;                        // assert chip select
    __asm (" NOP");                    // allow line to settle
    __asm (" NOP");
    __asm (" NOP");
    __asm (" NOP");
    A0 = 1;                            // set A0 for data
    SSI2_DR_R = data;                  // write data
    while (SSI2_SR_R & SSI_SR_BSY);    // wait for transmission to stop
    CS_NOT = 1;                        // de-assert chip select
}

void setGraphicsLcdPage(uint8_t page)
{
  sendGraphicsLcdCommand(0xB0 | page);
}

void setGraphicsLcdColumn(uint8_t x)
{
  sendGraphicsLcdCommand(0x10 | ((x >> 4) & 0x0F));
  sendGraphicsLcdCommand(0x00 | (x & 0x0F));
}

void refreshGraphicsLcd()
{
    uint8_t x, page;
    uint16_t i = 0;
    for (page = 0; page < 8; page ++)
    {
        setGraphicsLcdPage(page);
        setGraphicsLcdColumn(0);
        for (x = 0; x < 128; x++)
            sendGraphicsLcdData(pixelMap[i++]);
    }
}

void clearGraphicsLcd()
{
    uint16_t i;
    // clear data memory pixel map
    for (i = 0; i < 1024; i++)
        pixelMap[i] = 0;
    // copy to display
    refreshGraphicsLcd();
}

void initGraphicsLcd()
{
    sendGraphicsLcdCommand(0x40); // set start line to 0
    sendGraphicsLcdCommand(0xA1); // reverse horizontal order
    sendGraphicsLcdCommand(0xC0); // normal vertical order
    sendGraphicsLcdCommand(0xA6); // normal pixel polarity
    sendGraphicsLcdCommand(0xA3); // set led bias to 1/9 (should be A2)
    sendGraphicsLcdCommand(0x2F); // turn on voltage booster and regulator
    sendGraphicsLcdCommand(0xF8); // set internal volt booster to 4x Vdd
    sendGraphicsLcdCommand(0x00);
    sendGraphicsLcdCommand(0x27); // set contrast
    sendGraphicsLcdCommand(0x81); // set LCD drive voltage
    sendGraphicsLcdCommand(0x04);
    sendGraphicsLcdCommand(0xAC); // no flashing indicator
    sendGraphicsLcdCommand(0x00);
    clearGraphicsLcd();           // clear display
    sendGraphicsLcdCommand(0xAF); // display on
}

void setGraphicsLcdTextPosition(uint8_t x, uint8_t page)
{
    txtIndex = (page << 7) + x;
    setGraphicsLcdPage(page);
    setGraphicsLcdColumn(x);
}

void putcGraphicsLcd(char c)
{
    uint8_t i, val;
    uint8_t uc;
    // convert to unsigned to access characters > 127
    uc = (uint8_t) c;
    for (i = 0; i < 5; i++)
    {
        val = charGen[uc-' '][i];
        pixelMap[txtIndex++] = val;
        sendGraphicsLcdData(val);
    }
    pixelMap[txtIndex++] = 0;
    sendGraphicsLcdData(0);
}

void putsGraphicsLcd(char str[])
{
    uint8_t i = 0;
    while (str[i] != 0)
        putcGraphicsLcd(str[i++]);
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}

char itoa(uint8_t x)
{
  //  char a;
    switch(x)
    {
    case 0: {return '0';}
    case 1: {return '1';}
    case 2: {return '2';}
    case 3: {return '3';}
    case 4: {return '4';}
    case 5: {return '5';}
    case 6: {return '6';}
    case 7: {return '7';}
    case 8: {return '8';}
    case 9: {return '9';}
    default: { putsUart0("Error in switch case in itoa function");
               return '\0';}

    }
}

void putiUart0(uint16_t x)
{
    uint8_t i=0;
    int16_t j;
    char i_str[6];
    if(x<10)
    {
        putcUart0(itoa(x));
    }
    else
    {
        while(x>0)
        {
            i_str[i]=itoa(x%10);
            x=x/10;
            if(x!=0)
            {
                i++;
            }
        }
        for(j=i; j>=0; j--)
        {
            putcUart0(i_str[j]);
        }
    }
}



void clearString(char str[], uint8_t max)
{
    uint8_t i;
    for(i=0; i<=max; i++)
    {
        str[i]='\0';
    }
}

void clearArray(uint8_t array[], uint8_t max)
{
    uint8_t i;
    for(i=0; i<=max; i++)
    {
        array[i]='\0';
    }
}

char* getString(uint8_t field)
{
    return &str[position[field]];
}

int16_t getNumber(uint8_t field)
{
    int16_t i;
    //sscanf(i, "%d", &str[position[field]]);
    i=atoi(&str[position[field]]);
    return i;
}

bool isCommand(char strCmd[],uint8_t minArgs)
          {
             bool a;
             if((strcmp(&str[position[0]],strCmd))==0)
             {
              if(field>=minArgs)
              a=true;
             }
             else
              a=false;
             return a;
          }

// Blocking function that writes a serial byte when the UART1 buffer is not full
void putbUart1(char b)
{
    while (UART1_FR_R & UART_FR_TXFF);
    UART1_DR_R = b;
}

char getbUart1()
{
    while (UART1_FR_R & UART_FR_RXFE);
    return UART1_DR_R;
}

void setRgbColor(uint16_t red, uint16_t green, uint16_t blue)
{
    PWM1_2_CMPB_R = red;
    PWM1_3_CMPA_R = blue;
    PWM1_3_CMPB_R = green;
}

//void sendPacket(uint8_t destinationAddress, uint8_t command, uint8_t channel, uint8_t Size, uint8_t Data[])
void sendPacket(int16_t destinationAddress, uint8_t command, int16_t channel, uint8_t Size, int16_t Data[])
{
    uint8_t a,x,s;
    bool foundSpace=false;
    s=0;
    for(x=0;x<=MAX_MSGS;x++)
    {
        if(valid[x]==false) // while(valid[x]==false && x<=MAX_MSGS)
        {
              destAdd[x]=destinationAddress;
              chan[x]=channel;
              size[x]=Size;
              if(ackon==true)
              {
                   ackRequired[x]=ackon;
                   command|=0x80;
              }
              else
              {
                   ackRequired[x]=false;
              }

              for(a=0;a<Size;a++)
              {
                  data[x][a]=Data[a];
                  s=s+Data[a];
              }
              s=s+destinationAddress+sourceAddress+command+channel+Size+sequenceId;
              checksum=s^0xFF;
              putsUart0("\r\nTx checksum calculated");
              putiUart0(checksum);
              checkSum[x]=checksum;
              seqId[x]=sequenceId;
              sequenceId++;
              cmd[x]=command;
              retransCount[x]=0;
              retransTimeOut[x]=0;
              valid[x]=true;
              foundSpace=true;
              break;
        }

    }
    if(foundSpace==false)
        putsUart0("\r\nError\r\n");
}


void transmitMsg()
{
    int r,i;
    if(!inProgress)
    {
        for(i=0;i<=MAX_MSGS;i++)
        {
            if(valid[i]==true && (retransTimeOut[i]==0))
            {
                inProgress=true;
                currentIndex=i;
                currentPhase=0;
                r=rand()%4;
                if(randomRetrans==true)
                    retransTimeOut[currentIndex]=((T0)+(r*(2^retransCount[i])*T)); //T is 500ms and T0 is 100 ms
                else
                    retransTimeOut[currentIndex]=((T0)+((2^retransCount[i])*T));
                break;
            }
        }
    }

    if(inProgress)
    {
        GPIO_PORTC_DATA_R |= 0x040;

        int a=0;
           if(currentPhase==0)
           {
               if(((csEnable==false) || (rxPhase==0))&&((UART1_FR_R & UART_FR_BUSY)==0))
               {
                   UART1_LCRH_R &=~0x04;
                   UART1_DR_R= destAdd[currentIndex];

                   currentPhase=1;
                   // the following code has been added acc to the PDF of project refer pg5, para 4
                   putsUart0("\r\n Transmitting Msg N= ");
                   putiUart0(seqId[currentIndex]);
                   putsUart0(" , Attempt M= ");
                   putiUart0(retransCount[currentIndex]);
                   putsUart0("\r\n");
                   printTxAttemptMsg=true;

               }
           }
           if(currentPhase==1)
           {
               if((UART1_FR_R & UART_FR_BUSY) ==0)
               {
                   UART1_LCRH_R |=0x04;
                   UART1_DR_R= sourceAddress;
                   currentPhase++;
               }
           }
           else
           {
            UART1_DR_R=seqId[currentIndex];
            currentPhase++;
            UART1_DR_R=cmd[currentIndex];
            currentPhase++;
            UART1_DR_R=chan[currentIndex];
            currentPhase++;
            UART1_DR_R=size[currentIndex];
            currentPhase++;
            sizeTx = size[currentIndex]+7;  //adding the overhead bytes
            for(a=0;a<size[currentIndex];a++)
            {
                UART1_DR_R=data[currentIndex][a];
                currentPhase++;
            }

            UART1_DR_R=checkSum[currentIndex];
            currentPhase++;
           }

       if(currentPhase==sizeTx)
        {
           inProgress=false;
           currentPhase=0;
           redLed=1;
           ledTimeout=70;
           if(ackRequired[currentIndex]==false)
               valid[currentIndex]=false;
           else
           {
               retransCount[currentIndex]++;
           //    putiUart0(retransCount[currentIndex]);
           }
        }
       oldTxPhase=currentPhase;
    }



}

void processMsg()
{
    if((rxData[3] & 0x80)== 0x80) ////if ack requested checking if MSB set
    {
        sendPacket(rxData[1], Cmd_Acknowledge, 0x00, 0x01, &rxData[2]);
    }

    if(rxData[3]==Cmd_Acknowledge)
    {
        uint8_t var_k;
        for(var_k=0; var_k<MAX_MSGS; var_k++)
        {
            if((seqId[var_k]==rxData[6]))
            {
                valid[var_k]=false;
            }
        }
    }

    if((rxData[3]==Cmd_Set)||(rxData[3]==0x80))
    {
        if(rxData[4]==136 && rxData[6]==1)
            BLUE_LED =1;
        if(rxData[4]==136 && rxData[6]==0)
            BLUE_LED =0;
    }

    if((rxData[3]==Cmd_Poll_Request)||(rxData[3]==0xF8))
    {
        sendPacket(rxData[1], Cmd_Poll_Response, 0x00, 0x01, &sourceAdress_8bit);
    }

    if((rxData[3]==Cmd_Poll_Response)||(rxData[3]==0xF9))
    {
        putsUart0("\r\nAddress Received= ");
        putiUart0(rxData[6]);
        putsUart0("\r\n");
    }

    if((rxData[3]==Cmd_Get)||(rxData[3]==0xA0))
    {
        if(rxData[4]==96)
        {
            if((GPIO_PORTF_DATA_R & 0x10)==0)
                sendPacket(rxData[1], Cmd_Data_Report,0x00,0x01,&set);
            else
                sendPacket(rxData[1], Cmd_Data_Report,0x00,0x01,&clear);
        }
        if(rxData[4]==136)
        {
            if(BLUE_LED==1)
                sendPacket(rxData[1], Cmd_Data_Report,0x00,0x01,&set);
            else
                sendPacket(rxData[1], Cmd_Data_Report,0x00,0x01,&clear);
        }
    }


    if((rxData[3]==Cmd_Data_Report)||(rxData[3]==0xA1))
    {
        putsUart0("\r\nData Report from Address = ");
        putiUart0(rxData[1]);
        putsUart0(" Data = ");
        putiUart0(rxData[6]);
        if(rxData[6]==0)
            putsUart0("\r\npin is not set\r\n");
        else
            putsUart0("\r\npin is set\r\n");
    }

    if((rxData[3]==Cmd_Set_Address)||(rxData[3]==0xFA))
       {
           while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
           EEPROM_EEBLOCK_R = 1;//EEPROM block number
           EEPROM_EEOFFSET_R= 0;//offset for the block number
           EEPROM_EERDWRINC_R =rxData[6];//write to EEPROM
           while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);//wait till its done
           sourceAddress=rxData[6];//set the source address with requested address
           putsUart0("\r\nMy address is changed to ");
           putiUart0(rxData[6]);
           putsUart0("\r\n");
       }

    if((rxData[3]==Cmd_RGB)||(rxData[3]==0xC8))
    {
        if(rxData[4]==50)
        {
        initPWM();
        setRgbColor(rxData[6], rxData[7], rxData[8]);
        putsUart0("\r\nThe value set for red, blue and green is ");
        putiUart0(rxData[6]);
        putsUart0(" ");
        putiUart0(rxData[7]);
        putsUart0(" ");
        putiUart0(rxData[8]);
        putsUart0(" ");
        putsUart0("\r\n");
        waitMicrosecond(5000000);//Enough time to observe PWM output on LEDs
        clearPWM();
        }
    }
    if((rxData[3]==Cmd_Pulse)||(rxData[3]==0x82))
    {
        if(rxData[4]==6)
        {
            T_ON_ms1=rxData[7];
            T_ON_ms2=rxData[8];
            var_t=(T_ON_ms1<<8)|T_ON_ms2;

            initPWM();
            setRgbColor(rxData[6],0,0);
            T_ON=var_t*10;


            putsUart0("\r\nThe value of pulse is ");
            putiUart0(rxData[6]);
            putsUart0(" ");

            putsUart0("\r\nUpper byte is ");
            putiUart0(rxData[7]);
            putsUart0(" ");
            putsUart0("\r\nLower byte is ");
            putiUart0(rxData[8]);
            putsUart0(" ");
            putsUart0("\r\n");
        }
    }
    if((rxData[3]==Cmd_LCDtext)||(rxData[3]==0x40))
     {
          if(rxData[4]==3)
          {
              initGraphicsLcd();
              setGraphicsLcdTextPosition(0, 100);
              putsGraphicsLcd("Hey There!!");

          }
      }

    if((rxData[3]==Cmd_Square)||(rxData[3]==0x83))
    {
        if(rxData[4]==6)
        {
            int i;
            cycleU=rxData[12];
            cycleL=rxData[13];
            cycle=(cycleU<<8)|cycleL;
            T1_ON_ms1=rxData[8];
            T1_ON_ms2=rxData[9];
            var_t1=(T1_ON_ms1<<8)|T1_ON_ms2;
            T2_ON_ms1=rxData[10];
            T2_ON_ms2=rxData[11];
            var_t2=(T2_ON_ms1<<8)|T2_ON_ms2;
            for(i=0;i<cycle;i++)
            {
                    initPWM();
                    setRgbColor(rxData[6],0,0);
                    waitMicrosecond(var_t1*1000);
                    clearPWM();

                    initPWM();
                    setRgbColor(0,0,rxData[7]);
                    waitMicrosecond(var_t2*1000);
                    clearPWM();

            }

            putsUart0("\r\n value1 is ");
            putiUart0(rxData[6]);
            putsUart0(" ");
            putsUart0("\r\nT1 Upper byte is ");
            putiUart0(rxData[8]);
            putsUart0(" ");
            putsUart0("\r\nT1 Lower byte is ");
            putiUart0(rxData[9]);
            putsUart0(" ");
            putsUart0("\r\n");
            putsUart0("\r\n value2 is ");
            putiUart0(rxData[7]);
            putsUart0(" ");
            putsUart0("\r\nT2 Upper byte is ");
            putiUart0(rxData[10]);
            putsUart0(" ");
            putsUart0("\r\nT2 Lower byte is ");
            putiUart0(rxData[11]);
            putsUart0(" ");
            putsUart0("\r\n");
            putsUart0("\r\ncycle upper is ");
            putiUart0(rxData[12]);
            putsUart0(" ");
            putsUart0("\r\ncycle lower is ");
            putiUart0(rxData[13]);
            putsUart0(" ");
            putsUart0("\r\n");
        }
    }

    if((rxData[3]==Cmd_Reset)||(rxData[3]==0xFF))
   {
        putsUart0("\r\nThe board is going to reset\r\n");
        NVIC_APINT_R=NVIC_APINT_VECTKEY|NVIC_APINT_SYSRESETREQ;
   }

}

void receiveMsg()
{
    if((UART1_FR_R & UART_FR_RXFE)==0)
    {
        uint8_t temp2=(UART1_LCRH_R & 0x04)>>2;
        uint16_t temp=UART1_DR_R;
        uint8_t data1=temp & 0xFF;
        uint16_t temp1=temp & 0x200;
        uint8_t bit9=temp2 ^(temp1>>9);

        if((bit9==0) && (currentPhase==0))
        {
            rxPhase=0;
            rxData[rxPhase]=data1;
             // capturing destination address
            if((data1==sourceAddress)||(data1==Broadcast_Address)) // match if we are the recipient of the data
            {
                s=0;
                rxPhase++;
            }
        }
        else
        {
            if((rxPhase!=0) && (currentPhase==0))
            {
                rxData[rxPhase]=data1; // source address in [1]
                if(rxPhase == 5)
                {
                    msgSizeRx = rxData[rxPhase];
                }
                rxPhase++;
            }

            if(rxPhase==(7+msgSizeRx))            ////msg receive size+overhead of 7
            {
                rxPhase=0;
                uint8_t var_j;
                for(var_j=0;var_j<=5+msgSizeRx;var_j++)
                {
                    s=s+rxData[var_j];
                }
                greenLed=1;
                ledTimeout=70;

                checksumRx=s^0xFF;

                putsUart0("\r\nchecksum received");
                putiUart0(rxData[msgSizeRx+6]);
                putsUart0("\r\nRxcalculated checksum");
                putiUart0(checksumRx);

                if(checksumRx!=rxData[msgSizeRx+6])
                {
                   greenLed=1;
                   ledTimeout=2000;
                }

                //sprintf(str3,"\r\nMy address is %d\r\nTransmitter address is %d\r\nsequenceId is %d\r\ncommand is%d\r\nchannel is %d\r\nsize is %d\r\ndata is %d\r\nchecksum is %d\r\n", rxData[0], rxData[1], rxData[2], rxData[3], rxData[4], rxData[5], rxData[6], rxData[7]);
                //putsUart0(str3);

                processMsg();

            }
        }
        oldRxPhase=rxPhase;
    }

}

void deadlock()
{
    if(oldRxPhase != rxPhase)
        deadlockTimeoutRx++;

    else if(deadlockTimeoutRx>5)
        rxPhase=0;

    else if(oldTxPhase != currentPhase)
        deadlockTimeoutTx++;

    else if(deadlockTimeoutTx>10)
        currentPhase=0;

    else
    {
        deadlockTimeoutRx=0;
        deadlockTimeoutTx=0;
    }

}

// Frequency counter service publishing latest frequency measurements every second
void Timer1Isr()
{
    //GPIO_PORTF_DATA_R |= 0x8;
    //BLUE_LED ^= 1;                              // status
    uint8_t var_k;

    if(validity==1)
    {
        putsUart0("\r\nQueuing Msg N= ");
        putiUart0(sequenceId);
        validity++;
    }

    transmitMsg();

    if(valid[currentIndex]==true && (retransCount[currentIndex]>MAX_RETRIES))
     {
        valid[currentIndex]=false;
        redLed=1;
        ledTimeout=2000;
        //if(transmitError==true)
        putsUart0("\r\n Error Sending Msg N= ");
        putiUart0(seqId[currentIndex]);

        printErrorTxMsg=true;
     }


    if((UART1_FR_R & UART_FR_BUSY)==0)
        GPIO_PORTC_DATA_R &=~0x40;

    /////////receiving////////
    receiveMsg();
    deadlock();
    //////led timeout code////
    if(ledTimeout>0)
        ledTimeout--;

    if(ledTimeout==0)
    {
        redLed=0;
        greenLed=0;
    }

    for(var_k=0; var_k<MAX_MSGS; var_k++)
    {
        if(retransTimeOut[var_k]>0)
        retransTimeOut[var_k]--;
    }

    //////pulse timeout//////

    if(T_ON>0)
    {
       T_ON--;
    }
    if(T_ON==0)
    {
      clearPWM();
    }



    TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}


//-------------------------------------main function----------------------------------------------/////
void main(void)
{
    char c,c0,c1;
    char type[MAX_CHARS+1];
    uint8_t x,b,b1;
    uint8_t i=0;
    sourceAdress_8bit = sourceAddress; //to clear warning of incompatible uint8_t with int8_t
                                //while send packet is called inside Poll respond inside receive msg



    // Initialize hardware
        initHw();
        RED_BL_LED = 1;
        initGraphicsLcd();
        // Draw text on screen
        setGraphicsLcdTextPosition(0, 0);
        putsGraphicsLcd("Final Project by Neha");

        for(x=0;x<MAX_MSGS;x++)
        {
            valid[x]=false;
        }
        sequenceId=0;
        // Turn on and off green LED for 500 ms
        GPIO_PORTF_DATA_R |= 0x8;
        //greenLed=1;
        waitMicrosecond(500000);
        //ugreenLed=0;
        GPIO_PORTF_DATA_R &= ~0x8;
        waitMicrosecond(500000);

        // Display greeting
        putsUart0("\r\nFinal Project by Neha\r\n");


        while(1)
        {
        //enter string
            putsUart0("\r\nReady\r\n");
            validity=0;
            count=0;
            field=0;
            i=0;
            //position=0;
            clearString(str,MAX_CHARS);
            clearString(type,MAX_CHARS);
            clearArray(position,10);

            while(count<=MAX_CHARS)
            {
                c=getcUart0();
                b=(uint8_t) c; //ascii value of character stores in b

                if(b==8) //in case of backspace
                {
                    if(count>0)  //in case backspace is not the first character
                    {
                        count--;
                    }
                }

                if(b==13) //in case of c/r
                {
                    str[count++]='\0';
                    break;
                }

                if(b>=0x20) //all valid characters for a string
                {
                    c0=tolower(c);
                    str[count++]=c0;
                }

            }

            //discard code
           putsUart0("\r\n");
           putsUart0(str);
           putsUart0("\r\n");

            //step 3
            while(i<count)
            {
                 c1=str[i];
                 b1=(uint8_t) c1;

                  if(b1>=48 && b1<=57)
                  {
                      type[i]='n';
                      putcUart0(type[i]);
                  }

                  else if((b1>=65 && b1<=90) || (b1>=97 && b1<=122))
                  {
                      type[i]='a';
                      putcUart0(type[i]);
                  }

                  else
                  {
                      type[i]='d';
                      putcUart0(type[i]);
                      str[i]='\0';
                  }

                  if(type[i]!=type[i-1] && i>0 && (type[i]=='a'|| type[i]=='n'))
                  {
                         if (type[0]=='a'|| type[0]=='n')
                         {
                             if (field==0)
                             {
                                 position[field]=0;
                                 field++;
                             }
                             position[field]=i;
                             field++;
                         }
                         else
                         {
                             position[field]=i;
                             field++;
                         }
                  }

                 i++;
            }

            //discard code
            for(i=0;i<=(field-1);i++)
            {
                    if(type[position[i]]=='a')
                    {
                        sprintf(str2,"%u,alpha,%s\n",i,&str[position[i]]);
                    }
                    else
                    {
                        sprintf(str2,"%u,number,%s\n",i,&str[position[i]]);
                    }

                putsUart0("\r\n");
                putsUart0(str2);
            }

            ///step 4

            if(isCommand("set",3))
            {
                address=getNumber(1);
                channel=getNumber(2);
                value=getNumber(3);
                validity=1;
                sendPacket(address,Cmd_Set,channel,1,&value);

////////               Discard code for checking
                /*
                if(address==23)
                {
                    GPIO_PORTF_DATA_R |= 0x04;
                    waitMicrosecond(500000);
                    GPIO_PORTF_DATA_R &= ~0x04;
                    waitMicrosecond(500000);
                }
                if (channel==45)
                {
                    GPIO_PORTF_DATA_R |= 0x08;
                    waitMicrosecond(500000);
                    GPIO_PORTF_DATA_R &= ~0x08;
                    waitMicrosecond(500000);
                }
                if(value==1)
                {
                    GPIO_PORTF_DATA_R |= 0x02;
                    waitMicrosecond(500000);
                    GPIO_PORTF_DATA_R &= ~0x02;
                    waitMicrosecond(500000);
                }

                */
            }

            if(isCommand("cs",1))
            {
                if(strcmp(getString(1),"on")==0)
                {
                    csEnable=true;
                    putsUart0("\r\nCS is enabled\r\n");
                }

                else if(strcmp(getString(1),"off")==0)
                {
                    csEnable=false;
                    putsUart0("\r\nCS is disabled\r\n");
                }

                else
                    putsUart0("\r\ndo you want cs on or off?\r\n");

                validity=1;
          //      sendPacket(\0,Cmd_Set,channel,1,&value);
/*
                if(csEnable==true)
                {
                    GPIO_PORTF_DATA_R |= 0x04;
                    waitMicrosecond(500000);
                    GPIO_PORTF_DATA_R &= ~0x04;
                    waitMicrosecond(500000);
                }
                if (csEnable==false)
                {
                    GPIO_PORTF_DATA_R |= 0x08;
                    waitMicrosecond(500000);
                    GPIO_PORTF_DATA_R &= ~0x08;
                    waitMicrosecond(500000);
                }
*/
            }

            if(isCommand("reset",1))
            {
                address=getNumber(1);
                validity=1;
                sendPacket(address,Cmd_Reset,0,0,0);
            }

            if(isCommand("random",1))
            {
                if(strcmp(getString(1),"on")==0)
                    randomRetrans=true;

                else if(strcmp(getString(1),"off")==0)
                    randomRetrans=false;

                else
                    putsUart0("\r\ndo you want random on or off?");

                validity=1;
/*
                if(randomRetrans==true)
                {
                    GPIO_PORTF_DATA_R |= 0x04;
                    waitMicrosecond(500000);
                    GPIO_PORTF_DATA_R &= ~0x04;
                    waitMicrosecond(500000);
                }
                if (randomRetrans==false)
                {
                    GPIO_PORTF_DATA_R |= 0x08;
                    waitMicrosecond(500000);
                    GPIO_PORTF_DATA_R &= ~0x08;
                    waitMicrosecond(500000);
                }
                */
            }

            if(isCommand("get",2))
            {
                address=getNumber(1);
                channel=getNumber(2);
                sendPacket(address,Cmd_Get,channel,1,'\0');
                validity=1;

             /*                 if(address==23)
                                {
                                    GPIO_PORTF_DATA_R |= 0x04;
                                    waitMicrosecond(500000);
                                    GPIO_PORTF_DATA_R &= ~0x04;
                                    waitMicrosecond(500000);
                                }
                                if (channel==45)
                                {
                                    GPIO_PORTF_DATA_R |= 0x08;
                                    waitMicrosecond(500000);
                                    GPIO_PORTF_DATA_R &= ~0x08;
                                    waitMicrosecond(500000);
                                }
             */
            }

            if(isCommand("poll",0))
            {
                validity=1;
                sendPacket(Broadcast_Address,Cmd_Poll_Request,'\0','\0','\0');
               /*                  GPIO_PORTF_DATA_R |= 0x04;
                                   waitMicrosecond(500000);
                                   GPIO_PORTF_DATA_R &= ~0x04;
                                   waitMicrosecond(500000);
               */
            }


            if(isCommand("sa",2))
            {
                address=getNumber(1);
                newAddress=getNumber(2);
             //   tempNewAddress[0]=newAddress;
                validity=1;
                sendPacket(address,Cmd_Set_Address,'\0',1,&newAddress);
/*
                                if(address==23)
                                {
                                    GPIO_PORTF_DATA_R |= 0x04;
                                    waitMicrosecond(500000);
                                    GPIO_PORTF_DATA_R &= ~0x04;
                                    waitMicrosecond(500000);
                                }
                                if (newAddress==45)
                                {
                                    GPIO_PORTF_DATA_R |= 0x08;
                                    waitMicrosecond(500000);
                                    GPIO_PORTF_DATA_R &= ~0x08;
                                    waitMicrosecond(500000);
                                  }
*/
            }

            if(isCommand("ack",1))
            {
                if(strcmp(getString(1),"on")==0)
                    ackon=true;

                else if(strcmp(getString(1),"off")==0)
                    ackon=false;

                else
                    putsUart0("\r\ndo you want ack on or off?");

                validity=1;
            }

 /*           if(isCommand("ackon",0))
            {
                ackon=true;
                validity=1;


                GPIO_PORTF_DATA_R |= 0x04;
                waitMicrosecond(500000);
                GPIO_PORTF_DATA_R &= ~0x04;
                waitMicrosecond(500000);

            }

            if(isCommand("ackoff",0))
            {
                ackon=false;
                validity=1;

                GPIO_PORTF_DATA_R |= 0x04;
                waitMicrosecond(500000);
                GPIO_PORTF_DATA_R &= ~0x04;
                waitMicrosecond(500000);

            }
*/
            if(isCommand("rgb",5))
            {
                address=getNumber(1);
                channel=getNumber(2);
                for(var_n=0;var_n<3;var_n++)
                {
                    rgbData[var_n]=getNumber(var_n+3);
                }
                validity=1;
                sendPacket(address,Cmd_RGB,channel,3,&rgbData);
                ///discard code
                if(address==9)
                {
                    initPWM();
                    setRgbColor(rgbData[0], rgbData[1], rgbData[2]);
                    putsUart0("\r\nThe value set for red, blue and green is ");
                    putiUart0(rgbData[0]);
                    putsUart0(" ");
                    putiUart0(rgbData[1]);
                    putsUart0(" ");
                    putiUart0(rgbData[2]);
                    putsUart0(" ");
                    putsUart0("\r\n");
                    waitMicrosecond(5000000);//Enough time to observe PWM output on LEDs
                    clearPWM();

                }

            }

            if(isCommand("lcd",2))
            {
                address=getNumber(1);
                channel=getNumber(2);
                validity=1;
                sendPacket(address,Cmd_LCDtext,channel,'\0','\0');
                //discard code
                if(address==9)
                {
                  if(channel==3)
                  {
                      initGraphicsLcd();
                      setGraphicsLcdTextPosition(0, 50);
                      putsGraphicsLcd("Hey there!!");
                  }
                }

            }

            if(isCommand("pulse",5))
            {
                address=getNumber(1);
                channel=getNumber(2);
                for(var_n=0;var_n<3;var_n++)
                {
                    rgbData[var_n]=getNumber(var_n+3);
                }
                validity=1;
                sendPacket(address,Cmd_Pulse,channel,3,&rgbData);
                // discard code
                if(address==9)
                {
                    T_ON_ms1=rgbData[1];
                    T_ON_ms2=rgbData[2];
                    var_t=(T_ON_ms1<<8)|T_ON_ms2;

                    initPWM();
                    setRgbColor(rgbData[0],0,0);
                    T_ON=var_t*10;


                    putsUart0("\r\nThe value of pulse is ");
                    putiUart0(rgbData[0]);
                    putsUart0(" ");

                    putsUart0("\r\nUpper byte is ");
                    putiUart0(rgbData[1]);
                    putsUart0(" ");
                    putsUart0("\r\nLower byte is ");
                    putiUart0(rgbData[2]);
                    putsUart0(" ");
                    putsUart0("\r\n");
                }
            }

           if(isCommand("square",10))
            {
                address=getNumber(1);
                channel=getNumber(2);
                for(var_n=0;var_n<8;var_n++)
                {
                    squareData[var_n]=getNumber(var_n+3);
                }
                validity=1;
                sendPacket(address,Cmd_Square,channel,8,&squareData);
                // discard code
                if(address==9)
                {
                    int i;
                    cycleU=squareData[6];
                    cycleL=squareData[7];
                    cycle=(cycleU<<8)|cycleL;
                    T1_ON_ms1=squareData[2];
                    T1_ON_ms2=squareData[3];
                    var_t1=(T1_ON_ms1<<8)|T1_ON_ms2;
                    T2_ON_ms1=squareData[4];
                    T2_ON_ms2=squareData[5];
                    var_t2=(T2_ON_ms1<<8)|T2_ON_ms2;
                    for(i=0;i<cycle;i++)
                    {
                            initPWM();
                            setRgbColor(squareData[0],0,0);
                            waitMicrosecond(var_t1*1000);
                            clearPWM();

                            initPWM();
                            setRgbColor(0,0,squareData[1]);
                            waitMicrosecond(var_t2*1000);
                            clearPWM();

                    }

                    putsUart0("\r\n value1 is ");
                    putiUart0(squareData[0]);
                    putsUart0(" ");
                    putsUart0("\r\nT1 Upper byte is ");
                    putiUart0(squareData[2]);
                    putsUart0(" ");
                    putsUart0("\r\nT1 Lower byte is ");
                    putiUart0(squareData[3]);
                    putsUart0(" ");
                    putsUart0("\r\n");
                    putsUart0("\r\n value2 is ");
                    putiUart0(squareData[1]);
                    putsUart0(" ");
                    putsUart0("\r\nT2 Upper byte is ");
                    putiUart0(squareData[4]);
                    putsUart0(" ");
                    putsUart0("\r\nT2 Lower byte is ");
                    putiUart0(squareData[5]);
                    putsUart0(" ");
                    putsUart0("\r\n");
                    putsUart0("\r\ncycle upper is ");
                    putiUart0(squareData[6]);
                    putsUart0(" ");
                    putsUart0("\r\ncycle lower is ");
                    putiUart0(squareData[7]);
                    putsUart0(" ");
                    putsUart0("\r\n");
                }
            }

            if(!validity)
            {
                putsUart0("\r\nCommand entered is invalid");
            }

           // GPIO_PORTC_DATA_R |= 0x040;
            //putbUart1(0x77);
            /**char a;
            a=getbUart1();
            putcUart0(a);
            */
        }

}


