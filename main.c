//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"

#define MAX_SIZE 80
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4))) // Port F
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4))) // Port F
#define INTEGRATE    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4))) // Port E
#define MEAS_LR      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4))) // Port E
#define MEAS_C       (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4))) // Port E
#define HIGH_R       (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4))) // Port D
#define LOW_R        (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 3*4))) // Port D
#define COMPARATOR   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4))) // Port C
//List of Commands,States and pins
char commands[8][20]={"set","reset","voltage","resistor","capacitance","inductance","esr","auto"};
char states[2][10]={"on","off"};
char pins[5][20]={"measlr","measc","highr","lowr","integrate"};
// Global Variables
uint8_t argc;
uint8_t pos[];
uint32_t time = 0;
char type[];
char str[MAX_SIZE+1];

char s[20];
uint8_t k=0;
double dut1,dut2,diffVoltage;
uint16_t raw1=0,raw2=0;

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A E D F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOC;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0A;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0A; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x1A;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure pins
    GPIO_PORTE_DIR_R = 0x0E;  // bits 1,2 and 3 is outputs, other pins are inputs
    GPIO_PORTE_DR2R_R = 0x0E; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DEN_R = 0x0E;  // enable pins
    GPIO_PORTD_DIR_R = 0x0C;  // bits 2 and 3 is outputs, other pins are inputs
    GPIO_PORTD_DR2R_R = 0x0C; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTD_DEN_R = 0x0C;  // enable pins

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // Configure AN6(DUT1) as an analog input using analong module 0
    SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking
    GPIO_PORTD_AFSEL_R |= 0x02;                      // select alternative functions for AN6 (PD1) DUT1
    GPIO_PORTD_DEN_R &= ~0x02;                       // turn off digital operation on pin PD1
    GPIO_PORTD_AMSEL_R |= 0x02;                      // turn on analog operation on pin PD1
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC0_SSMUX3_R = 6;                               // set first sample to AN6
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

    // Configure AN7(DUT2) as an analog input using analong module 1
    SYSCTL_RCGCADC_R |= 2;                           // turn on ADC module 1 clocking
    GPIO_PORTD_AFSEL_R |= 0x01;                      // select alternative functions for AN7 (PD0) DUT2
    GPIO_PORTD_DEN_R &= ~0x01;                       // turn off digital operation on pin PD0
    GPIO_PORTD_AMSEL_R |= 0x01;                      // turn on analog operation on pin PD0
    ADC1_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC1_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC1_SSMUX3_R = 7;                               // set first sample to AN7
    ADC1_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

    // Configure analog comparator
    SYSCTL_RCGCACMP_R |= 0x01;                       // enable comparator
    GPIO_PORTC_AFSEL_R = 0x80;                       // select pin PC7 for alternate function
    GPIO_PORTC_DEN_R &= ~0x080;                      // Turn off digital enable for PC7
    GPIO_PORTC_AMSEL_R |= 0x80;                      // turn on analog operation on pin PC7
    COMP_ACREFCTL_R = 0x20F;                         // En=1 RNG=0 Vref=F
    COMP_ACCTL0_R = 0x40C;                           // Either edge ; Interupt enabled ; Coomparator positive is Vref=3.3
    COMP_ACINTEN_R |= 1;                             // Interupt enable
    COMP_ACRIS_R  |= 0x01;                           // Interupt
    NVIC_EN0_R &= ~(1 << (INT_COMP0 - 16));          // turn-off interrupt 41(COMP1)

    //Timer configuration
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;                                 // turn-on timer
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                                            // turn-off counter before reconfiguring
    WTIMER5_CFG_R = 4;                                                           // configure as 32-bit counter (A only)
    WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER5_CTL_R = TIMER_CTL_TAEVENT_POS;                                       // measure time from positive edge to positive edge
    WTIMER5_TAV_R = 0;                                                           // zero counter for first period
    NVIC_EN3_R &= ~(1 << (INT_WTIMER5A-16-96));                                  // turn-on interrupt 120 (WTIMER5A)

}

void analogComparatorISR(void)
{
    time=WTIMER5_TAV_R;
    time/=40;
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;
    COMP_ACMIS_R = 0x01;    // Clear the interupt flag
}
// Blocking function that returns ADC value when the ADC Active Sample Sequencer is not busy
int16_t readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}

// Blocking function that returns ADC value when the ADC Active Sample Sequencer is not busy
int16_t readAdc1Ss3()
{
    ADC1_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC1_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC1_SSFIFO3_R;                           // get single result from the FIFO
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
// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicroseconwaitd(uint32_t us)
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
void getsTerminal(char* str)
{
        char c;
        uint8_t count=0;
        while(1)
        {
            c=getcUart0();
            if(c==8)
            {
                if(count==0)
                    continue;
                else
                {
                    count--;
                    continue;
                }
            }
            if(c==13)
            {
                str[count]=0;
                break;
            }
            if(c>=' ')
            {
                if(c>='A' && c<='Z')
                    str[count++]=c+32;
                else
                    str[count++]=c;
                if(count==MAX_SIZE)
                {
                    str[count]=0;
                    break;
                }
            }

        }
}
void parseString(char* str)
{
    argc=0;
    uint8_t i=0,posi=0;
    while(1)
    {
        if(str[i]>='a' && str[i]<='z')
        {
            pos[posi]=i;
            type[posi]='c';
            posi++;
            argc++;
            while(str[i]>='a' && str[i]<='z')
            {
                i++;
            }
        }
        else if(str[i]>='0' && str[i]<='9')
        {
            pos[posi]=i;
            type[posi]='d';
            posi++;
            argc++;
            while(str[i]>='0' && str[i]<='9')
            {
                i++;
            }
        }
        else if(str[i]==0)
        {
            type[posi]='\0';
            break;
        }
        else
        {
            str[i]=0;
            i++;
        }
    }
}
uint8_t isCommand(char* s , char *c)
{
    uint8_t i=0,k=0,j=0,r=0;
    c[0]='0';
    for(i=0;i<8;i++)
    {
        if(strcmp(s+pos[0],commands+i)==0)
        {
           r+=(i+1);
        }
    }
    if(r==1)
    {
        for(j=0;j<5;j++)
            {
                if(strcmp(s+pos[1],pins+j)==0)
                {
                    r+=10*(j+1);
                }
            }
        for(k=0;k<2;k++)
            {
                if(strcmp(s+pos[2],states+k)==0)
                {
                    r+=(k+1)*100;
                }
            }
    }
    i=0;
    while(r>0)
    {
        j=r%10;
        r=r/10;
        c[i]=j+48;
        i++;
        c[i]='\0';
    }
    if(c[0]=='0')
        return 0;
    if(c[0]=='1' && (c[1]=='0' || c[2]=='0'|| c[1]=='\0' || c[2]=='\0'))
        return 0;
    if(c[0]>='2' && argc!=1)
        return 0;
    return 1;
}
void doCommand(char* c)
{
    switch(c[0])
    {
        case'1':setCommand(c);
                break;
        case'2':resetCommand();
                break;
        case'3':voltageCommand();
                break;
        case'4':resistorCommand();
                break;
        case'5':capacitorCommand();
                break;
        case'6':inductorCommand();
                break;
        case'7':break;
        case'8':break;
    }
    return ;
}

void setCommand(char* c)
{
    switch(c[1])
    {
        case '1':if(MEAS_C)
                {
                    putsUart0("Please Turn OFF measc first\n\r");
                }
                else if(c[2]=='1')
                {
                    putsUart0("MEAS_LR ON\n\r");
                    MEAS_LR=1;
                }
                else
                    MEAS_LR=0;
                break;
        case '2':if(MEAS_LR)
                 {
                     putsUart0("Please Turn OFF measlr first\n\r");
                 }
                 else if(c[2]=='1')
                 {
                     putsUart0("MEAS_C ON\n\r");
                     MEAS_C=1;
                 }
                 else
                     MEAS_C=0;
                 break;
        case '3':if(c[2]=='1')
                     HIGH_R=1;
                 else
                     HIGH_R=0;
                 break;
        case '4':if(c[2]=='1')
                     LOW_R=1;
                 else
                     LOW_R=0;
                 break;
        case '5':if(c[2]=='1')
                     INTEGRATE=1;
                 else
                     INTEGRATE=0;
                 break;
    }
    return ;
}

void voltageCommand()
{
    k=0;
    while(k<4)
    {
       raw1+=readAdc0Ss3();
       k++;
    }
    raw1=raw1/4;
    k=0;
    while(k<4)
    {
        raw2+=readAdc1Ss3();
        k++;
    }
    raw2=raw2/4;
    dut1=(raw1*3.3)/4095;
    dut2=(raw2*3.3)/4095;

    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");
    putsUart0("\n\r");
    sprintf(s, "%lf", dut1);
    putsUart0(s);
    putsUart0("\n\r");
    diffVoltage=dut2-dut1;
    sprintf(s, "%lf", dut2);
    putsUart0(s);
    putsUart0("\n\r");
    sprintf(s, "%lf", diffVoltage);
    putsUart0(s);
    return ;
}

void resetCommand()
{
    putcUart0(27);
    putcUart0('[');
    putcUart0('2');
    putcUart0('J');
    waitMicroseconwaitd(100000);
    NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ ;
}

void resistorCommand()
{
    char buffer[20];
    uint32_t resistance;
    MEAS_LR=0;
    LOW_R=1;
    INTEGRATE=1;
    waitMicroseconwaitd(1000000);
    WTIMER5_TAV_R=0;
    LOW_R=0;
    MEAS_LR=1;
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;              // turn-on counter
    NVIC_EN0_R |= (1 << (INT_COMP0 - 16));        // turn-on interrupt 41(COMP0)
    waitMicroseconwaitd(500000);
    NVIC_EN0_R &= ~(1 << (INT_COMP0 - 16));        // turn-on interrupt 41(COMP)
    sprintf(buffer, "%lu",time );
    putsUart0("\n\r");
    putsUart0(buffer);


    putsUart0("\n\r");
    resistance=0.81307039*time-6.20937877;
    sprintf(buffer, "%lu",resistance );
    putsUart0("\n\r");
    putsUart0(buffer);
    putsUart0("\n\r");
    MEAS_LR=0;
    MEAS_C=0;
    return;
}
void capacitorCommand()
{
    char buffer[20];
    double capacitance;
    MEAS_C=1;
    LOW_R=1;
    INTEGRATE=0;
    waitMicroseconwaitd(1000000);
    WTIMER5_TAV_R=0;
    LOW_R=0;
    HIGH_R=1;
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;              // turn-on counter
    NVIC_EN0_R |= (1 << (INT_COMP0 - 16));        // turn-on interrupt 41(COMP0)
    waitMicroseconwaitd(5000000);
    NVIC_EN0_R &= ~(1 << (INT_COMP0 - 16));        // turn-on interrupt 41(COMP)
    sprintf(buffer, "%lu",time );
    putsUart0("\n\r");
    putsUart0(buffer);


    putsUart0("\n\r");
    capacitance= (0.000004699/7041037)*time+0.006265702000000317/7041037;
    sprintf(buffer, "%0.11lf",capacitance );
    putsUart0("\n\r");
    putsUart0(buffer);
    putsUart0("\n\r");
    return;
}
void inductorCommand()
{
    char buffer[20];
        uint32_t incductance;
        MEAS_LR=1;
        LOW_R=1;;
        WTIMER5_TAV_R=0;
        WTIMER5_CTL_R |= TIMER_CTL_TAEN;              // turn-on counter
        NVIC_EN0_R |= (1 << (INT_COMP0 - 16));        // turn-on interrupt 41(COMP0)
        waitMicroseconwaitd(500000);
        NVIC_EN0_R &= ~(1 << (INT_COMP0 - 16));        // turn-on interrupt 41(COMP)
        sprintf(buffer, "%lu",time );
        putsUart0("\n\r");
        putsUart0(buffer);


        putsUart0("\n\r");
     /*   resistance=0.81307039*time-6.20937877;
        sprintf(buffer, "%lu",resistance );
        putsUart0("\n\r");
        putsUart0(buffer);
        putsUart0("\n\r");
        return;*/
}
int main(void)
{
    initHw();
    uint8_t i=0;
    char command[3];
    GREEN_LED=1;
    waitMicroseconwaitd(1000000);
    GREEN_LED=0;
    while(1)
    {
        putsUart0("\rEnter Command");
        putsUart0("\r\n");
        getsTerminal(str);
        putsUart0("\r\n");
        putsUart0(str);
        parseString(str);
        putsUart0("\r\n");
        for(i=0;i<argc;i++)
        {
            putsUart0(str+pos[i]);
            putsUart0("\r\n");
        }
        putsUart0(type);
        putsUart0("\r\n");
        i=isCommand(str,command);
        if(i==1)
        putsUart0(command);
        else
        putsUart0("Invalid Command");
        doCommand(command);
        putsUart0("\r\n");

    }
}

