#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "adc0.h"

//Port C BitBanding
#define DEINT  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4)))
#define COMP   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))

//Port A Bitbanding
#define PUMP   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))
#define SPEAKER (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))

// PortE masks
#define AIN3_MASK 1
#define AIN2_MASK 2
#define AIN1_MASK 4

//Port C Masks
#define DEINT_MASK 16
#define COMP_MASK 128

// PortA masks
#define UART_TX_MASK 2
#define UART_RX_MASK 1
#define MAX_CHARS 80
#define MAX_FIELDS 6
#define PUMP_MASK 128
#define SPEAKER_MASK 64

char FieldString[MAX_CHARS];

typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

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

void initHw()
{
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    //Enable Analog Comparator Clock
    SYSCTL_RCGCACMP_R |= SYSCTL_RCGCACMP_R0;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1 | SYSCTL_RCGCTIMER_R2;

    //Enable Clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R4 | SYSCTL_RCGCGPIO_R0;
    _delay_cycles(3);

    //Configure Port A pins for pump and speaker as outputs
    GPIO_PORTA_DIR_R |= PUMP_MASK;
    GPIO_PORTA_DEN_R |= PUMP_MASK;
    GPIO_PORTA_DIR_R |= SPEAKER_MASK;
    GPIO_PORTA_DEN_R |= SPEAKER_MASK;

    // Configure AIN3 as an analog input
    GPIO_PORTE_AFSEL_R |= AIN3_MASK;
    GPIO_PORTE_DEN_R &= ~AIN3_MASK;
    GPIO_PORTE_AMSEL_R |= AIN3_MASK;

    //Configure AIN2 as an analog input
    GPIO_PORTE_AFSEL_R |= AIN2_MASK;
    GPIO_PORTE_DEN_R &= ~AIN2_MASK;
    GPIO_PORTE_AMSEL_R |= AIN2_MASK;

    //Configure AIN1 as an analog input
    GPIO_PORTE_AFSEL_R |= AIN1_MASK;
    GPIO_PORTE_DEN_R &= ~AIN1_MASK;
    GPIO_PORTE_AMSEL_R |= AIN1_MASK;


    //Configure Port C pins
    GPIO_PORTC_DIR_R |= DEINT_MASK ;
    GPIO_PORTC_DEN_R |= DEINT_MASK ;
    GPIO_PORTC_DEN_R &= ~COMP_MASK;
    GPIO_PORTC_AMSEL_R |= COMP_MASK;

    //Comparator Configuration
    COMP_ACREFCTL_R |= COMP_ACREFCTL_VREF_M | COMP_ACREFCTL_EN;
    COMP_ACREFCTL_R &= ~COMP_ACREFCTL_RNG;
    COMP_ACCTL0_R = 0x0000040C;
    waitMicrosecond(10);
    COMP_ACSTAT0_R |= COMP_ACSTAT0_OVAL;

    //Configure Timer1
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
    TIMER1_TAMR_R |=TIMER_TAMR_TACDIR;
    TIMER1_CTL_R |= TIMER_CTL_TAEN;

    //Configure Timer2
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
    TIMER2_TAILR_R = 40000000;
    TIMER2_IMR_R = TIMER_IMR_TATOIM;
    NVIC_EN0_R |= 1 << (INT_TIMER2A-16);

    //Configuration of Hibernation Module
    HIB_IM_R |= HIB_IM_WC;
    HIB_CTL_R |=HIB_CTL_CLK32EN;
    while(HIB_MIS_R==0x0);
    HIB_CTL_R |= HIB_CTL_RTCEN;
}


// Initialize UART0
void initUart0()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    //SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
    _delay_cycles(3);

    // Configure UART0 pins
    GPIO_PORTA_DIR_R |= UART_TX_MASK;                   // enable output on UART0 TX pin
    GPIO_PORTA_DIR_R &= ~UART_RX_MASK;                   // enable input on UART0 RX pin
    GPIO_PORTA_DR2R_R |= UART_TX_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= UART_TX_MASK | UART_RX_MASK;    // enable digital on UART0 pins
    GPIO_PORTA_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;  // use peripheral to drive PA0, PA1
    GPIO_PORTA_PCTL_R &= ~(GPIO_PCTL_PA1_M | GPIO_PCTL_PA0_M); // clear bits 0-7
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                        // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
    UART0_IBRD_R = 21;                                  // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                                  // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // enable TX, RX, and module
}

// Set baud rate as function of instruction cycle frequency
void setUart0BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
                                                        // where r = fcyc / 16 * baudRate
    UART0_IBRD_R = divisorTimes128 >> 7;                 // set integer value to floor(r)
    UART0_FBRD_R = ((divisorTimes128 + 1)) >> 1 & 63;    // set fractional value to round(fract(r)*64)
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart0(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}


void getsUart0(USER_DATA* data)
{
    int count=0;
    char c;
    while(1)
    {
        //if(kbhitUart0())
        //{
            c=getcUart0();
            if((c==8 || c==127) && count>0)
            {
                count--;
            }

            else if(c==13)
            {
                data->buffer[count]='\0';
                return;
            }

            else if(c>=32)
            {
                data->buffer[count]=c;
                count++;
                if(count==MAX_CHARS)
                {
                    data->buffer[count]='\0';
                    return ;
                }
            }
        //}
        //else
        //{
          //  kbhitUart0();
          //  return;
        //}
    }
}
// Returns the status of the receive buffer
bool kbhitUart0()
{
    return !(UART0_FR_R & UART_FR_RXFE);
}

int stringCompare(const char *str1, const char *str2)
{
    int i = 0;
    while(str1[i] == str2[i])
    {
        if(str1[i] == '\0' && str2[i] == '\0')
            break;
            i++;
    }
    return str1[i] - str2[i];
}

int atoi(char* str)
{
    int res = 0; // Initialize result
    int i;
    // Iterate through all characters of input string and
    // update result
    for (i = 0; str[i] != '\0'; i++)
        res = res * 10 + str[i] - '0';

    // return result.
    return res;
}

void parseFields(USER_DATA* data)
{
    int counter = 0;
    int field_count = 0;
    int counter_two;
    while (true)
    {
        if ((data->fieldType[field_count-1] != 'a') || (counter == 0))
        {
            if (((data->buffer[counter] >= 65) && (data->buffer[counter] <= 90)) || ((data->buffer[counter] >= 97) && (data->buffer[counter] <= 122)))
            {
                if(data->fieldType[field_count-1] == 'd')
                {
                    data->fieldType[field_count-1] = 'a';
                    data->fieldPosition[field_count-1] = counter;
                }
                else
                {
                    data->fieldType[field_count] = 'a';
                    data->fieldPosition[field_count] = counter;
                    field_count++;
                }
            }
        }
        if ((data->fieldType[field_count-1] != 'n') || (counter == 0))
        {
            if ((data->buffer[counter] >= 48) && (data->buffer[counter] <= 57))
            {
                if(data->fieldType[field_count-1] == 'd')
                {
                    data->fieldType[field_count-1] = 'n';
                    data->fieldPosition[field_count-1] = counter;
                }
                else
                {
                    data->fieldType[field_count] = 'n';
                    data->fieldPosition[field_count] = counter;
                    field_count++;
                }
            }
        }
        if (!((data->buffer[counter] >= 65) && (data->buffer[counter] <= 90)) && (data->fieldType[field_count-1] != 'd'))
        {
            if (!((data->buffer[counter] >= 97) && (data->buffer[counter] <= 122)))
            {
                if (!((data->buffer[counter] >= 48) && (data->buffer[counter] <= 57)))
                {
                    data->fieldType[field_count] = 'd';
                    field_count++;
                }
            }
        }
        if ((field_count == MAX_FIELDS) || (data->buffer[counter] == '\0'))
        {
            for (counter_two = 0; counter_two <= counter; counter_two ++)
            {
                if (!((data->buffer[counter_two] >= 65) && (data->buffer[counter_two] <= 90)))
                {
                    if (!((data->buffer[counter_two] >= 97) && (data->buffer[counter_two] <= 122)))
                    {
                        if (!((data->buffer[counter_two] >= 48) && (data->buffer[counter_two] <= 57)))
                        {
                            data->buffer[counter_two] = '\0';
                        }
                    }
                }
                if ((data->fieldType[counter_two] == 'd') && (counter_two <= 5))
                {
                    data->fieldType[counter_two] = '\0';
                }
             }
            data->fieldCount = field_count-1;
            return;
        }
        counter++;

     }

}


char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
   int counter=0;
   int counter2= data->fieldPosition[fieldNumber];
   if(fieldNumber <= MAX_FIELDS)
   {
       while(data->buffer[counter2] != '\0')
       {
           FieldString[counter]=data->buffer[counter2];
           counter++;
           counter2++;

       }
       FieldString[counter]= '\0';
       return FieldString;
   }
   else
   {
     FieldString[0]='\0';
     return FieldString;
   }


}

uint32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    uint32_t integer_pointer;
    int counter=0;
    int counter2= data->fieldPosition[fieldNumber];
    if(fieldNumber <= MAX_FIELDS)
       {
           while(data->buffer[counter2]!= '\0')
           {
               FieldString[counter]=data->buffer[counter2];
               counter++;
               counter2++;
           }
           FieldString[counter]= '\0';
           integer_pointer= atoi(&FieldString);
           return integer_pointer;
       }
       else
       {
         integer_pointer=0;
         return integer_pointer;
       }

}

bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
    char* firstField= '\0';
    firstField = getFieldString(data,0);
    if(stringCompare(firstField,strCommand)==0 && data->fieldCount >= minArguments)
    {
        return true;
    }

    return false;
}


uint32_t getVolume()
{
    uint32_t result=0;
    DEINT=1;
    DEINT=0;

    DEINT=1;
    while(COMP_ACSTAT0_R == 0x0);

    TIMER1_TAV_R=0;
    DEINT=0;
    TIMER1_TAV_R=0;

    while(COMP_ACSTAT0_R !=0x0);
    result = (TIMER1_TAV_R-344)/1.472;



    return result;

}

float getLightPercentage()
{
    float lightPercent=0;
    float raw=0;
    setAdc0Ss3Mux(3);
    setAdc0Ss3Log2AverageCount(4);
    raw = readAdc0Ss3();
    raw= ((raw+0.5) / 4096 * 3.3);
    lightPercent= (raw/0.51)*100;
    if(lightPercent >=100)
    {
        lightPercent=100;
    }
    return lightPercent;


}

float getMoisturePercentage()
{
    float MoisturePercent=0;
    float rawValue=0;
    setAdc0Ss3Mux(2);
    setAdc0Ss3Log2AverageCount(4);
    rawValue= readAdc0Ss3();
    rawValue= ((rawValue+0.5) / 4096 * 3.3);
    MoisturePercent= (rawValue/3.264148)*100;
    MoisturePercent= 100-MoisturePercent;
    return MoisturePercent;
}

float getBatteryVoltage()
{
    float Voltage=0;
    setAdc0Ss3Mux(1);
    setAdc0Ss3Log2AverageCount(4);
    Voltage= readAdc0Ss3();
    Voltage= ((Voltage+0.5) / 4096 * 3.3);
    Voltage= (147000*Voltage);
    Voltage=Voltage/47000;
    return Voltage;
}

void enablePump()
{
    PUMP=1;
    return;
}

void disablePump()
{
    PUMP=0;
    return;
}

void timer2Isr()
{
    SPEAKER ^= 1;
    TIMER2_ICR_R = TIMER_ICR_TATOCINT;
}
void playBatteryLowAlert()
{
   uint32_t count=0;

   while(count!=2)
   {
       TIMER2_TAILR_R = 85837;
       TIMER2_CTL_R |= TIMER_CTL_TAEN;
       waitMicrosecond(1000000);
       TIMER2_CTL_R &= ~TIMER_CTL_TAEN;

       TIMER2_TAILR_R = 80972;
       TIMER2_CTL_R |= TIMER_CTL_TAEN;
       waitMicrosecond(1000000);
       TIMER2_CTL_R &= ~TIMER_CTL_TAEN;

       TIMER2_TAILR_R = 76336;
       TIMER2_CTL_R |= TIMER_CTL_TAEN;
       waitMicrosecond(1000000);
       TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
       count++;
   }
   return;
}

void playWaterLowAlert()
{
    uint32_t count=0;

    while(count!=2)
    {
        TIMER2_TAILR_R = 45455;
        TIMER2_CTL_R |= TIMER_CTL_TAEN;
        waitMicrosecond(1000000);
        TIMER2_CTL_R &= ~TIMER_CTL_TAEN;

        TIMER2_TAILR_R = 48135;
        TIMER2_CTL_R |= TIMER_CTL_TAEN;
        waitMicrosecond(1000000);
        TIMER2_CTL_R &= ~TIMER_CTL_TAEN;

        TIMER2_TAILR_R = 51020;
        TIMER2_CTL_R |= TIMER_CTL_TAEN;
        waitMicrosecond(1000000);
        TIMER2_CTL_R &= ~TIMER_CTL_TAEN;

        TIMER2_TAILR_R = 64309;
        TIMER2_CTL_R |= TIMER_CTL_TAEN;
        waitMicrosecond(1000000);
        TIMER2_CTL_R &= ~TIMER_CTL_TAEN;

        TIMER2_TAILR_R = 60698;
        TIMER2_CTL_R |= TIMER_CTL_TAEN;
        waitMicrosecond(1000000);
        TIMER2_CTL_R &= ~TIMER_CTL_TAEN;

        TIMER2_TAILR_R = 57307;
        TIMER2_CTL_R |= TIMER_CTL_TAEN;
        waitMicrosecond(1000000);
        TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
        count++;
    }
    return;
}

int getCurrentSeconds()
{
    int current_sec=0;
    current_sec=HIB_RTCC_R;
    current_sec=current_sec % 86400;
    return current_sec;

}

bool isWateringAllowed(int lowerLimit,int upperLimit)
{
    int time=getCurrentSeconds();
    if(time>lowerLimit && time<upperLimit)
    {
        return true;
    }
    return false;
}

int main()
{
    USER_DATA data;
    char string[100];
    uint32_t volume;
    float light;
    float moisture;
    float BatteryLevel;
    float level=20.0;
    int lowerWindow=43200;
    int upperWindow=61200;
    int seconds_day=0;
    bool watering;
    float light_level=5.0;
    initHw();
    initUart0();
    initAdc0Ss3();


    while(true)
    {
        if(kbhitUart0()==true)
        {
            getsUart0(&data);
            putsUart0(data.buffer);
            putcUart0('\n');
            putcUart0('\r');

            parseFields(&data);

            uint8_t i;
            for (i = 0; i < data.fieldCount; i++)
            {
                putcUart0(data.fieldType[i]);
                putcUart0('\t');
                putsUart0(&data.buffer[data.fieldPosition[i]]);
                putsUart0("\n\r");
            }

            bool valid = false;
            volume=0;


            if (isCommand(&data, "alert", 1))
            {
                light_level = getFieldInteger(&data,1);

                if(getLightPercentage()>=light_level && getVolume()<200)
                {
                    playWaterLowAlert();
                    valid=true;
                }
                if(getLightPercentage()>=light_level && getBatteryVoltage()<4)
                {
                    waitMicrosecond(1000000);
                    playBatteryLowAlert();
                    valid=true;
                }
                else
                {
                    valid=true;
                    continue;
                }


            }

            if(isCommand(&data,"status",0))
            {
                volume = getVolume();
                sprintf(string,"Volume = %u mL\r\n",volume);
                putsUart0(string);

                light= getLightPercentage();
                sprintf(string,"Light Percentage: %.2f percent\r\n",light);
                putsUart0(string);

                moisture= getMoisturePercentage();
                sprintf(string,"Moisture Percentage: %.2f percent\r\n",moisture);
                putsUart0(string);


                BatteryLevel= getBatteryVoltage();
                sprintf(string,"Battery Voltage: %1.2f Volts\r\n",BatteryLevel);
                putsUart0(string);

                seconds_day = getCurrentSeconds();
                sprintf(string,"Current Seconds = %d sec\r\n",seconds_day);
                putsUart0(string);

                sprintf(string,"Watering Window = %d sec\t%d sec\r\n",lowerWindow,upperWindow);
                putsUart0(string);

                valid =true;
            }

            if(isCommand(&data,"pump",0))
            {
                char *pump = getFieldString(&data,1);
                if(stringCompare(pump,"ON")==0)
                {
                    enablePump();
                    valid=true;
                }
                else if(stringCompare(pump,"OFF")==0)
                {
                    disablePump();
                    valid=true;
                }
                else
                {
                    putsUart0("Invalid command\n\r");
                    continue;
                }

            }

            if(isCommand(&data,"time",2))
            {
                int hours=getFieldInteger(&data,1);
                int minutes=getFieldInteger(&data,2);

                HIB_RTCLD_R= (minutes*60)+(hours*60*60);
                valid=true;

            }

            if(isCommand(&data,"water",4))
            {
                int hours1=getFieldInteger(&data,1);
                int minutes1=getFieldInteger(&data,2);
                int hours2=getFieldInteger(&data,3);
                int minutes2=getFieldInteger(&data,4);

                lowerWindow=(minutes1*60)+(hours1*60*60);
                upperWindow=(minutes2*60)+(hours2*60*60);
                watering=isWateringAllowed(lowerWindow,upperWindow);
                if(watering==true)
                {
                    sprintf(string,"Watering is Allowed");
                    putsUart0(string);
                }
                else
                {
                    sprintf(string,"Watering is not Allowed");
                    putsUart0(string);
                }
                valid=true;

            }

            if(isCommand(&data,"level",1))
            {
                level= getFieldInteger(&data,1);
                valid=true;
            }


            if (!valid)
            {
                putsUart0("Invalid command\n\r");
            }
        }
        else
        {
            volume = getVolume();
            light= getLightPercentage();
            moisture= getMoisturePercentage();
            BatteryLevel= getBatteryVoltage();
            seconds_day = getCurrentSeconds();


            if(light>=light_level && volume<200)
            {
                playWaterLowAlert();
                waitMicrosecond(10000000);
            }
            if(light>=light_level && BatteryLevel<4)
            {
                playBatteryLowAlert();
                waitMicrosecond(10000000);
            }
            watering=isWateringAllowed(lowerWindow,upperWindow);
            if(moisture<level && watering==true && volume>200)
            {
                while(moisture<=60 && volume>200)
                {
                    enablePump();
                    waitMicrosecond(5000000);
                    disablePump();
                    waitMicrosecond(30000000);
                    moisture=getMoisturePercentage();
                    volume= getVolume();
                }
            }

        }
    }

}
