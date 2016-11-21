#include <stdint.h> // Variable definitions for the C99 standard.
#include <stdio.h> // Input and output facilities for the C99 standard.
#include <stdbool.h> // Boolean definitions for the C99 standard.
#include <math.h>
#include "inc/tm4c123gh6pm.h" // Definitions for the interrupt and register assignments.
#include "inc/hw_memmap.h" // Memory map definitions of the Tiva C Series device.
#include "inc/hw_types.h" // Definitions of common types and macros.
#include "inc/hw_ssi.h"
#include "driverlib/sysctl.h" // Definitions and macros for System Control API of DriverLib.
#include "driverlib/interrupt.h" // Defines and macros for NVIC Controller API of DriverLib.
#include "driverlib/gpio.h" // Definitions and macros for GPIO API of DriverLib.
#include "driverlib/timer.h" // Defines and macros for Timer API of DriverLib.
#include "driverlib/pin_map.h" //Mapping of peripherals to pins for all parts.
#include "driverlib/uart.h" // Definitions and macros for UART API of DriverLib.
#include "driverlib/fpu.h" // Prototypes for the FPU manipulation routines.
#include "driverlib/ssi.h" // Definitions and prototypes for the SSI/SPI routines.
#include "utils/uartstdio.h" // Prototypes for the UART console functions.
							// Needs to add "utils/uartstdio.c" through a relative link.
#include "driverlib/adc.h"//ADC definitions and prototypes
//#include "utils/ustdlib.h"

#define TIMER0_FREQ    2 // Freqency in Hz, heartbeat timer
#define UART0_BAUDRATE    9600 // UART baudrate in bps
#define NUM_DISP_TEXT_LINE    4
#define buffer_length 27
#define lat_length 	12
#define long_length 10

// function prototypes
void data_parse(void);
void lat_to_int(void);
void long_to_int(void);
void init_buffers(void);
void init_LEDs(void);
void init_timer(void);
void init_UART(void);
void Timer0_ISR(void);
int check_boundaries(long lat_data, long long_data);
extern void UARTStdioIntHandler(void);

// global variables
volatile long x1=36836213, x2=36835978, x3=36835601, x4=36835900,
y1=-88263881, y2=-88263374, y3=-88263605, y4=-88264106;
int cur_LED=2;
const char *disp_text[NUM_DISP_TEXT_LINE] = {
		"\n",
		">NEO6MV2 UART GPS Module Debug\n ",
		"Raw data strings untill lat and long are found\n ",
		"Chris Askings/Nathan Ruprecht \n \n \n "};
long sys_clock;

volatile char uart_buffer[buffer_length];
volatile char lat_buffer[buffer_length];
volatile char long_buffer[buffer_length];
volatile double lat_value=0;
volatile long long_data=0, lat_data=0;

int main(void)
{
	long i;
    int condition;
	unsigned char user_cmd;

	//sys_clock = SysCtlClockGet();
	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); // 80 MHz
	sys_clock = 80000000; // Hard-coded for 80MHz because of a bug in SysCtlClockGet().

	// Enable the floating-point unit (FPU).
	FPUEnable();
	// Configure FPU to perform lazy stacking of the floating-point state.
	FPULazyStackingEnable();

	init_buffers();
	init_LEDs();
	init_UART();
	init_timer();

	// Enable the processor to respond to interrupts.
	IntMasterEnable();

	// Start the timer by enabling operation of the timer module.
	TimerEnable(TIMER0_BASE, TIMER_A);

	// Initial display on terminal.
	for(i=0; i<NUM_DISP_TEXT_LINE; i++)
		UARTprintf(disp_text[i]);

	while(1) {
		// Read user inputs from UART if available.
		if(UARTRxBytesAvail())
	       user_cmd = UARTgetc();
		else
			user_cmd = 0;

		switch(user_cmd){
		case 'L':

				UARTprintf("\n ");
				UARTgets(uart_buffer, buffer_length);
				UARTprintf("\n uart-buff is: \%s \n \n ", uart_buffer);
				data_parse();
				break;
		}

		condition = check_boundaries(lat_data, long_data);

		if( condition ) {
			//We're good, green LED
            cur_LED=1;
		} else {
            //We're bad, red LED
            cur_LED=0;
        }//end if

	}//end while
}//end main


void data_parse(void)
{
	long x;
	long y;
	x=2;
	y=16;
	while(x<lat_length){
	lat_buffer[x-2]=uart_buffer[x];
	long_buffer[y-16]=uart_buffer[y];
	x++;
	y++;
	}
	UARTprintf("\n lat is: \%s \n ", lat_buffer);
	UARTprintf("\n long is: \%s \n \n ", long_buffer);
	lat_to_int();
	long_to_int();
}//end data_parse


void lat_to_int(void)
{
	long i=0, end_dex=9;
	char hold;

	while(i<end_dex){
		hold=lat_buffer[i];
		switch(hold){
		case '9':
			lat_data=lat_data+(9*pow(10,8-i));
			//UARTprintf("\n 9lat is now: \%i \n ", lat_data);
		break;
		case '8':
			lat_data=lat_data+(8*pow(10,8-i));
			//UARTprintf("\n 8lat is now: \%i \n ", lat_data);
		break;
		case '7':
			lat_data=lat_data+(7*pow(10,8-i));
			//UARTprintf("\n 7lat is now: \%i \n ", lat_data);
		break;
		case '6':
			lat_data=lat_data+(6*pow(10,8-i));
			//UARTprintf("\n 6lat is now: \%i \n ", lat_data);
		break;
		case '5':
			lat_data=lat_data+(5*pow(10,8-i));
			//UARTprintf("\n 5lat is now: \%i \n ", lat_data);
		break;
		case '4':
			lat_data=lat_data+(4*pow(10,8-i));
			//UARTprintf("\n 4lat is now: \%i \n ", lat_data);
		break;
		case '3':
			lat_data=lat_data+(3*pow(10,8-i));
			//UARTprintf("\n 3lat is now: \%i \n ", lat_data);
		break;
		case '2':
			lat_data=lat_data+(2*pow(10,8-i));
			//UARTprintf("\n 2lat is now: \%i \n ", lat_data);
		break;
		case '1':
			lat_data=lat_data+(1*pow(10,8-i));
			//UARTprintf("\n 1lat is now: \%i \n ", lat_data);
		break;
		case '0':
			lat_data=lat_data+(0*pow(10,8-i));
			//UARTprintf("\n 0lat is now: \%i \n ", lat_data);
		break;
		case '.':
			lat_data=lat_data/10;
			//UARTprintf("\n .lat is now: \%i \n ", lat_data);
		break;
		}
		i++;
	}
	UARTprintf("\n lat is now: \%i \n ", lat_data);
}//end lat_to_int


void long_to_int(void)
{
	long i=0, end_dex=9;
	char hold;

	while(i<end_dex){
		hold=long_buffer[i];
		switch(hold){
		case '9':
			long_data=long_data+(9*pow(10,8-i));
			//UARTprintf("\n 9lat is now: \%i \n ", long_data);
		break;
		case '8':
			long_data=long_data+(8*pow(10,8-i));
			//UARTprintf("\n 8lat is now: \%i \n ", long_data);
		break;
		case '7':
			long_data=long_data+(7*pow(10,8-i));
			//UARTprintf("\n 7lat is now: \%i \n ", long_data);
		break;
		case '6':
			long_data=long_data+(6*pow(10,8-i));
			//UARTprintf("\n 6lat is now: \%i \n ", long_data);
		break;
		case '5':
			long_data=long_data+(5*pow(10,8-i));
			//UARTprintf("\n 5lat is now: \%i \n ", long_data);
		break;
		case '4':
			long_data=long_data+(4*pow(10,8-i));
			//UARTprintf("\n 4lat is now: \%i \n ", long_data);
		break;
		case '3':
			long_data=long_data+(3*pow(10,8-i));
			//UARTprintf("\n 3lat is now: \%i \n ", long_data);
		break;
		case '2':
			long_data=long_data+(2*pow(10,8-i));
			//UARTprintf("\n 2lat is now: \%i \n ", long_data);
		break;
		case '1':
			long_data=long_data+(1*pow(10,8-i));
			//UARTprintf("\n 1lat is now: \%i \n ", long_data);
		break;
		case '0':
			long_data=long_data+(0*pow(10,8-i));
			//UARTprintf("\n 0lat is now: \%i \n ", long_data);
		break;
		case '.':
			long_data=long_data/10;
			//UARTprintf("\n .lat is now: \%i \n ", long_data);
		break;
		}
		i++;
	}
	long_data=-long_data;
	UARTprintf("\n long is now: \%i \n \n \n \n ", long_data);
}//end long_to_int


void init_buffers(void)
{
	long i;
	for(i=0; i<buffer_length; i++)
		lat_buffer[i]=0;
		long_buffer[i]=0;
}//end init_buffers


void init_LEDs(void)
{
      // Red LED
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
      GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

      // Green LED
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
      GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_3);

      // Blue LED
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
      GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);
}//end init_LEDs


void init_timer(void)
{
	// Enable and configure Timer0 peripheral.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	// Configure as a 32-bit timer in periodic mode.
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	// Initialize timer load register.
	TimerLoadSet(TIMER0_BASE, TIMER_A, sys_clock/TIMER0_FREQ -1);

	// Registers a function to be called when the interrupt occurs.
	IntRegister(INT_TIMER0A, Timer0_ISR);
	// The specified interrupt is enabled in the interrupt controller.
	IntEnable(INT_TIMER0A);
	// Enable the indicated timer interrupt source.
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}//end init_timer


void init_UART(void)
{
	// Enable and configure UART0 for debugging printouts.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Registers a function to be called when the interrupt occurs.
	IntRegister(INT_UART0, UARTStdioIntHandler);
	UARTStdioConfig(0, UART0_BAUDRATE, sys_clock);
}//end init_UART


// Timer0 interrupt service routine
void Timer0_ISR(void)
{
      // Clear the timer interrupt.
      TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

      if( (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3)) ||
             (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_3)) ||
             (GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) )
      {//All LEDs off
          GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
          GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);
          GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0);
      }
      else {
          switch( cur_LED )
          {
                case 0://Red
                      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 8);
                       break;
                case 1://Green
                      GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 8);
                       break;
                case 2://Blue
                      GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 16);
                       break;
          }//End switch
      }
}//end ISR


int check_boundaries( long lat_data, long long_data )
{
      float m12, m41, m43, m32, b12, b41, b43, b32, xcp1, xcp2,  ycp1, ycp2;

      long_data = 7;
      lat_data = 7;

      m12 = (y2-y1) / (x2-x1);
      m41 = (y1-y4) / (x1-x4);
      m43 = (y3-y4) / (x3-x4);
      m32 = (y2-y3) / (x2-x3);

      b12 = y1 - m12*x1;
      b43 = y4 - m43*x4;
      b41 = y4 - m41*x4;
      b32 = y3 - m32*x3;

      //Take lat_data and find the corresponding y values on the lines from point 1-2
      //and point 4-3. These y values are a range.
      //Test long_data against those corresponding y values for a vertical check
      ycp1 = m12*lat_data + b12;
      ycp2 = m43*lat_data + b43;

      //Take long_data and find the corresponding x values on the lines from point 4-1
      //and point 3-2.  These x values are a range.
      //Test lat_data against those corresponding x values for a horizontal check
      xcp1 = (long_data - b41) / m41;
      xcp2 = (long_data - b32) / m32;

      if( (long_data <= ycp1) && (long_data >= ycp2) )//Check within vertical boundaries
      {
             if( (lat_data <= xcp2) && (lat_data >= xcp1) )//Check within horizontal boundaries
             {
                   return 1; //We're inside the fence
             }
      }

      return 0; //DEAR GOD WE'RE BAD
}//end check_boundaries

