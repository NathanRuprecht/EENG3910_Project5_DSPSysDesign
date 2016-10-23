/*************************************************
 Sample onboard temprature sensor, and
 print the temp value on UART terminal,
 with FPU enabled.

 By Dr. Xinrong Li, xinrong@UNT.EDU, Sept. 10, 2014

 Edited by Nathan Ruprecht
 	 nathan.ruprecht@outlook.com
 	 21 February 2016
 *************************************************/

#include <stdint.h> // Variable definitions for the C99 standard.
#include <stdio.h> // Input and output facilities for the C99 standard.
#include <stdbool.h> // Boolean definitions for the C99 standard.

#include "inc/tm4c123gh6pm.h" // Definitions for the interrupt and register assignments.
#include "inc/hw_memmap.h" // Memory map definitions of the Tiva C Series device.
#include "inc/hw_types.h" // Definitions of common types and macros.

#include "driverlib/sysctl.h" // Definitions and macros for System Control API of DriverLib.
#include "driverlib/interrupt.h" // Defines and macros for NVIC Controller API of DriverLib.
#include "driverlib/gpio.h" // Definitions and macros for GPIO API of DriverLib.
#include "driverlib/timer.h" // Defines and macros for Timer API of DriverLib.
#include "driverlib/pin_map.h" //Mapping of peripherals to pins for all parts.
#include "driverlib/uart.h" // Definitions and macros for UART API of DriverLib.
#include "driverlib/adc.h" // Definitions for ADC API of DriverLib.
#include "driverlib/fpu.h" // Prototypes for the FPU manipulation routines.

#include "utils/uartstdio.h" // Prototypes for the UART console functions.
							 // Needs to add "utils/uartstdio.c" through a relative link.

#define TIMER0_FREQ    2 // Freqency in Hz
#define TIMER1_FREQ    0.5 // Frequency in Hz
#define UART0_BAUDRATE    115200 // UART baudrate in bps

#define ADC0_SEQ_NUM 0 // ADC Sample Sequence Number

#define RED_LED    GPIO_PIN_1
#define BLUE_LED    GPIO_PIN_2
#define GREEN_LED    GPIO_PIN_3

#define DISP_TEXT_LINE_NUM    4
#define TEMP_STR_LEN    20


// function prototypes
void init_LEDs(void);
void init_timer(void);
void Timer0_ISR(void);
void Timer1_ISR(void);
void init_UART(void);
void init_ADC(void);

extern void UARTStdioIntHandler(void);


// global variables
uint32_t sys_clock;
uint8_t cur_LED = RED_LED;

const char *disp_text[DISP_TEXT_LINE_NUM] = {
		"\n",
		"UART and LED Demo\n",
		"H: help, R: red, G: green, B: blue\n",
		"> " };

uint32_t cur_temp=0;
float cur_temp_C=0, cur_temp_F=0;
char temp_str[TEMP_STR_LEN];


int main(void)
{
	uint32_t i;
	unsigned char user_cmd;

	// Configure system clock at 40 MHz.
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	sys_clock = SysCtlClockGet();

	// Enable the floating-point unit (FPU).
	FPUEnable();
	// Configure FPU to perform lazy stacking of the floating-point state.
	FPULazyStackingEnable();

	init_LEDs();
	init_ADC();
	init_UART();
	init_timer();

	// Enable the processor to respond to interrupts.
	IntMasterEnable();

	// Start the timer by enabling operation of the timer module.
	TimerEnable(TIMER0_BASE, TIMER_A);
	TimerEnable(TIMER1_BASE, TIMER_A);

	// Initial display on terminal.
	for(i=0; i<DISP_TEXT_LINE_NUM; i++)
		UARTprintf(disp_text[i]);

	while(1) {
		// Read user inputs from UART if available.
		if(UARTRxBytesAvail())
	        user_cmd = UARTgetc();
		else
			user_cmd = 0;

		switch(user_cmd){
		case '\r':
		case ' ':
		case 'H':
		case 'h':
			for(i=0; i<DISP_TEXT_LINE_NUM; i++)
				UARTprintf(disp_text[i]);
			break;
		case 'R':
		case 'r':
			cur_LED = RED_LED;
			UARTprintf("\n> ");
			break;
		case 'B':
		case 'b':
			cur_LED = BLUE_LED;
			UARTprintf("\n> ");
			break;
		case 'G':
		case 'g':
			cur_LED = GREEN_LED;
			UARTprintf("\n> ");
			break;
		}
	}
}


void init_LEDs(void)
{
	// Enable and configure LED peripheral.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Enable GPIO Port F.
	// Three onboard LEDs, R:PF1, B:PF2, G:PF3.
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
}


void init_timer(void)
{
	// Enable and configure timer peripheral.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

	// Configure Timer0 as a 32-bit timer in periodic mode.
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	// Initialize timer load register.
	TimerLoadSet(TIMER0_BASE, TIMER_A, sys_clock/TIMER0_FREQ -1);

	// Configure Timer1 as a 32-bit timer in periodic mode.
	TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
	// Initialize timer load register.
	TimerLoadSet(TIMER1_BASE, TIMER_A, sys_clock/TIMER1_FREQ -1);


	// Registers a function to be called when the interrupt occurs.
	IntRegister(INT_TIMER0A, Timer0_ISR);
	// The specified interrupt is enabled in the interrupt controller.
	IntEnable(INT_TIMER0A);
	// Enable the indicated timer interrupt source.
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	// Registers a function to be called when the interrupt occurs.
	IntRegister(INT_TIMER1A, Timer1_ISR);
	// The specified interrupt is enabled in the interrupt controller.
	IntEnable(INT_TIMER1A);
	// Enable the indicated timer interrupt source.
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
}


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
}


void init_ADC(void)
{
	// Enable and configure ADC0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	ADCSequenceConfigure(ADC0_BASE, ADC0_SEQ_NUM, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, ADC0_SEQ_NUM, 0, ADC_CTL_TS|ADC_CTL_IE|ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, ADC0_SEQ_NUM);
}


// Timer0 interrupt service routine
void Timer0_ISR(void)
{
	// Clear the timer interrupt.
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	// Blink LED. Read the current state of GPIO pins and write back the opposite state.
	if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3)) {
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
	}
	else {
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, cur_LED);
	}
}


// Timer1 interrupt service routine
void Timer1_ISR(void)
{
	// Clear the timer interrupt.
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	ADCProcessorTrigger(ADC0_BASE, ADC0_SEQ_NUM);

	while(!ADCIntStatus(ADC0_BASE, ADC0_SEQ_NUM, false)) {
	}
	ADCIntClear(ADC0_BASE, ADC0_SEQ_NUM);

	ADCSequenceDataGet(ADC0_BASE, ADC0_SEQ_NUM, &cur_temp);

	cur_temp_C = 147.5 - (247.5*cur_temp)/4096; // Convert to degree C.
	cur_temp_F = (cur_temp_C*9 + 160)/5; // Convert to degree F.

	snprintf(temp_str, TEMP_STR_LEN, "%.1fC = %.1fF", cur_temp_C, cur_temp_F);
	// Print current temperature to UART0
	UARTprintf("\nTemp = %s\n> ", temp_str);

}
