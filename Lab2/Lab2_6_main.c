/*************************************************
 Programming the two pushbuttons on Tiva LaunchPad

 By Dr. Xinrong Li, xinrong@UNT.EDU, Feb. 09, 2015
 Adapted from Tiva example code.

 Edited by Nathan Ruprecht
 	 nathan.ruprecht@outlook.com
 	 14 February 2016
 *************************************************/

#include <stdint.h> // Variable definitions for the C99 standard.
#include <stdbool.h> // Boolean definitions for the C99 standard.

#include "inc/tm4c123gh6pm.h" // Definitions for the interrupt and register assignments.
#include "inc/hw_memmap.h" // Memory map definitions of the Tiva C Series device.
#include "inc/hw_types.h" // Definitions of common types and macros.
#include "inc/hw_gpio.h" // Defines and Macros for GPIO hardware.

#include "driverlib/sysctl.h" // Definitions and macros for System Control API of DriverLib.
#include "driverlib/interrupt.h" // Definitions and macros for NVIC Controller API of DriverLib.
#include "driverlib/gpio.h" // Definitions and macros for GPIO API of DriverLib.
#include "driverlib/timer.h" // Definitions and macros for Timer API of DriverLib.
#include "driverlib/pin_map.h" //Mapping of peripherals to pins for all parts.
#include "driverlib/uart.h" // Definitions and macros for UART API of DriverLib.

#include "utils/uartstdio.h" // Prototypes for the UART console functions.
							 // Needs to add "utils/uartstdio.c" through a relative link.

#define RED_LED    GPIO_PIN_1
#define BLUE_LED    GPIO_PIN_2
#define GREEN_LED    GPIO_PIN_3

#define NUM_BUTTONS    2
#define LEFT_BUTTON    GPIO_PIN_4
#define RIGHT_BUTTON    GPIO_PIN_0
#define NUM_DEBOUNCE_CHECKS    10 // For 50 msec debounce time.

#define TIMER0_FREQ    2 // Freqency in Hz, for blinking LED.
#define TIMER1_FREQ    200 // Frequency in Hz, for pushbutton debouncing.
#define UART0_BAUDRATE    115200 // UART baudrate in bps.

#define NUM_DISP_TEXT_LINES    4


// function prototypes
void init_LEDs(void);
void init_timer(void);
void init_UART(void);
void init_buttons(void);
void set_button_states(void);

void Timer0_ISR(void);
void Timer1_ISR(void);

extern void UARTStdioIntHandler(void);


// global variables
uint32_t sys_clock;
uint8_t cur_LED = RED_LED;
const char *disp_text[NUM_DISP_TEXT_LINES] = {
		"\n",
		"UART and LED Demo\n",
		"H: help, R: red, G: green, B: blue.\n",
		"> " };

volatile uint8_t raw_button_states[NUM_DEBOUNCE_CHECKS]; // Raw button states in circular buffer.
volatile uint32_t raw_button_states_index=0;
volatile uint8_t button_states=0;


int main(void)
{
	uint32_t i;
	unsigned char user_cmd;
	uint8_t saved_button_states=0, cur_button_states;

	// Configure system clock at 40 MHz.
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	sys_clock = SysCtlClockGet();

	init_LEDs();
	init_buttons();
	init_UART();
	init_timer();

	// Enable interrupt.
	IntMasterEnable();

	// Enable timers.
	TimerEnable(TIMER0_BASE, TIMER_A);
	TimerEnable(TIMER1_BASE, TIMER_A);

	// Initial display on terminal.
	for(i=0; i<NUM_DISP_TEXT_LINES; i++)
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
			for(i=0; i<NUM_DISP_TEXT_LINES; i++)
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

		// Check button states.
		cur_button_states = button_states;
		if(saved_button_states != cur_button_states){
			if((~saved_button_states & LEFT_BUTTON) && (cur_button_states & LEFT_BUTTON)) {
				UARTprintf("Left button pushed down.\n> ");
				cur_LED = RED_LED;
			}
			if((saved_button_states & LEFT_BUTTON) && (~cur_button_states & LEFT_BUTTON)) {
				UARTprintf("Left button released.\n> ");
			}
			if((~saved_button_states & RIGHT_BUTTON) && (cur_button_states & RIGHT_BUTTON)) {
				UARTprintf("Right button pushed down.\n> ");
				cur_LED = GREEN_LED;
			}
			if((saved_button_states & RIGHT_BUTTON) && (~cur_button_states & RIGHT_BUTTON)) {
				UARTprintf("Right button released.\n> ");
			}
			if(cur_button_states == (LEFT_BUTTON | RIGHT_BUTTON)) {
				UARTprintf("Both buttons held down.\n> ");
				cur_LED = BLUE_LED;
			}


			saved_button_states = cur_button_states;
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
	// enable and configure UART0 for debugging printouts.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Registers a function to be called when the interrupt occurs.
	IntRegister(INT_UART0, UARTStdioIntHandler);
	UARTStdioConfig(0, UART0_BAUDRATE, sys_clock);
}


// Timer0 interrupt service routine
void Timer0_ISR(void)
{
	// Clear the timer interrupt.
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	// Blink blue LED.
	// Read the current state of GPIO pins and write back the opposite state.
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
	set_button_states();
}


void init_buttons(void)
{
    uint32_t i;

	// Enable the GPIO port connected to the two onboard pushbuttons.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // PF0 is muxed with NMI, so unlock it first to configure as a GPIO input.
    // Then re-lock it to prevent further changes.
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    // Set each of the button GPIO pins as an input with a pull-up.
    GPIODirModeSet(GPIO_PORTF_BASE, LEFT_BUTTON|RIGHT_BUTTON, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTF_BASE, LEFT_BUTTON|RIGHT_BUTTON,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Initialize global variable.
    for(i=0; i<NUM_DEBOUNCE_CHECKS; i++)
        raw_button_states[i] = 0;
}


// This function should be called regularly in a timer ISR.
void set_button_states(void)
{
	uint32_t i;
	uint8_t states = LEFT_BUTTON|RIGHT_BUTTON;

	raw_button_states[raw_button_states_index] = ~ GPIOPinRead(GPIO_PORTF_BASE, LEFT_BUTTON|RIGHT_BUTTON);

	if(raw_button_states_index >= NUM_DEBOUNCE_CHECKS-1)
		raw_button_states_index = 0;
	else
		raw_button_states_index ++;

	for(i=0; i<NUM_DEBOUNCE_CHECKS; i++)
	    states = states & raw_button_states[i];

	button_states = states;
}
