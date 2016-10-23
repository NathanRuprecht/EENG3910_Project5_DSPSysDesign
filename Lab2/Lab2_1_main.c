/*************************************************
 Blink LEDs on Tiva C Series LaunchPad
 with Timer0.

 By Dr. Xinrong Li, xinrong@UNT.EDU, Aug. 25, 2014
 Adapted from Tiva example code.

 Edited by Nathan Ruprecht
 	 nathan.ruprecht@outlook.com
 	 1 February 2016
 *************************************************/

#include <stdint.h> // Variable definitions for the C99 standard.
#include <stdbool.h> // Boolean definitions for the C99 standard.

#include "inc/tm4c123gh6pm.h" // Definitions for the interrupt and register assignments.
#include "inc/hw_memmap.h" // Memory map definitions of the Tiva C Series device.
#include "inc/hw_types.h" // Definitions of common types and macros.

#include "driverlib/sysctl.h" // Definitions and macros for System Control API of DriverLib.
#include "driverlib/interrupt.h" // Definitions and macros for NVIC Controller API of driverLib.
#include "driverlib/gpio.h" // Definitions and macros for GPIO API of DriverLib.
#include "driverlib/timer.h" // Definitions and macros for Timer API of driverLib.


#define TIMER0_FREQ    20 // Freqency in Hz


// function prototypes
void init_LEDs(void);
void init_timer(void);
void Timer0_ISR(void);


uint32_t sys_clock;


int main(void)
{
	// Configure system clock at 40 MHz.
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	sys_clock = SysCtlClockGet();

	init_LEDs();
	init_timer();

	// Enable the processor to respond to interrupts.
	IntMasterEnable();
	// Start the timer by enabling operation of the timer module.
	TimerEnable(TIMER0_BASE, TIMER_A);

	while(1) {
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
}


void Timer0_ISR(void)
{
	// Clear the timer interrupt.
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	// Blink blue LED.
	// Read the current state of GPIO pin and write back the opposite state.
	if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2)) {
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
	}
	else {
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 4);
	}
}
