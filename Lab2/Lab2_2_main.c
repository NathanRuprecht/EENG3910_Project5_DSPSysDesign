/*************************************************
 Blink LEDs on Tiva C Series LaunchPad
 with SysTick timer.

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
#include "driverlib/systick.h" // Deifinitions and macros for SysTick timer API of DriverLib.


#define TIMER0_FREQ    4.7 // Freqency in Hz


// function prototypes
void init_LEDs(void);
void init_SysTick(void);
void SysTick_ISR(void);


uint32_t sys_clock;


int main(void)
{
	// Configure system clock at 80 MHz.
	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	//sys_clock = SysCtlClockGet();
	sys_clock = 80000000;


	init_LEDs();
	init_SysTick();

    IntMasterEnable();
    SysTickEnable();

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


void init_SysTick(void)
{
	// Enable and configure SysTick timer.
	// The period of the SysTick counter must be between 1 and 16777216, inclusive.
	// That is, the SysTick timer is a 24-bit timer. pow(2,24)=16777216.
    SysTickPeriodSet(sys_clock/TIMER0_FREQ);

	// Registers a function to be called when the interrupt occurs.
	SysTickIntRegister(SysTick_ISR);
    SysTickIntEnable();
}


void SysTick_ISR(void)
{
	// Blink green LED.
	// Read the current state of GPIO pin and write back the opposite state.
	if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3)) {
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
	}
	else {
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 8);
	}
}
