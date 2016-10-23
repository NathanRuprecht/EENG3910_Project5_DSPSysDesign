/*************************************************
 Blink LEDs on Tiva C Series LaunchPad
 with looped delay.

 By Dr. Xinrong Li, xinrong@UNT.EDU, Aug. 25, 2014
 Adapted from Tiva example code.

 Edited by Nathan Ruprecht
 	 nathan.ruprecht@outlook.com
 	 1 February 2016
 *************************************************/

#include <stdint.h> // Variable definitions for the C99 standard
#include <stdbool.h> // Boolean definitions for the C99 standard

#include "inc/hw_memmap.h" // Memory map definitions of the Tiva C Series device
#include "inc/hw_types.h" // Definitions of common types and macros

#include "driverlib/sysctl.h" // Definitions and macros for System Control API of DriverLib
#include "driverlib/gpio.h" // Definitions and macros for GPIO API of DriverLib

int main(void)
{
	uint8_t pin_data=2;
	uint32_t delay_count = 1.33e7; // For about 2s delay at 20 MHz clock rate.
	// delay_count = 2.67e7 For about 2s delay at 40 MHz clock rate

	// Configure system clock at 20 MHz.
	SysCtlClockSet(SYSCTL_SYSDIV_10|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

	// Configure system clock at 40 MHz.
//	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

	// Configure system clock at 50 MHz.
//	SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

	// Configure system clock at 80 MHz.
//	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);


	// Enable GPIO Port F
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	// Three onboard LEDs, R:PF1, B:PF2, G:PF3.
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

	while(1) {
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, pin_data);
		SysCtlDelay(delay_count);  // Loop timer. Each loop is 3 CPU cycles.

		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
		SysCtlDelay(delay_count);

		if(pin_data == 8)
			pin_data = 2;
		else
			pin_data = pin_data*2;
	}
}
