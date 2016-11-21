/*************************************************
 Signal output with PWM

 By Dr. Xinrong Li, xinrong@UNT.EDU, Mar. 6, 2015

 Edited by Nathan Ruprecht
 	 nathan.ruprecht@outlook.com
 	 29 February 2016
 *************************************************/

#include <stdint.h> // Variable definitions for the C99 standard.
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
#include "driverlib/pwm.h" // Definitions for PWM API of DriverLib.
#include "utils/uartstdio.h" // Prototypes for the UART console functions.
							 // Needs to add "utils/uartstdio.c" through a relative link.
#include "inc/hw_gpio.h" // Defines and Macros for GPIO hardware.
#include <math.h>
#include <stdio.h>
#include "driverlib/fpu.h" // Prototypes for the FPU manipulation routines.
#include "driverlib/ssi.h" // Definitions and prototypes for the SSI/SPI routines.


#define TIMER0_FREQ    2 // Freqency in Hz, heartbeat timer
#define UART0_BAUDRATE    115200 // UART baudrate in bps

#define PWM_FREQ    55 // Frequency in Hz
#define PWM_DUTYCYCLE_BASE    100
#define PWM_DUTYCYCLE_MIN    10
#define PWM_DUTYCYCLE_MAX    90

#define RED_LED    GPIO_PIN_1
#define BLUE_LED    GPIO_PIN_2
#define GREEN_LED    GPIO_PIN_3

#define NUM_BUTTONS    2
#define LEFT_BUTTON    GPIO_PIN_4
#define RIGHT_BUTTON    GPIO_PIN_0
#define NUM_DEBOUNCE_CHECKS    10 // For 50 msec debounce time.

#define SAMP_FREQ    3000 // Frequency in Hz, common sampling rates for digital audio: 44100, 48000
#define SIG_FREQ    1000 // Signal frequency in Hz.
#define SIG_LEN    3 // = SAMP_FREQ/SIG_FREQ, period of discrete-time signal

#define SPI_BITRATE    15000000 // needs to > SAMP_FREQ*16, and < SysCtrlClock/2, max 20M
#define SPI_DATA_MASK    0x0FFF // See datasheet, MCP4921
#define SPI_CTRL_MASK    0x5000 // See datasheet, MCP4921

#define NUM_DISP_TEXT_LINE    4


// function prototypes
void init_LEDs(void);
void init_timer(void);
void init_UART(void);
void init_PWM(void);
void Timer0_ISR(void);
void Timer1_ISR(void);
void init_buttons(void);
void set_button_states(void);
void init_SPI(void);
void signal(void);


extern void UARTStdioIntHandler(void);


// global variables
uint8_t cur_LED = RED_LED;
const char *disp_text[NUM_DISP_TEXT_LINE] = {
		"\n",
		"UART and LED Demo\n",
		"H: help, R: red, G: green, B: blue. Left button: increase dutycycle. Right button: decrease dutycycle \n",
		"> " };
uint32_t sys_clock;
uint32_t PWM_clock;
uint32_t PWM_timer_load;

// PWM duty cycle: percentage of one period in which the signal is high
uint8_t PWM_dutycycle = 25; //  duty cycle = PWM_dutycycle/PWM_DUTYCYCLE_BASE
uint8_t PWM_dutycycle_step = 5;
uint16_t sig[1];
uint16_t out;

volatile uint8_t raw_button_states[NUM_DEBOUNCE_CHECKS]; // Raw button states in circular buffer.
volatile uint32_t raw_button_states_index=0;
volatile uint8_t button_states=0;


int main(void)
{
	uint32_t i;
	unsigned char user_cmd;
	uint8_t saved_button_states=0, cur_button_states;

	// Configure system clock.
	//SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); // 50 MHz
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); // 40 MHz
	sys_clock = SysCtlClockGet();
	//SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); // 80 MHz
	//sys_clock = 80000000; // Hard coded for 80MHz because of a bug in SysCtlClockGet().

	init_LEDs();
	init_UART();
	init_timer();
	init_PWM();
	init_buttons();
	init_SPI();
	signal();

	// Enable the processor to respond to interrupts.
	IntMasterEnable();

	// Start the timer by enabling operation of the timer module.
	TimerEnable(TIMER0_BASE, TIMER_A);
	TimerEnable(TIMER1_BASE, TIMER_A);

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
		case '\r':
		case ' ':
		case 'H':
		case 'h':
			for(i=0; i<NUM_DISP_TEXT_LINE; i++)
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

			// Left button pressed, then released
			if((saved_button_states & LEFT_BUTTON) && (~cur_button_states & LEFT_BUTTON)) {
				UARTprintf("\n> ");
				PWM_dutycycle = PWM_dutycycle + PWM_dutycycle_step;
				if(PWM_dutycycle > PWM_DUTYCYCLE_MAX) {
					PWM_dutycycle = PWM_DUTYCYCLE_MAX;
					UARTprintf("PWM_dutycycle has reached max value.\n> ");
				}
				PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, PWM_dutycycle*PWM_timer_load/PWM_DUTYCYCLE_BASE);
				signal();
				UARTprintf("\n%d \n", PWM_dutycycle);
			}

			// Right button pressed, then released
			if((saved_button_states & RIGHT_BUTTON) && (~cur_button_states & RIGHT_BUTTON)) {
				UARTprintf("\n> ");
				PWM_dutycycle = PWM_dutycycle - PWM_dutycycle_step;
				if(PWM_dutycycle < PWM_DUTYCYCLE_MIN) {
					PWM_dutycycle = PWM_DUTYCYCLE_MIN;
					UARTprintf("PWM_dutycycle has reached min value.\n> ");
				}
				PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, PWM_dutycycle*PWM_timer_load/PWM_DUTYCYCLE_BASE);
				signal();
				UARTprintf("\n%d \n", PWM_dutycycle);
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

	// Enable and configure Timer1 peripheral.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	// Configure as a 32-bit timer in periodic mode.
	TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
	// Initialize timer load register.
	TimerLoadSet(TIMER1_BASE, TIMER_A, sys_clock/SAMP_FREQ -1);

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


void init_PWM(void)
{
	// PWM clock is set to be system clock divided by the specified divider, 64 in this case.
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
	PWM_clock = sys_clock/64;

	// Enable and configure PC4 (M0PWM6) as PWM output
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
	GPIOPinConfigure(GPIO_PE4_M0PWM4);

	PWM_timer_load = (PWM_clock/PWM_FREQ) -1;
	PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, PWM_timer_load);

	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, PWM_dutycycle*PWM_timer_load/PWM_DUTYCYCLE_BASE);
	PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
	PWMGenEnable(PWM0_BASE, PWM_GEN_3);
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
	//Jumps to set_button_states function
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


void init_SPI(void)
{
	// Enable peripheral for SSI/SPI.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

	// Configure the muxing and GPIO settings to bring the SSI/SPI functions out to the pins
	// PA2: SSI0CLK, PA3: SSI0FSS, PA5: SSI0TX
	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	GPIOPinConfigure(GPIO_PA5_SSI0TX);

	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5);

	SSIConfigSetExpClk(SSI0_BASE, sys_clock, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_BITRATE, 16);
	SSIEnable(SSI0_BASE);
}


void signal(void)
{
	out=PWM_dutycycle*40.95;
	sig[1] = SPI_CTRL_MASK | (SPI_DATA_MASK & out);
		// Send data to SPI.
		SSIDataPut(SSI0_BASE, sig[1]);
}
