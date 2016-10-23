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

#define TIMER0_FREQ    2 // Freqency in Hz, heartbeat timer
#define UART0_BAUDRATE    115200 // UART baudrate in bps

#define SAMP_FREQ    16000 // Frequency in Hz, common sampling rates for digital audio: 44100, 48000
#define SIG_FREQ    1000 // Signal frequency in Hz.
#define SIG_LEN    16 // = SAMP_FREQ/SIG_FREQ, period of discrete-time signal

#define SPI_BITRATE    15000000 // needs to > SAMP_FREQ*16, and < SysCtrlClock/2, max 20M
#define SPI_DATA_MASK    0x0FFF // See datasheet, MCP4921
#define SPI_CTRL_MASK    0x7000 // See datasheet, MCP4921

#define MATH_PI    3.14159265358979323846

#define RED_LED    GPIO_PIN_1
#define BLUE_LED    GPIO_PIN_2
#define GREEN_LED    GPIO_PIN_3

#define NUM_DISP_TEXT_LINE    4

#define ADC0_SEQ_NUM	0

// function prototypes
void init_LEDs(void);
void init_timer(void);
void init_UART(void);
void init_SPI(void);
void init_signal(void);
void init_adc();

void Timer0_ISR(void);
void Timer1_ISR(void);

extern void UARTStdioIntHandler(void);

// global variables
uint8_t cur_LED = RED_LED;
const char *disp_text[NUM_DISP_TEXT_LINE] = {
		"\n",
		"UART and LED Demo\n",
		"T:test, R: red, G: green, B: blue.\n",
		"> ",
		"wORKING"};

uint32_t sig[SIG_LEN], sig_index=0, x=0;
uint32_t sys_clock;

int main(void)
{
	uint32_t i;
	unsigned char user_cmd;
	//sys_clock = SysCtlClockGet();
	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); // 80 MHz
	sys_clock = 80000000; // Hard-coded for 80MHz because of a bug in SysCtlClockGet().

	// Enable the floating-point unit (FPU).
	FPUEnable();
	// Configure FPU to perform lazy stacking of the floating-point state.
	FPULazyStackingEnable();

	init_LEDs();
	init_UART();
	init_timer();
	init_SPI();
	init_adc();
	init_signal();

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
		case 't':
		case 'T':
			UARTprintf(disp_text[5]);
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


void init_SPI(void)
{
	// Enable peripheral for SSI/SPI.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // enable gpio port A
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0); //enable ssi/spi

	// Configure the muxing and GPIO settings to bring the SSI/SPI functions out to the pins
	// PA2: SSI0CLK, PA3: SSI0FSS, PA5: SSI0TX
	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	GPIOPinConfigure(GPIO_PA5_SSI0TX);

	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5);

	SSIConfigSetExpClk(SSI0_BASE, sys_clock, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_BITRATE, 16);
	SSIEnable(SSI0_BASE);
}


// Initialize signal table
void init_signal(void)
{
	uint32_t i;
	float w;

	w = 2*MATH_PI*SIG_FREQ/SAMP_FREQ;
	for(i=0; i<SIG_LEN; i++) {
		sig[i] = 0x0FFF*(1 + sinf(w*i))/2;
		sig[i] = SPI_CTRL_MASK | (SPI_DATA_MASK & sig[i]);
	}
}


//initialization of ADC
void init_adc(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); //enable port D
	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0); //Sets PD0 as ADC0 input

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);	//enable ADC0
	ADCSequenceConfigure(ADC0_BASE, ADC0_SEQ_NUM, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, ADC0_SEQ_NUM, 0, ADC_CTL_IE|ADC_CTL_CH7|ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, ADC0_SEQ_NUM);

}


// Timer1 interrupt service routine
void Timer1_ISR(void)
{
	// Clear the timer interrupt.
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	//ADC Read
	ADCProcessorTrigger(ADC0_BASE, ADC0_SEQ_NUM);

	while(!ADCIntStatus(ADC0_BASE, ADC0_SEQ_NUM, false)) {
	}

	ADCIntClear(ADC0_BASE, ADC0_SEQ_NUM);
	ADCSequenceDataGet(ADC0_BASE, ADC0_SEQ_NUM, &sig[x]);
	if(x < SIG_LEN-1){
		sig[x] = SPI_CTRL_MASK | (SPI_DATA_MASK & sig[x]);
		x++;
	}else{
		x=0;
	}

	// Send data to SPI.
	SSIDataPut(SSI0_BASE, sig[sig_index]);

	if(sig_index < SIG_LEN-1)
		sig_index++;
	else
		sig_index = 0;

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
