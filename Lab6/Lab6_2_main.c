/*************************************************
 Signal pass through with triple buffering
 from ADC to DAC (MCP4921)

 By Dr. Xinrong Li, xinrong@UNT.EDU, Oct. 3, 2014

 Edited by Nathan Ruprecht
 	 nathan.ruprecht@outlook.com
 	 21 March 2016
 *************************************************/

#include <stdint.h> // Variable definitions for the C99 standard.
#include <stdbool.h> // Boolean definitions for the C99 standard.

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
#include "driverlib/ssi.h" // Prototypes for the SSI/SPI routines.
#include "driverlib/adc.h" // Definitions for ADC API of DriverLib.

#include "utils/uartstdio.h" // Prototypes for the UART console functions.
							 // Needs to add "utils/uartstdio.c" through a relative link.

#define TIMER0_FREQ    2 // Freqency in Hz, heartbeat timer

#define RED_LED    GPIO_PIN_1
#define BLUE_LED    GPIO_PIN_2
#define GREEN_LED    GPIO_PIN_3

#define UART0_BAUDRATE    115200 // UART baudrate in bps
#define NUM_DISP_TEXT_LINE    4

#define SAMP_FREQ    44100 // Frequency in Hz, common sampling rates for digital audio: 44100, 48000
#define ADC0_SEQ_NUM 0 // ADC Sample Sequence Number

#define SPI_BITRATE    15000000 // needs to > SAMP_FREQ*16, and < SysCtrlClock/2, max 20M
#define SPI_DATA_MASK    0x0FFF // See datasheet, MCP4921
#define SPI_CTRL_MASK    0x7000 // See datasheet, MCP4921

// Definitions for triple buffering.
#define NUM_BUFFER 4
#define BUFFER_LEN 100

// function prototypes
void init_LEDs(void);
void init_timer(void);
void init_UART(void);
void init_SPI(void);
void init_ADC(void);

void init_buffer(void);
void process_data(void);

void Timer0_ISR(void);
void Timer1_ISR(void);

extern void UARTStdioIntHandler(void);


// global variables
uint8_t cur_LED = RED_LED;
const char *disp_text[NUM_DISP_TEXT_LINE] = {
		"\n",
		"UART and LED Demo\n",
		"H: help, R: red, G: green, B: blue.\n",
		"> " };

uint32_t sys_clock;

volatile uint32_t ping_pong_buffer[NUM_BUFFER][BUFFER_LEN];
volatile uint32_t in_buff = 0, in_data = 1, out_buff = 2, out_data = 3;
volatile bool data_ready=false, buffer_overrun=false;


int main(void)
{
	uint32_t i;
	unsigned char user_cmd;

	// Configure system clock.
	//SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); // 50 MHz
	//SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); // 40 MHz
	//sys_clock = SysCtlClockGet();
	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); // 80 MHz
	sys_clock = 80000000; // Hard-coded for 80MHz because of a bug in SysCtlClockGet().

	init_LEDs();
	init_ADC();
	init_UART();
	init_timer();
	init_SPI();

	init_buffer();

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

		if(buffer_overrun) {
			// This error indicates the process_data() routine takes too much time to complete.
			// Optimize this routine to run faster or set the BUFFER_LEN larger.
			UARTprintf("\nError: buffer overrun!\n");
			buffer_overrun = false;
		}

		// SysCtlDelay(100000); // Un-comment this line to generate buffer_overrun error.

		if(data_ready) {
    		process_data();
    		data_ready = false;
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
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

	// Configure the muxing and GPIO settings to bring the SSI functions out to the pins
	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	GPIOPinConfigure(GPIO_PA5_SSI0TX);

	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5);

	SSIConfigSetExpClk(SSI0_BASE, sys_clock, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_BITRATE, 16);
	SSIEnable(SSI0_BASE);
}


void init_ADC(void)
{
	// Enable and configure ADC0. Sample from PD0/AIN7/BY
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0);

	ADCSequenceConfigure(ADC0_BASE, ADC0_SEQ_NUM, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, ADC0_SEQ_NUM, 0, ADC_CTL_IE|ADC_CTL_CH7|ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, ADC0_SEQ_NUM);
}


void init_buffer(void)
{
	uint32_t i, j;

	for(i=0; i<NUM_BUFFER; i++)
		for(j=0; j<BUFFER_LEN; j++) {
			ping_pong_buffer[i][j] = 0;
		}
}


void process_data(void)
{
	uint32_t i;

	//Do some processing here. For example:
	for(i=0; i<BUFFER_LEN; i++) {
		ping_pong_buffer[out_buff][i] = ping_pong_buffer[in_data][i]*3;
	}
}


// Timer1 interrupt service routine
void Timer1_ISR(void)
{
	static uint32_t sample_index=0;
	uint32_t data, tmp_ui32;

	// Clear the timer interrupt.
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	// Read data from ADC
	ADCProcessorTrigger(ADC0_BASE, ADC0_SEQ_NUM);
	while(!ADCIntStatus(ADC0_BASE, ADC0_SEQ_NUM, false)) {
	}
	ADCIntClear(ADC0_BASE, ADC0_SEQ_NUM);
	ADCSequenceDataGet(ADC0_BASE, ADC0_SEQ_NUM, &data);

	ping_pong_buffer[in_buff][sample_index] = data;

	// Send data to SPI DAC.
	SSIDataPut(SSI0_BASE, SPI_CTRL_MASK|(SPI_DATA_MASK&ping_pong_buffer[out_data][sample_index]));

	// Update buffer indices and sample index.
	if(++sample_index >= BUFFER_LEN) {
		sample_index = 0;

		if(data_ready) {
			buffer_overrun = true;
		}

		tmp_ui32 = in_buff;
		in_buff = in_data;
		in_data = tmp_ui32;

		tmp_ui32 = out_buff;
		out_buff = out_data;
		out_data = tmp_ui32;

		data_ready = true;
	}
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
