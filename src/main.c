#include <stdint.h>
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include "tm4c123gh6pm.h"
#include "DIO.h"
#include "queue.h"

int cntr = 0;
QueueHandle_t xQueue;

char *itoa(int num, char *str, int base)
{
	int i = 0;
	int isNegative = 0;

	// Handle negative numbers
	if (num < 0 && base == 10)
	{
		isNegative = 1;
		num = -num;
	}

	// Process individual digits
	do
	{
		int rem = num % base;
		str[i++] = (rem > 9) ? (rem - 10) + 'a' : rem + '0';
		num /= base;
	} while (num);

	// If number is negative, append '-'
	if (isNegative)
	{
		str[i++] = '-';
	}

	str[i] = '\0'; // Null-terminate the string

	// Reverse the string
	int j = 0;
	int len = i;
	for (j = 0; j < len / 2; j++)
	{
		char temp = str[j];
		str[j] = str[len - j - 1];
		str[len - j - 1] = temp;
	}

	return str;
}

void UART5_init(void)
{
	SYSCTL_RCGCUART_R |= 0x20; /* enable clock to UART5 */
	SYSCTL_RCGCGPIO_R |= 0x10; /* enable clock to PORTE for PE4/Rx and RE5/Tx */
	/* UART0 initialization */
	UART5_CTL_R = 0;	 /* UART5 module disbable */
	UART5_IBRD_R = 104;	 /* for 9600 baud rate, integer = 104 */
	UART5_FBRD_R = 11;	 /* for 9600 baud rate, fractional = 11*/
	UART5_CC_R = 0;		 /*select system clock*/
	UART5_LCRH_R = 0x60; /* data lenght 8-bit, not parity bit, no FIFO */
	UART5_CTL_R = 0x301; /* Enable UART5 module, Rx and Tx */

	/* UART5 TX5 and RX5 use PE4 and PE5. Configure them digital and enable alternate function */
	GPIO_PORTE_DEN_R |= 0x30;		/* set PE4 and PE5 as digital */
	GPIO_PORTE_AFSEL_R = 0x30;		/* Use PE4,PE5 alternate function */
	GPIO_PORTE_AMSEL_R = 0;			/* Turn off analg function*/
	GPIO_PORTE_PCTL_R = 0x00110000; /* configure PE4 and PE5 for UART */
}

void UART0_init(void)
{
	SYSCTL_RCGCUART_R |= 0x01; /* Enable clock to UART0 */
	SYSCTL_RCGCGPIO_R |= 0x01; /* Enable clock to Port A for PA0/Rx and PA1/Tx */

	/* UART0 initialization */
	UART0_CTL_R = 0;	 /* UART0 module disable */
	UART0_IBRD_R = 104;	 /* Integer part of the baud rate divisor for 9600 baud rate */
	UART0_FBRD_R = 11;	 /* Fractional part of the baud rate divisor for 9600 baud rate */
	UART0_CC_R = 0;		 /* Select system clock as the UART clock source */
	UART0_LCRH_R = 0x60; /* Data length 8-bit, no parity bit, no FIFO */
	UART0_CTL_R = 0x301; /* Enable UART0 module, Rx and Tx */

	/* UART0 TX0 and RX0 use PA1 and PA0. Configure them digital and enable alternate function */
	GPIO_PORTA_DEN_R |= 0x03;		/* Set PA0 and PA1 as digital */
	GPIO_PORTA_AFSEL_R |= 0x03;		/* Use PA0, PA1 alternate function */
	GPIO_PORTA_AMSEL_R = 0;			/* Turn off analog function */
	GPIO_PORTA_PCTL_R = 0x00000011; /* Configure PA0 and PA1 for UART0 */
}

void UART_Write(unsigned char data)
{
	while ((UART0_FR_R & (1 << 5)) != 0)
		;			   /* wait until Tx buffer not full */
	UART0_DR_R = data; /* before giving it another byte */
}

void UART_Write_String(char *str)
{
	while (*str)
	{
		UART_Write(*(str++));
	}
}

void raiseUp()
{
	
}

void MotorDriver(int direction)
{
	static int semaphore;
	semaphore++;
	if (semaphore == 1)
	{
		if (direction)
		{
			// raise up
		}
		else
		{
			// lower down
		}
	}
	else
	{
		// pass
	}
	semaphore--;
}

void AutoUp()
{
	// how to detect limit switch is fired
}

void PassengerListner(void *pvParameters)
{
	int btn1State;
	while (1)
	{
		btn1State = GPIO_PORTF_DATA_R & (1 << 0);
		if (btn1State == 0)
		{ // Assuming active-low button
			GPIO_PORTF_DATA_R |= 0x02;
			// raise window
			// or use semaphores

			vTaskDelay(100 / portTICK_PERIOD_MS); // Debounce delay
		}
		else
		{
			GPIO_PORTF_DATA_R &= ~0x02;
			// lower window
		}
	}
}

void DriverListner(void *pvParameters)
{
	while (1)
	{
		int btn2State = GPIO_PORTF_DATA_R & (1 << 4);
		if (btn2State == 0)
		{
			GPIO_PORTF_DATA_R = 0x04;
			//

			vTaskDelay(500 / portTICK_PERIOD_MS); // Debounce delay
		}
		else
		{
			GPIO_PORTF_DATA_R &= ~(0x04);
		}
	}
}

void LimitSwitchListner()
{
	while (1)
	{
		int upState = GPIO_PORTF_DATA_R & (1 << 4);	  // get from gpio
		int downState = GPIO_PORTF_DATA_R & (1 << 4); // get from gpio
		int objectDetected = GPIO_PORTF_DATA_R & (1 << 4);
		if (upState == 0)
		{
			if (objectDetected == 0)
			{
				while (true)
				{
					// lower down
				}
			}
			else
			{
				// operation is done
			}
		}
		elif (downState == 0)
		{
			// operation is done
		}
		else
		{
			taskYield();
		}
	}
}

void Task3(void *pvParameters)
{
	int rcvCntr;
	while (1)
	{
		if (xQueueReceive(xQueue, &rcvCntr, portMAX_DELAY))
		{
			// printf("Entered here");
			char final_message[20];
			sprintf(final_message, "Recieved: %d\n", cntr);
			cntr = 0;
			UART_Write_String(final_message); // put rcvcntr
			xQueueReset(xQueue);
			taskYIELD();
		}
	}
}

void psh_btn_init()
{
	SYSCTL_RCGCGPIO_R |= 0x20; // Enable clock for PORTF
	GPIO_PORTF_LOCK_R = 0x4C4F434B;
	GPIO_PORTF_CR_R = 0x11;
	GPIO_PORTF_DIR_R &= ~0x11; // PF0 as input
	GPIO_PORTF_DIR_R |= 0x0E;  // PF1 as output
	GPIO_PORTF_DEN_R |= 0x1F;  // Enable digital I/O for PF0 and PF1
	GPIO_PORTF_PUR_R = 0x11;
}

void Init()
{
	psh_btn_init();
	UART0_init();
}

int main()
{
	Init();
	xQueue = xQueueCreate(10, sizeof(uint32_t));

	/*BaseType_t xTaskCreate( TaskFunction_t pxTaskCode,
							const char * const pcName,
							const configSTACK_DEPTH_TYPE usStackDepth,
							void * const pvParameters,
							UBaseType_t uxPriority,
							TaskHandle_t * const pxCreatedTask );*/
	xTaskCreate(PassengerListner, "PB1", 200, NULL, 1, NULL);
	xTaskCreate(DriverListner, "PB2", 200, NULL, 1, NULL);
	xTaskCreate(Task3, "Uart0", 200, NULL, 1, NULL);
	// Startup of the FreeRTOS scheduler.  The program should block here.
	vTaskStartScheduler();

	// The following line should never be reached.
	// Failure to allocate enough memory from the heap could be a reason.
	for (;;)
		;
}
