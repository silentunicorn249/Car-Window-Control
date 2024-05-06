#include <stdint.h>
#include <stdio.h>
#include <freertos.h>
#include <task.h>
#include "tm4c123gh6pm.h"
#include "DIO.h"
#include "queue.h"
#include "timers.h"

void timer0Init(void);
void timer0_Delay(int time);

short lastDirection = 0; // directions are -1, 0, 1
QueueHandle_t xQueue;

// TODO: make queues for message passing that if the passenger or driver wanted to use the MotorDriver
// the MotorDriver will listen for the messages and execute them accordingly

volatile char buttonPressed = 0;
volatile TickType_t buttonPressStartTime = 0;

int Button_Pressed(void)
{
	return !(GPIO_PORTF_DATA_R & 0x01); // Return 1 if button is pressed (active low)
}

void MotorDriver(int direction)
{
	int rcvCntr;
	if (xQueueReceive(xQueue, &rcvCntr, portMAX_DELAY))
	{
		if (lastDirection == 1 && rcvCntr == -1)
		{
			lastDirection = 0;
			// stop motion
		}
		else if (lastDirection == -1 && rcvCntr == 1)
		{
			lastDirection = 0;
			// stop motion
		}
		if (rcvCntr == 0)
		{
			lastDirection = 1;
			// auto roll up
		}
		else if (rcvCntr == 1)
		{
			// manual up
		}
		else if (rcvCntr == 2)
		{
			lastDirection = -1;
			// auto roll down
		}
		else if (rcvCntr == 3)
		{
			// manual down
		}
	}
}

void PassengerListner(void *pvParameters)
{
	while (1)
	{
		int cntr = 0;
		if (Button_Pressed())
		{

			timer0_Delay(5000);

			if (Button_Pressed())
			{
				// Turn on one LED, turn off the other
				cntr = 1;
				xQueueSend(xQueue, &cntr, portMAX_DELAY);
			}
			else
			{
				// TODO: make variable that listens if the button is still pressed after long press
				cntr = 0;
				xQueueSend(xQueue, &cntr, portMAX_DELAY);
			}

			timer0_Delay(10000);
		}
		// reset leds, may be helpful in jamming
		GPIO_PORTF_DATA_R &= ~(1 << 1);
		GPIO_PORTF_DATA_R &= ~(1 << 2);

		vTaskDelay(pdMS_TO_TICKS(50)); // Check button every 50ms
	}
}

void DriverListner(void *pvParameters)
{
	// TODO: add lock
	while (1)
	{
		int cntr = 0;
		if (Button_Pressed())
		{

			timer0_Delay(5000);

			if (Button_Pressed())
			{
				// Turn on one LED, turn off the other
				cntr = 1;
				xQueueSend(xQueue, &cntr, portMAX_DELAY);
			}
			else
			{
				// TODO: make variable that listens if the button is still pressed after long press
				cntr = 0;
				xQueueSend(xQueue, &cntr, portMAX_DELAY);
			}

			timer0_Delay(10000);
		}
		// reset leds, may be helpful in jamming
		GPIO_PORTF_DATA_R &= ~(1 << 1);
		GPIO_PORTF_DATA_R &= ~(1 << 2);

		vTaskDelay(pdMS_TO_TICKS(50)); // Check button every 50ms
	}
}

void LimitSwitchListner()
{
	while (1)
	{
		int cntr = 0;
		int upState = GPIO_PORTF_DATA_R & (1 << 4);	  // get from gpio
		int downState = GPIO_PORTF_DATA_R & (1 << 4); // get from gpio
		int objectDetected = GPIO_PORTF_DATA_R & (1 << 4);
		if (upState == 0)
		{
			if (objectDetected == 0)
			{
				while (1)
				{
					// lower down
					cntr = 1;
					if (xQueueSend(xQueue, &cntr, portMAX_DELAY) == pdPASS)
					{
					}
				}
			}
			else
			{
				// operation is done
			}
		}
		else if (downState == 0)
		{
			// operation is done
		}
		else
		{
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
	timer0Init();
}

// Timer callback function
void timerCallback(TimerHandle_t xTimer)
{
	// Timer expired, perform actions here
	GPIO_PORTF_DATA_R ^= 0x02;
}

int main()
{
	Init();
	xQueue = xQueueCreate(10, sizeof(uint32_t));

	// xTaskCreate(PassengerListner, "PB1", 200, NULL, 1, NULL);
	// xTaskCreate(DriverListner, "PB2", 200, NULL, 1, NULL);

	xTaskCreate(PassengerListner, "ButtonTask", 200, NULL, 2, NULL);
	xTaskCreate(MotorDriver, "Motor", 200, NULL, 2, NULL);

	vTaskStartScheduler();

	for (;;)
		;
}

void timer0Init(void)
{
	SYSCTL_RCGCTIMER_R |= 0x01;
	TIMER0_CTL_R = 0x00;
	TIMER0_CFG_R = 0x00;
	TIMER0_TAMR_R = 0x02;
	TIMER0_CTL_R = 0x03;
}
void timer0_Delay(int time)
{
	TIMER0_CTL_R = 0x00;
	TIMER0_TAILR_R = 16000 * time - 1;
	TIMER0_ICR_R = 0x01;
	TIMER0_CTL_R |= 0x03;
	while ((TIMER0_RIS_R & 0x01) == 0)
		;
}