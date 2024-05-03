#include <stdint.h>
#include <stdio.h>
#include <freertos.h>
#include <task.h>
#include "tm4c123gh6pm.h"
#include "DIO.h"
#include "queue.h"

int cntr = 0;
QueueHandle_t xQueue;

// TODO: make queues for message passing that if the passenger or driver wanted to use the MotorDriver
// the MotorDriver will listen for the messages and execute them accordingly

// Define variables
volatile bool buttonPressed = false;
volatile TickType_t buttonPressStartTime = 0;

// Task for Button Monitoring
void buttonTask(void *pvParameters)
{
	while (1)
	{
		// Check button state
		if (buttonIsPressed())
		{
			if (!buttonPressed)
			{
				// Button has been pressed
				buttonPressed = true;
				buttonPressStartTime = xTaskGetTickCount(); // Record start time
			}
		}
		else
		{
			// Button released
			buttonPressed = false;
		}
		vTaskDelay(pdMS_TO_TICKS(50)); // Check button every 50ms
	}
}

// Timer callback function
void buttonTimerCallback(TimerHandle_t xTimer)
{
	// Check if button has been pressed for more than 1 second
	int currentTime = xTaskGetTickCount();
	if (buttonPressed && (currentTime - buttonPressStartTime >= pdMS_TO_TICKS(1000)))
	{
		// Button has been pressed for more than 1 second
		// send queue message raise up
	}
	else if (buttonPressed && (currentTime - buttonPressStartTime < pdMS_TO_TICKS(1000)))
	{
		// Button pressed for less than 1 second
		// send queue message auto roll
	}
}

// Task for Timer Management
void timerTask(void *pvParameters)
{
	TimerHandle_t buttonTimer = xTimerCreate("ButtonTimer", pdMS_TO_TICKS(100), pdFALSE, 0, buttonTimerCallback);
	if (buttonTimer != NULL)
	{
		while (1)
		{
			// Start or stop the timer based on button state
			if (buttonPressed)
			{
				xTimerStart(buttonTimer, 0);
			}
			else
			{
				xTimerStop(buttonTimer, 0);
			}
			vTaskDelay(pdMS_TO_TICKS(100)); // Check timer every 100ms
		}
	}
}

void raiseUp()
{
	
}

void MotorDriver(int direction)
{
	// listen for queue instructions
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
}

int main()
{
	Init();
	xQueue = xQueueCreate(10, sizeof(uint32_t));

	xTaskCreate(PassengerListner, "PB1", 200, NULL, 1, NULL);
	xTaskCreate(DriverListner, "PB2", 200, NULL, 1, NULL);
	xTaskCreate(Task3, "Uart0", 200, NULL, 1, NULL);

	xTaskCreate(buttonTask, "ButtonTask", 200, NULL, 2, NULL);
	xTaskCreate(timerTask, "TimerTask", 200, NULL, 2, NULL);

	vTaskStartScheduler();

	for (;;)
		;
}
