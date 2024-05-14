#include <stdint.h>
#include <stdio.h>
#include <freertos.h>
#include <task.h>
#include "tm4c123gh6pm.h"
#include "DIO.h"
#include "queue.h"
#include "timers.h"
#include <semphr.h>

xSemaphoreHandle xLockSemaphore;

void PORTF_Interrupt_Enable(void);
void PORTA_Interrupt_Enable(void);

short lastDirection = 0; // directions are -1, 0, 1
char windowActive = 1;
char windowUpActive = 1;
char windowDownActive = 1;
char lockEnabled = 0;

QueueHandle_t xQueue;
TaskHandle_t DriverHandle;
TaskHandle_t PassengerHandler;
xSemaphoreHandle xMutex;

int Lock_Enabled(void)
{
	return !(GPIO_PORTF_DATA_R & 0x01); // Return 1 if button is pressed (active low)
}

int UpLimitSwitchDetected(void)
{
	return !(GPIO_PORTA_DATA_R & 0x10); // Return 1 if button is pressed (active low)
}

int DownLimitSwitchDetected(void)
{
	return !(GPIO_PORTA_DATA_R & 0x20); // Return 1 if button is pressed (active low)
}

int JammingLimitSwitchDetected(void)
{
	return !(GPIO_PORTF_DATA_R & 0x10); // Return 1 if button is pressed (active low)
}

int UpPressedPassenger(void)
{
	return !(GPIO_PORTA_DATA_R & (1 << 2)); // Return 1 if button is pressed (active low)
}

int DownPressedPassenger(void)
{
	return !(GPIO_PORTA_DATA_R & (1 << 7)); // Return 1 if button is pressed (active low)
}

int UpPressedDriver(void)
{
	return !(GPIO_PORTA_DATA_R & (1 << 3)); // Return 1 if button is pressed (active low)
}

int DownPressedDriver(void)
{
	return !(GPIO_PORTA_DATA_R & (1 << 4)); // Return 1 if button is pressed (active low)
}

void MotorDriver()
{
	while (1)
	{
		int rcvCommand;
		if (xQueueReceive(xQueue, &rcvCommand, portMAX_DELAY))
		{
			if (windowActive)
			{
				if (lastDirection == 1 && rcvCommand == 3)
				{
					rcvCommand = -1;
					lastDirection = 0;
					GPIO_PORTF_DATA_R &= 0x08;
					GPIO_PORTF_DATA_R &= ~0x03; // 0xfff 11
					// GPIO_PORTF_DATA_R = 0;
					// stop motion
				}
				else if (lastDirection == -1 && rcvCommand == 1)
				{
					rcvCommand = -1;
					lastDirection = 0;
					GPIO_PORTF_DATA_R &= 0x08;
					GPIO_PORTF_DATA_R &= ~0x03;
					// GPIO_PORTF_DATA_R = 0;
					// stop motion
				}

				if (lastDirection == 0 && rcvCommand != -1)
				{

					// automatic up
					if (rcvCommand == 1)
					{
						lastDirection = 1;
						GPIO_PORTF_DATA_R |= (1 << 1);
						GPIO_PORTF_DATA_R &= ~(1 << 2);
					}


					// automatic down
					else if (rcvCommand == 3)
					{
						lastDirection = -1;
						GPIO_PORTF_DATA_R |= (1 << 2);
						GPIO_PORTF_DATA_R &= ~(1 << 1);
					}
				}
			}
		}
	}
}

void PassengerLisnter(void *pvParameters)
{
	while (1)
	{
		if(lockEnabled){
			taskYIELD();
			continue;
		}
		xSemaphoreTake(xMutex, portMAX_DELAY);
		int cntr = 0;
		if (UpPressedPassenger())
		{

			vTaskDelay(pdMS_TO_TICKS(500));

			if (UpPressedPassenger())
			{
				// manual
				while (UpPressedPassenger())
				{
					lastDirection = 1;
					GPIO_PORTF_DATA_R = (1 << 1);
					// vTaskDelay(pdMS_TO_TICKS(5)); // to go to other tasks
				}
				lastDirection = 0;
				GPIO_PORTF_DATA_R = 0;
			}
			else
			{
				// automatic
				cntr = 1;
				xQueueSend(xQueue, &cntr, 0);
			}
			vTaskDelay(pdMS_TO_TICKS(500));
		}
		else if (DownPressedPassenger())
		{

			vTaskDelay(pdMS_TO_TICKS(500));

			if (DownPressedPassenger())
			{

				// manual
				while (DownPressedPassenger())
				{
					GPIO_PORTF_DATA_R = (1 << 2);
					lastDirection = -1;
					// vTaskDelay(pdMS_TO_TICKS(5)); // to go to other tasks
				}
				lastDirection = 0;
				GPIO_PORTF_DATA_R = 0;

				// cntr = 2;
				// xQueueSend(xQueue, &cntr, portMAX_DELAY);
			}
			else
			{
				// automatic
				cntr = 3;
				xQueueSend(xQueue, &cntr, 0);
			}
			vTaskDelay(pdMS_TO_TICKS(500));
		}
		xSemaphoreGive(xMutex);

		vTaskDelay(pdMS_TO_TICKS(50)); // Check button every 50ms
	}
}

void DriverListner(void *pvParameters)
{
	while (1)
	{
		xSemaphoreTake(xMutex, portMAX_DELAY);
		int cntr = 0;
		if (UpPressedDriver())
		{

			vTaskDelay(pdMS_TO_TICKS(500));

			if (UpPressedDriver())
			{

				// manual
				while (UpPressedDriver())
				{
					lastDirection = 1;
					GPIO_PORTF_DATA_R = (1 << 1);
				}
				lastDirection = 0;
				GPIO_PORTF_DATA_R = 0;
			}
			else
			{

				// automatic
				cntr = 1;
				xQueueSend(xQueue, &cntr, 0);
			}
			vTaskDelay(pdMS_TO_TICKS(500));
		}
		else if (DownPressedDriver())
		{

			vTaskDelay(pdMS_TO_TICKS(500));

			if (DownPressedDriver())
			{

				// manual
				while (DownPressedDriver())
				{
					GPIO_PORTF_DATA_R = (1 << 2);
					lastDirection = -1;
				}
				lastDirection = 0;
				GPIO_PORTF_DATA_R = 0;
			}
			else
			{
				// automatic
				cntr = 3;
				xQueueSend(xQueue, &cntr, 0);
			}
			vTaskDelay(pdMS_TO_TICKS(500));
		}

		xSemaphoreGive(xMutex);
		vTaskDelay(pdMS_TO_TICKS(50)); // Check button every 50ms
	}
}

void LimitSwitchListner()
{
	while (1)
	{
		int cntr = 0;
		int upState = UpLimitSwitchDetected();
		int downState = DownLimitSwitchDetected();
		int objectDetected = JammingLimitSwitchDetected();
		if (upState && lastDirection == 1)
		{
			windowActive = 0;
		}
		else
		{
			windowActive = 1;
		}
		if (downState && lastDirection == -1)
		{
			windowActive = 0;
		}
		else
		{
			windowActive = 1;
		}
	}
}

void psh_btn_init()
{
	DIO_Init(PORTA);

	// tri-state driver
	DIO_DIR(PORTA, PIN3, INPUT);
	DIO_DIR(PORTA, PIN4, INPUT);

	// tri-state passenger
	DIO_DIR(PORTA, PIN2, INPUT);
	DIO_DIR(PORTA, PIN7, INPUT);

	// limit switch 1 (up limit)
	DIO_DIR(PORTA, PIN5, INPUT);
	// limit switch 2 (down limit)
	DIO_DIR(PORTA, PIN6, INPUT);

	DIO_Init(PORTF);
	// Lock
	DIO_DIR(PORTF, PIN0, INPUT);

	// jamming
	DIO_DIR(PORTF, PIN4, INPUT);

	// h-bridge
	DIO_DIR(PORTF, PIN3, OUTPUT);
	DIO_DIR(PORTF, PIN2, OUTPUT);
	DIO_DIR(PORTF, PIN1, OUTPUT);
}

void Init()
{
	psh_btn_init();
	PORTF_Interrupt_Enable();
}

// Timer callback function
void timerCallback(TimerHandle_t xTimer)
{
	// Timer expired, perform actions here
	GPIO_PORTF_DATA_R ^= 0x02;
}

void GPIOF_Handler(void)
{
	if (GPIO_PORTF_RIS_R & 0x01)
	{
		GPIO_PORTF_ICR_R = 0x01; // Writing a '1' to the 4th bit and 0th bit
		for (volatile uint32_t i = 0; i < 1000; i++)
		{
			__asm("NOP");
		}

		lockEnabled = !(GPIO_PORTF_DATA_R & 0x01);
		if (lockEnabled)
			SET_BIT(GPIO_PORTF_DATA_R, 3);
		else
			CLEAR_BIT(GPIO_PORTF_DATA_R, 3);
	}
	if (GPIO_PORTF_RIS_R & 0x10)
	{
		// Clear the interrupt immediately
		GPIO_PORTF_ICR_R = 0x10; // Writing a '1' to the 4th bit and 0th bit

		if (lastDirection == 1 && JammingLimitSwitchDetected())
		{
			lastDirection = -1;
			GPIO_PORTF_DATA_R = (1 << 2);

			for (volatile uint32_t i = 0; i < 6000000; i++)
			{
				__asm("NOP");
			}
			GPIO_PORTF_DATA_R = 0x0;
			lastDirection = 0;
		}
	}
}

int main()
{
	Init();

	vSemaphoreCreateBinary(xLockSemaphore);
    xMutex = xSemaphoreCreateMutex();
	xQueue = xQueueCreate(10, sizeof(uint32_t));
	__asm("CPSIE i");
	if (xLockSemaphore != NULL)
	{
		// xTaskCreate(lock, "lock", 200, NULL, 4, NULL);
		xTaskCreate(DriverListner, "Driver", 200, NULL, 1, &DriverHandle);
		xTaskCreate(PassengerLisnter, "Driver", 200, NULL, 1, &PassengerHandler);
		xTaskCreate(MotorDriver, "Motor", 200, NULL, 3, NULL);
		// xTaskCreate(LimitSwitchListner, "LimitSwitch", 200, NULL, 5, NULL);

		vTaskStartScheduler();
	}

	for (;;)
		;
}

void PORTF_Interrupt_Enable(void)
{

	GPIO_PORTF_IM_R &= 0;
	// Configure PF4 for falling edge trigger
	GPIO_PORTF_IS_R &= ~0x11;  // PF4 is edge-sensitive and PF0
	GPIO_PORTF_IBE_R &= ~0x11; // PF4 is not both edges and PF0
	GPIO_PORTA_IBE_R |= 0x01;  // Enable both edges interrupt for PF0
	GPIO_PORTF_IEV_R &= ~0x11; // PF4 falling edge event and PF0
	GPIO_PORTF_ICR_R = 0x11;   // clear flag4 and PF0
	GPIO_PORTF_IM_R |= 0x11;   // arm interrupt on PF4 and PF0

	// Enable Interrrupts on PORTF
	NVIC_EN0_R |= (1 << 30); // Enable interrupt 30 in NVIC (GPIOF)
}

void PORTA_Interrupt_Enable(void)
{

	GPIO_PORTF_IM_R &= 0;
	// Configure PF4 for falling edge trigger
	GPIO_PORTF_IS_R &= ~0x11;  // PF4 is edge-sensitive and PF0
	GPIO_PORTF_IBE_R &= ~0x11; // PF4 is not both edges and PF0
	GPIO_PORTA_IBE_R |= 0x01;  // Enable both edges interrupt for PF0
	GPIO_PORTF_IEV_R &= ~0x11; // PF4 falling edge event and PF0
	GPIO_PORTF_ICR_R = 0x11;   // clear flag4 and PF0
	GPIO_PORTF_IM_R |= 0x11;   // arm interrupt on PF4 and PF0

	// Enable Interrrupts on PORTF
	NVIC_EN0_R |= (1 << 30); // Enable interrupt 30 in NVIC (GPIOF)
}