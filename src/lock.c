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



void timer0Init(void);
void timer0_Delay(int time);

short lastDirection = 0; // directions are -1, 0, 1
QueueHandle_t xQueue;

// TODO: make queues for message passing that if the passenger or driver wanted to use the MotorDriver
// the MotorDriver will listen for the messages and execute them accordingly

volatile char buttonPressed = 0;
volatile TickType_t buttonPressStartTime = 0;


int Button_Pressed2(void)
{
	return !(GPIO_PORTF_DATA_R & 0x10); // Return 1 if button is pressed (active low)
}
int Lock_Enabled(void)
{
	return !(GPIO_PORTF_DATA_R & 0x01); // Return 1 if button is pressed (active low)
}

static void lock(void* pvParameters) {
    xSemaphoreTake(xLockSemaphore, 0);
    while (1) {
			//Take semaphore
        xSemaphoreTake(xLockSemaphore, portMAX_DELAY);
			
			//Check on Button State
				if (Lock_Enabled()){
					GPIO_PORTF_DATA_R |= (1<<3); 				// Turn RED LED ON for indication
					vTaskDelay(pdMS_TO_TICKS(500));
				}
				/*
				else if(GET_BIT(GPIO_PORTA_DATA_R,3)==1)
				{
					GPIO_PORTF_DATA_R &= ~(1<<1); 		// Turn RED LED OFF for indication
					vTaskPrioritySet(DriverHandle,1);	// CHange Driver Task Priority to 1
				}
				*/
				GPIO_PORTF_DATA_R &= ~(1<<3); 	
		}
	}














void MotorDriver(int direction)
{
	while (1){
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
			GPIO_PORTF_DATA_R |= (1 << 1);
			// auto roll up
		}
		else if (rcvCntr == 1)
		{
			// manual up
			GPIO_PORTF_DATA_R |= (1 << 2);
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
}

void PassengerListner(void *pvParameters)
{
	while (1)
	{
		int cntr = 0;
		if (Lock_Enabled())
		{

			vTaskDelay(pdMS_TO_TICKS(500));

			if (Lock_Enabled())
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

			//timer0_Delay(10000);
		}
		// reset leds, may be helpful in jamming
		vTaskDelay(pdMS_TO_TICKS(1000));
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
		if (Lock_Enabled())
		{

			timer0_Delay(5000);

			if (Lock_Enabled())
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
void PORTF_Interrupt_Enable(void);


void GPIOF_Handler(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xLockSemaphore, &xHigherPriorityTaskWoken);
  
    if (GPIO_PORTF_RIS_R & 0x10) {  
        // Clear the interrupt immediately
        GPIO_PORTF_ICR_R = 0x10;  // Writing a '1' to the 4th bit

        // Handle the interrupt event, e.g., give a semaphore
        
    }

    // Check if a context switch is required following the ISR
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
int main()
{
	DIO_Init(PORTF);
  DIO_DIR(PORTF,1,OUTPUT);
	DIO_DIR(PORTF,2,OUTPUT);
	DIO_DIR(PORTF,3,OUTPUT);
	DIO_DIR(PORTF,PIN4,INPUT);
	DIO_DIR(PORTF,PIN0,INPUT);
	PORTF_Interrupt_Enable();
	vSemaphoreCreateBinary(xLockSemaphore);
	xQueue = xQueueCreate(10, sizeof(uint32_t));
__asm("CPSIE i");
	// xTaskCreate(PassengerListner, "PB1", 200, NULL, 1, NULL);
	// xTaskCreate(DriverListner, "PB2", 200, NULL, 1, NULL);
	if( xLockSemaphore != NULL ){
xTaskCreate( lock, "lock", 200, NULL, 4, NULL );
	xTaskCreate(PassengerListner, "ButtonTask", 200, NULL, 1, NULL);
	xTaskCreate(MotorDriver, "Motor", 200, NULL, 2, NULL);

	vTaskStartScheduler();
	}

	for (;;)
		;
}





void PORTF_Interrupt_Enable(void)
{

	
GPIO_PORTF_IM_R &=0;
    // Configure PF4 for falling edge trigger
GPIO_PORTF_IS_R &= ~0x10;  // PF4 is edge-sensitive
GPIO_PORTF_IBE_R &= ~0x10; // PF4 is not both edges
GPIO_PORTF_IEV_R &= ~0x10; // PF4 falling edge event
GPIO_PORTF_ICR_R = 0x10;   // clear flag4
GPIO_PORTF_IM_R |= 0x10;   // arm interrupt on PF4
	
	
	// Enable Interrrupts on PORTF
 NVIC_EN0_R |= (1<<30);  // Enable interrupt 30 in NVIC (GPIOF)
    
	
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
