void DIO_Init(int PORT_NO);
//void DIO_WritePin();
//void DIO_WritePort();
void DIO_WritePin(unsigned int Pin, int PORT_NO ,int value);
void DIO_WritePort(int PORT_NO ,int value);
int DIO_ReadPin(unsigned int Pin,int PORT_NO);
int DIO_ReadPort(unsigned int *  volatile Port);
 
#define PORTF 6
#define PORTE 5
#define PORTD 4
#define PORTC 3
#define PORTB 2
#define PORTA 1
#define HIGH 1
#define LOW 0