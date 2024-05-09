#ifndef DIO_H_
#define DIO_H_
#include "tm4c123gh6pm.h"
#include "std_types.h"
// PORTs Names:
#define PORTA 0
#define PORTB 1
#define PORTC 2
#define PORTD 3
#define PORTE 4
#define PORTF 5

// PINs Names:
#define PIN0 0
#define PIN1 1
#define PIN2 2
#define PIN3 3
#define PIN4 4
#define PIN5 5
#define PIN6 6
#define PIN7 7

typedef uint32 Dio_PortType;
typedef uint32_ptr Dio_PortType_ptr;
typedef uint8 Dio_PinType;
typedef uint8 Dio_PinDirectionType;
typedef uint8 Dio_ValueType;

void DIO_Init(Dio_PortType port_name);

void DIO_DIR(Dio_PortType port_name, Dio_PinType pin_num, Dio_PinDirectionType direction);

void DIO_WritePin(Dio_PortType port_name, Dio_PinType pin_num, Dio_ValueType value);

void DIO_WritePort(Dio_PortType port_name, Dio_ValueType value);
void DIO_togglepin(Dio_PortType port_name, Dio_PinType pin_num);
#endif