#include "DIO.h"
#include "std_types.h"

void DIO_Init(Dio_PortType port_name)
{
    SET_BIT(SYSCTL_RCGCGPIO_R, port_name); // Activate the clock of the MCU to the PORT
    while (READ_BIT(SYSCTL_PRGPIO_R, port_name) == 0)
    {
    }; // Loop untill check if it is enabled
}

void DIO_DIR(Dio_PortType port_name, Dio_PinType pin_num, Dio_PinDirectionType direction)
{
    volatile Dio_PortType_ptr LOCK_PTR = NULL_PTR;
    volatile Dio_PortType_ptr CR_PTR = NULL_PTR;
    volatile Dio_PortType_ptr DIR_PTR = NULL_PTR;
    volatile Dio_PortType_ptr PUR_PTR = NULL_PTR;
    volatile Dio_PortType_ptr DEN_PTR = NULL_PTR;

    switch (port_name)
    {
    case PORTA:
        DIR_PTR = (Dio_PortType_ptr)&GPIO_PORTA_DIR_R;
        PUR_PTR = (Dio_PortType_ptr)&GPIO_PORTA_PUR_R;
        DEN_PTR = (Dio_PortType_ptr)&GPIO_PORTA_DEN_R;
        break;

    case PORTB:
        DIR_PTR = (Dio_PortType_ptr)&GPIO_PORTB_DIR_R;
        PUR_PTR = (Dio_PortType_ptr)&GPIO_PORTB_PUR_R;
        DEN_PTR = (Dio_PortType_ptr)&GPIO_PORTB_DEN_R;
        break;

    case PORTC:
        LOCK_PTR = (Dio_PortType_ptr)&GPIO_PORTC_LOCK_R;
        CR_PTR = (Dio_PortType_ptr)&GPIO_PORTC_CR_R;
        DIR_PTR = (Dio_PortType_ptr)&GPIO_PORTC_DIR_R;
        PUR_PTR = (Dio_PortType_ptr)&GPIO_PORTC_PUR_R;
        DEN_PTR = (Dio_PortType_ptr)&GPIO_PORTC_DEN_R;
        break;

    case PORTD:
        LOCK_PTR = (Dio_PortType_ptr)&GPIO_PORTD_LOCK_R;
        CR_PTR = (Dio_PortType_ptr)&GPIO_PORTD_CR_R;
        DIR_PTR = (Dio_PortType_ptr)&GPIO_PORTD_DIR_R;
        PUR_PTR = (Dio_PortType_ptr)&GPIO_PORTD_PUR_R;
        DEN_PTR = (Dio_PortType_ptr)&GPIO_PORTD_DEN_R;
        break;

    case PORTE:
        DIR_PTR = (Dio_PortType_ptr)&GPIO_PORTE_DIR_R;
        PUR_PTR = (Dio_PortType_ptr)&GPIO_PORTE_PUR_R;
        DEN_PTR = (Dio_PortType_ptr)&GPIO_PORTE_DEN_R;
        break;

    case PORTF:
        LOCK_PTR = (Dio_PortType_ptr)&GPIO_PORTF_LOCK_R;
        CR_PTR = (Dio_PortType_ptr)&GPIO_PORTF_CR_R;
        DIR_PTR = (Dio_PortType_ptr)&GPIO_PORTF_DIR_R;
        PUR_PTR = (Dio_PortType_ptr)&GPIO_PORTF_PUR_R;
        DEN_PTR = (Dio_PortType_ptr)&GPIO_PORTF_DEN_R;
        break;

    default:
        break;
    }

    if (direction == OUTPUT)
    {
        SET_BIT(*DIR_PTR, pin_num); // 3. Direction
    }
    else
    {
        CLEAR_BIT(*DIR_PTR, pin_num);
        *LOCK_PTR = 0x4C4F434B; // 1. Unlock the PORT [if exist]
        // GPIO_PORTF_CR_R = 0x1F;
        SET_BIT(*CR_PTR, pin_num);
        // GPIO_PORTF_PUR_R = 0x11;
        SET_BIT(*PUR_PTR, pin_num);
    }
    SET_BIT(*DEN_PTR, pin_num); // 5. Enable the pins
}
void DIO_togglepin(Dio_PortType port_name, Dio_PinType pin_num){
    volatile Dio_PortType_ptr DATA_PTR = NULL_PTR;
    switch (port_name)
    {
    case PORTA:
        DATA_PTR = (Dio_PortType_ptr)&GPIO_PORTA_DATA_R;
        break;
    case PORTB:
        DATA_PTR = (Dio_PortType_ptr)&GPIO_PORTB_DATA_R;
        break;
    case PORTC:
        DATA_PTR = (Dio_PortType_ptr)&GPIO_PORTC_DATA_R;
        break;
    case PORTD:
        DATA_PTR = (Dio_PortType_ptr)&GPIO_PORTD_DATA_R;
        break;
    case PORTE:
        DATA_PTR = (Dio_PortType_ptr)&GPIO_PORTE_DATA_R;
        break;
    case PORTF:
        DATA_PTR = (Dio_PortType_ptr)&GPIO_PORTF_DATA_R;
        break;
    default:
        break;
    }
    
    TOGGLE_BIT(*DATA_PTR, pin_num);
    
  
}

void DIO_WritePin(Dio_PortType port_name, Dio_PinType pin_num, Dio_ValueType value)
{
    volatile Dio_PortType_ptr DATA_PTR = NULL_PTR;
    switch (port_name)
    {
    case PORTA:
        DATA_PTR = (Dio_PortType_ptr)&GPIO_PORTA_DATA_R;
        break;
    case PORTB:
        DATA_PTR = (Dio_PortType_ptr)&GPIO_PORTB_DATA_R;
        break;
    case PORTC:
        DATA_PTR = (Dio_PortType_ptr)&GPIO_PORTC_DATA_R;
        break;
    case PORTD:
        DATA_PTR = (Dio_PortType_ptr)&GPIO_PORTD_DATA_R;
        break;
    case PORTE:
        DATA_PTR = (Dio_PortType_ptr)&GPIO_PORTE_DATA_R;
        break;
    case PORTF:
        DATA_PTR = (Dio_PortType_ptr)&GPIO_PORTF_DATA_R;
        break;
    default:
        break;
    }

    if (value == STD_HIGH)
    {
        SET_BIT(*DATA_PTR, pin_num);
    }
    else
    {
        CLEAR_BIT(*DATA_PTR, pin_num);
    }
}

void DIO_WritePort(Dio_PortType port_name, Dio_ValueType value)
{

    switch (port_name)
    {
    case PORTA:
        GPIO_PORTA_DATA_R = value;
        break;
    case PORTB:
        GPIO_PORTB_DATA_R = value;
        break;
    case PORTC:
        GPIO_PORTC_DATA_R = value;
        break;
    case PORTD:
        GPIO_PORTD_DATA_R = value;
        break;
    case PORTE:
        GPIO_PORTE_DATA_R = value;
        break;
    case PORTF:
        GPIO_PORTF_DATA_R = value;
        break;
    default:
        break;
    }
}
