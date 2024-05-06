#include"DIO.h"
//#include"type.h"
#include"tm4c123gh6pm.h"
#include"bitwise_operation.h"

void DIO_Init(int PORT_NO){
  if(PORT_NO == PORTF)
  {
   SYSCTL_RCGCGPIO_R |= 0x00000020;
  
  while((SYSCTL_PRGPIO_R&0x00000020) == 0){};
  GPIO_PORTF_LOCK_R = 0x4C4F434B; 
  GPIO_PORTF_CR_R = 0x1F;
  }
  else if(PORT_NO==PORTE){
    SYSCTL_RCGCGPIO_R |= 0x00000010;
  
  while((SYSCTL_PRGPIO_R&0x00000010) == 0){};
  GPIO_PORTE_LOCK_R = 0x4C4F434B; 
  GPIO_PORTE_CR_R = 0x1F;
    
  }
  else if(PORT_NO==PORTD){
    SYSCTL_RCGCGPIO_R |= 0x00000008;
  
  while((SYSCTL_PRGPIO_R&0x00000008) == 0){};
  GPIO_PORTD_LOCK_R = 0x4C4F434B; 
  GPIO_PORTD_CR_R = 0x1F;
    
  }
  else if(PORT_NO==PORTC){
    SYSCTL_RCGCGPIO_R |= 0x00000004;
  
  while((SYSCTL_PRGPIO_R&0x00000004) == 0){};
  GPIO_PORTC_LOCK_R = 0x4C4F434B; 
  GPIO_PORTC_CR_R = 0x1F;
    
  }
  else if(PORT_NO==PORTB){
    SYSCTL_RCGCGPIO_R |= 0x00000002;
  
  while((SYSCTL_PRGPIO_R&0x00000002) == 0){};
  GPIO_PORTB_LOCK_R = 0x4C4F434B; 
  GPIO_PORTB_CR_R = 0x1F;
    
  }
  else if(PORT_NO==PORTA){
    SYSCTL_RCGCGPIO_R |= 0x00000001;
  
  while((SYSCTL_PRGPIO_R&0x00000001) == 0){};
  GPIO_PORTA_LOCK_R = 0x4C4F434B; 
  GPIO_PORTA_CR_R = 0x1F;
    
  }
}
void DIO_WritePort(int PORT_NO ,int value){

  if(PORT_NO == PORTF){
    GPIO_PORTF_DATA_R=value;
  }
  else if(PORT_NO==PORTE){
    GPIO_PORTE_DATA_R=value;
  }
  else if(PORT_NO==PORTD){
   GPIO_PORTD_DATA_R=value;
  }
  else if(PORT_NO==PORTC){
   GPIO_PORTC_DATA_R=value;
  }
  else if(PORT_NO==PORTB){
   GPIO_PORTB_DATA_R=value;
  }
  else if(PORT_NO==PORTA){
   GPIO_PORTA_DATA_R=value;
  }
  
}
void DIO_WritePin(unsigned int Pin, int PORT_NO ,int value){
   if(PORT_NO == PORTF){
     if (value==HIGH){
       SET_BIT(&GPIO_PORTF_DATA_R,Pin);
     }
     else
     {
       CLEAR_BIT(&GPIO_PORTF_DATA_R,Pin);
     }
  }
  else if(PORT_NO==PORTE){
     if (value==HIGH){
       SET_BIT(&GPIO_PORTF_DATA_R,Pin);
     }
     else
     {
       CLEAR_BIT(&GPIO_PORTF_DATA_R,Pin);
     }
  }
  else if(PORT_NO==PORTD){
    if (value==HIGH){
       SET_BIT(&GPIO_PORTD_DATA_R,Pin);
     }
     else
     {
       CLEAR_BIT(&GPIO_PORTD_DATA_R,Pin);
     }
  }
  else if(PORT_NO==PORTC){
    if (value==HIGH){
       SET_BIT(&GPIO_PORTC_DATA_R,Pin);
     }
     else
     {
       CLEAR_BIT(&GPIO_PORTC_DATA_R,Pin);
     }
  }
  else if(PORT_NO==PORTB){
    if (value==HIGH){
       SET_BIT(&GPIO_PORTB_DATA_R,Pin);
     }
     else
     {
       CLEAR_BIT(&GPIO_PORTB_DATA_R,Pin);
     }
  }
  else if(PORT_NO==PORTA){
    if (value==HIGH){
       SET_BIT(&GPIO_PORTA_DATA_R,Pin);
     }
     else
     {
       CLEAR_BIT(&GPIO_PORTA_DATA_R,Pin);
     }
  }
  
}
int DIO_ReadPin(unsigned int Pin,int PORT_NO){
  if(PORT_NO == PORTF){
    return GET_BIT(&GPIO_PORTF_DATA_R,Pin);
  }
  else if(PORT_NO==PORTE){
    return GET_BIT(&GPIO_PORTF_DATA_R,Pin);
  }
  else if(PORT_NO==PORTD){
    return GET_BIT(&GPIO_PORTD_DATA_R,Pin);
  }
  else if(PORT_NO==PORTC){
    return GET_BIT(&GPIO_PORTC_DATA_R,Pin);
  }
  else if(PORT_NO==PORTB){
    return GET_BIT(&GPIO_PORTB_DATA_R,Pin);
  }
  else if(PORT_NO==PORTA){
    return GET_BIT(&GPIO_PORTA_DATA_R,Pin);
  }
  return 0;
}


