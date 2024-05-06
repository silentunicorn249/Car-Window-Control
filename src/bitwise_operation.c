#include"bitwise_operation.h"

inline int GET_BIT(unsigned long volatile  *Register,int BIT)
{
 return (*Register&(1<<BIT))>>BIT;
 
}
inline void SET_BIT(unsigned long volatile *Register,int BIT) 
{
  *Register|=(1<<BIT);
}

inline void CLEAR_BIT(unsigned long volatile *Register,int BIT)
{
  *Register&=~(1<<BIT);
}
inline void TOGGLE_BIT(unsigned long volatile *Register,int BIT)
{ 
  *Register ^=(1<<BIT);
}

