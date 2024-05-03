#define SET_BIT(REG, BIT_POSN)    (REG |= (1 <<BIT_POSN))
#define CLEAR_BIT(REG, BIT_POSN)  (REG &= ~(1<<BIT_POSN))
#define TOGGLE_BIT(REG, BIT_POSN) (REG ^=(1 << BIT_POSN))
#define READ_BIT(REG, BIT_POSN)   (REG>>BIT_POSN) & 1




#define INPUT  (0U)
#define OUTPUT (1U)

#define STD_LOW   0x00U
#define STD_HIGH  0x01U



typedef signed char int8;
typedef unsigned char uint8;
typedef unsigned long long uint64;
typedef  long long int64;
//typedef unsigned float ufloat32;
typedef  float float32;
//typedef unsigned double udouble64;
typedef  double sdouble64;
typedef short int16;
typedef unsigned short uint16;
typedef int int32;
typedef unsigned int uint32;
typedef signed char* int8_ptr;
typedef unsigned char* uint8_ptr;
typedef short* int16_ptr;
typedef unsigned short* uint16_ptr;
typedef int* int32_ptr;
typedef unsigned int* uint32_ptr;

typedef unsigned long long* uint64_ptr;
typedef  long long* int64_ptr;
//typedef unsigned float* ufloat32_ptr;
typedef  float* float32_ptr;
//typedef unsigned double* udouble64_ptr;
typedef  double* sdouble64_ptr;

#define NULL_PTR                 ((void*)0)