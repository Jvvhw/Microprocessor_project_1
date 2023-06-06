#ifndef INCLUDE_VARIABLE_H_
#define INCLUDE_VARIABLE_H_

// System setting
#define TIM_CLK         ((float)108.e6)
#define TIM_CLK_HALF    ((float)54.e6)

// Switching frequency
#define Fsw 5.e3
#define Tsw 1./Fsw

// General purpose define
#define OFF 0
#define ON 1

// PWM
#define PPR 4680
#define PPR_HALF 2340
#define PPD 13
#define BACKWARD 0
#define FORWARD 1
#define STOP 2


// ADC
#define ADC_MAX  4095
#define ADC_FIRE 1500
#define ADC_MIN 0

// Interrupt

#endif 
