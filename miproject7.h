#ifndef INCLUDE_MIPROJECT7_H_
#define INCLUDE_MIPROJECT7_H_

#include "stm32f767xx.h"

#define DEADTIME 0
//5.e-6

/*
------Timer------
*/
extern unsigned int PwmMaxCnt,PwmHalfCnt;
extern unsigned int PWM, tenpercent ;    // pulse width, 10% duty
extern int speedFactor;
extern unsigned int firstLimit, secondLimit;        // motor degree limit
extern unsigned int newLimit_A, newLimit_B;     // update limit
extern int update_flag;     

typedef enum {
    MONITORING_MODE = 0,
    EXTINGUISHING_MODE = 1
} Mode;

extern Mode DeviceMode;

extern void InitTIM1(void);
extern void InitTIM4(void);
extern void InitPwm(void);
extern void InitEnc(void);

extern void ConfigPwm(unsigned int PwmMaxCnt);

extern void align_motor(void);
extern int motor_direction(void);
extern void change_direction(int DIR);
extern void rotate_motor(int DIR, int degree);
extern void reset_encoder();
extern unsigned int read_encoder(void);
extern void update_limit(unsigned int A, unsigned int B);

/*
------GPIO------
*/
extern void InitGPIO(void);

extern int isMagnetOn();

/*
------ADC------
*/
extern void InitADC1(void);
extern void setAWDTH(unsigned int HTR, unsigned int LTR);


/*
--------Intterupt--------
*/
extern void InitINT(void);
// extern void TIM1_UP_TIM10_IRQHandler(void);     /* TIM1 interrupt function*/
extern void EXTI0_IRQHandler(void);			    /* EXTI0 interrupt function */
extern void EXTI1_IRQHandler(void);			    /* EXTI1 interrupt function */
extern void EXTI2_IRQHandler(void);			    /* EXTI2 interrupt function */
extern void EXTI3_IRQHandler(void);			    /* EXTI3 interrupt function */
extern void ADC_IRQHandler(void);               /* ADC1 interrupt function */

#endif 
