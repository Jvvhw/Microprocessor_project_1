/*
miproject7.h
miproject7.c
variable.h
Build this main.c with above-mentioned files in the same folder

2023.05.18 JunHwan
*/

#include "stm32f767xx.h"
#include "OK-STM767.h"
#include "variable.h"
#include "miproject7.h"

/* -------- HEAD -------- */

void ADC_IRQHandler(void)       /* Fire watchdog interrupt function */
{
  NVIC->ICPR[0] = 0x00040000;  // clear pending bit of (18)ADC1
  ADC1->SR = 0x00000000;	     // clear AWD flag

  static int detection = 0;
  static int prev_direction = 3;

  if(DeviceMode == MONITORING_MODE)
  {
    if(detection == 0)    // first detection
    {
      newLimit_A = read_encoder();    
      prev_direction = motor_direction();
      detection++;
      Beep();
    }
    else if(detection !=0 && prev_direction != motor_direction()) // second detection
    {
      newLimit_B = read_encoder();
      prev_direction = motor_direction();
      update_flag = 1;
      Beep();
    }
  }
  else if(DeviceMode == EXTINGUISHING_MODE)
  {
    detection = 0;
    prev_direction = 3;
    // water pump
  }
}

void EXTI0_IRQHandler(void)			/* EXTI0(KEY1) interrupt function */
{ 
  LCD_command(0x01);    
  LCD_string(0x80,"Motor Align...");
  align_motor();              // align motor by magnet sensor
  LCD_string(0x80,"Complete!");

  while((GPIOC->IDR & 0x00000001) != 0x00000001); // debouncing
  Delay_ms(20);
  EXTI->PR = 0x00000001;			// clear pending bit of EXTI0
  NVIC->ICPR[0] = 0x00000040;			// clear pending bit of (6)EXTI0
}

void EXTI1_IRQHandler(void)			/* EXTI1(KEY2) interrupt function */
{
  change_direction(FORWARD);
  LCD_command(0x01);
  LCD_string(0x86,"START");

  while((GPIOC->IDR & 0x00000002) != 0x00000002); // debouncing
  Delay_ms(20);
  EXTI->PR = 0x00000002;			// clear pending bit of EXTI1
  NVIC->ICPR[0] = 0x00000080;			// clear pending bit of (7)EXTI1
}

void EXTI2_IRQHandler(void)			/* EXTI2(KEY3) interrupt function */
{
  change_direction(STOP);
  LCD_command(0x01);
  LCD_string(0x86,"STOP");

  while((GPIOC->IDR & 0x00000004) != 0x00000004); // debouncing
  Delay_ms(20);
  EXTI->PR = 0x00000004;			// clear pending bit of EXTI2
  NVIC->ICPR[0] = 0x00000100;			// clear pending bit of (8)EXTI2
}

void EXTI3_IRQHandler(void)			/* EXTI3(KEY4) interrupt function */
{
  DeviceMode = MONITORING_MODE;
  update_limit(0,PPR_HALF);
  LCD_command(0x01);
  LCD_string(0x86,"RESET");
  
  while((GPIOC->IDR & 0x00000008) != 0x00000008); // debouncing
  Delay_ms(20);
  EXTI->PR = 0x00000008;			// clear pending bit of EXTI3
  NVIC->ICPR[0] = 0x00000200;			// clear pending bit of (9)EXTI3
}

/* -------- BODY -------- */

int main(void)
{
  // MCU
  Initialize_MCU();				// initialize MCU and kit
  Delay_ms(50);					  // wait for system stabilization

  //LCD
  Initialize_LCD();				// initialize text LCD module
  

  //PWM
  InitTIM1();   //TIM1 initialize
  InitPwm();    //PWM initialize

  // Encoder
  InitTIM4();
  InitEnc();

  // ADC
  InitADC1();
  unsigned int result = 0;

  // GPIO
  InitGPIO();

  //Interrupt
  InitINT();    //Interrupt initialize

  Beep();

  LCD_command(0x01);    //clear display
  ADC1->CR2 |= 0x40000000;    // start conversion by software
  update_limit(0,PPR_HALF);   // set 180 degree monitoring
  change_direction(STOP);    // Initial direction forward
  
  while(1)
    {
      result = ADC1->DR;
      LCD_command(0xC6);
      LCD_unsigned_decimal(result, 1, 4);

      LCD_command(0xC0);
      LCD_unsigned_decimal(TIM4->CNT,1,5);

      if(motor_direction() == FORWARD && read_encoder() >= secondLimit)
      {
        change_direction(BACKWARD);
      }
      else if(motor_direction() == BACKWARD && read_encoder() <= firstLimit+10)
      {
        change_direction(FORWARD);
      }

      if(update_flag)
      {
        update_limit(newLimit_A,newLimit_B);
        update_flag = 0;
        DeviceMode = EXTINGUISHING_MODE;
      }
    }
}
