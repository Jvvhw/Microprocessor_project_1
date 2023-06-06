#include "miproject7.h"
#include "variable.h"

/*
------Timer------
*/

unsigned int PwmMaxCnt = 0;
unsigned int PwmHalfCnt = 0;
unsigned int PWM, tenpercent ;
int speedFactor = 1;
unsigned int firstLimit = 0;
unsigned int secondLimit = PPR >> 1;
unsigned int newLimit_A = 0;
unsigned int newLimit_B = 0;
int update_flag = 0;
Mode DeviceMode = MONITORING_MODE;

void InitTIM1(void) 
{
  GPIOE->MODER &= 0xFFF0FFFF;			// PE8,9 = alternate function mode
  GPIOE->MODER |= 0x000A0000;
  GPIOE->AFR[1] &= 0xFFFFFF00;		// PE8,9 = TIM1_CH1N, TIM1_CH1
  GPIOE->AFR[1] |= 0x00000011;
  RCC->APB2ENR |= 0x00000001;			// enable TIM1 clock
}

void InitTIM4(void)
{
  GPIOD->MODER &= 0xFAFFFFFF;			// PD12,13(TIM_CH1,2) = alternate function mode
  GPIOD->MODER |= 0x0A000000;
  GPIOD->AFR[1] &= 0xFF22FFFF;			// PD12,13 = TIM4_CH1N, TIM4_CH1
  GPIOD->AFR[1] |= 0x00220000;
  RCC->APB1ENR |= 0x00000004;    // enable TIM4 clock
}

void InitPwm(void) 
{
  PwmMaxCnt = ((unsigned int) (TIM_CLK/Fsw))>>1; // Carrier switching frequency 
  PwmMaxCnt = ((PwmMaxCnt >> 1) << 1) - 1; // Get ARR
  PwmHalfCnt = (PwmMaxCnt + 1) >> 1;
  ConfigPwm(PwmMaxCnt);
  PWM = TIM1->CCR1;                     
  tenpercent = (PwmMaxCnt + 1) * 0.1;   // ARR(MAX COUNT) always end with 9
}

void InitEnc(void)
{
  TIM4->PSC = 0;              
  TIM4->ARR = PPR;       
  TIM4->CNT = 0;        // clear counter
  TIM4->CCER &= 0xFFFFFFCC;       // non-inverting polarity , capture disable
  TIM4->CCMR1 = 0x00000101;       // CH1 -> TI1 , CH2 -> TI2 mapping
  TIM4->SMCR = 0x00000003;        // quadrature encoder interface
  TIM4->CR1 = 0x0005;         // counter enable
}

void ConfigPwm(unsigned int PwmMaxCnt) 
{
  TIM1->PSC = 0;        // 108MHz/(0+1) = 108MHz 
  TIM1->ARR = PwmMaxCnt;	// 108MHz / (10799+1) = 5KHz when center-aligned 
  TIM1->CCR1 = (PwmMaxCnt+1) >> 1; //initialize duty to 50%
  TIM1->CNT = 0;				// clear counter
  TIM1->CCMR1 = 0x00000060;			// OC1M = 0110 (PWM mode 1), CC1S = 00(output)
  TIM1->CCER = 0x00000005;			// CC1E,CC1NE = 1(enable OC1,OC1N output)
  TIM1->BDTR = 0x00008000;			// MOE = 1 
  TIM1->CR1 = 0x00F5;				// center-aligned,  enable preload buffer

  /* 
  Deadtime configuration
  TIM1->CR1 |= 0x0200;    // Tdts = 4*Tck_int
  TIM1->BDTR |= 0x000000C2; // DTG config
  */
}

int motor_direction()
{
  if(TIM1->CCR1 > PwmHalfCnt)
  {
    return FORWARD;
  }
  else if(TIM1->CCR1 < PwmHalfCnt)
  {
    return BACKWARD;
  }
  else if(TIM1->CCR1 == PwmHalfCnt)
  {
    return STOP;
  }
}

void change_direction(int DIR)
{
  if(DIR == FORWARD)
  {
    PWM = PwmHalfCnt + tenpercent * speedFactor;
    TIM1->CCR1 = PWM;
  }
  else if(DIR == BACKWARD)
  {
    PWM = PwmHalfCnt - tenpercent * speedFactor;
    TIM1->CCR1 = PWM;
  }
  else if(DIR == STOP)
  {
    PWM = PwmHalfCnt;
    TIM1->CCR1 = PWM;
  }
}

void rotate_motor(int DIR, int degree)
{
  int target_count = degree * PPD;
  int initial_count = read_encoder();

  change_direction(DIR);

  while(1)
  {
    int current_count = read_encoder();
    int delta_count;

    if(DIR == FORWARD)
    {
      delta_count = (current_count - initial_count + PPR) % PPR;
    }
    else
    {
      delta_count = (initial_count - current_count + PPR) % PPR;
    }

    if(delta_count >= target_count)
    {
      change_direction(STOP);
      break;
    }
  }
}


void align_motor(void)
{
  int direction = FORWARD;
  speedFactor = 2;

  for(int degree = 10; degree <= 360; degree += 10)
  {
    rotate_motor(direction, degree);
    Delay_ms(500);
    
    if(isMagnetOn())
    {
      reset_encoder();
      break;
    }

    direction = (direction == FORWARD) ? BACKWARD : FORWARD;
  }

  speedFactor = 1;
  Beep();
}

void reset_encoder()
{
  TIM4->CNT = 0;
}

unsigned int read_encoder()
{
  return TIM4->CNT;
}

void update_limit(unsigned int A, unsigned int B)
{
  firstLimit = A;
  secondLimit = B;
}

/*
------ADC------
*/

void InitADC1(void)
{
  GPIOA->MODER |= 0x0000C000;      // use ADC7
  RCC->APB2ENR |= 0x00000100;     // enable ADC1 clock
  ADC->CCR = 0x00000000;          // ADCCLK = 54MHz/2 = 27MHz
  ADC1->SMPR2 = 0x00200000;       // sampling time of channel 7 = 15 cycle
  ADC1->CR1 = 0x00800147;         // 12-bit resolution, AWD enable, AWD intterupt enable, scan mode
  setAWDTH(ADC_MAX,ADC_FIRE);
  ADC1->SQR1 = 0x00000000;        // total regular channel number = 1
  ADC1->SQR3 = 0x00000007;        // channel 7 (+5.0V)
  ADC1->CR2 = 0x00000003;         // right alignment, continuous conversion, ADON = 1
}


void setAWDTH(unsigned int HTR, unsigned int LTR)
{
  ADC1->HTR = HTR;    // higher threshold register
  ADC1->LTR = LTR;    // lower threshold register
}

/*
------GPIO------
*/

void InitGPIO(void) 
{
  GPIOC->MODER &= 0xFFFFFCFF;		// set PC4(LED) to output
  GPIOC->MODER |= 0x00000100;			
  GPIOC->ODR |= 0x00000010;			// PC4(LED) = 1 for Vcc
  
  GPIOE->MODER &= 0xC3FFFFFF;
  GPIOE->MODER |= 0x10000000;   // PE14(TP10) = output, PE13(TP9) = input
  GPIOE->ODR |= 0x00004000;     // PE14(TP10) = 1 for Magnet sensor
  GPIOE->PUPDR &= ~(0x03<<(13*2));
  GPIOE->PUPDR |= (0x02<<(13*2));   //PE13(TP9) internal pull-down
}

int isMagnetOn()
{
  return ((GPIOE->IDR & 0x2000) != 0) ? 1 : 0;
}

/*
--------Intterupt--------
*/

void InitINT(void)
{
  //TIM1->DIER = 0x0001;            // enable update interrupt
  //NVIC->ISER[0] |= 0x02000000;    // enable (25)TIM1 interrupt
  //TIM4->DIER = 0x0001;        
  //NVIC->ISER[0] |= 0x40000000;        

  SYSCFG->EXTICR[0] = 0x00002222;	// EXTI3,2,1,0 = PC3,2,1,0(KEY4,3,2,1)
  EXTI->IMR = 0x0000000F;			    // enable EXTI3,2,1,0 interrupt
  EXTI->RTSR = 0x00000000;			  // disable EXTI3,2,1,0 rising edge
  EXTI->FTSR = 0x0000000F;			  // enable EXTI3,2,1,0 falling edge
  NVIC->ISER[0] = 0x000003C0;			// enable (9)EXTI3 ~ (6)EXTI0 interrupt

  NVIC->ISER[0] |= 0x00040000;    // enable ADC1 interrupt enable 
}

