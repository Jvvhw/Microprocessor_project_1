# Microprocessor_project_1
 학부 3학년 "마이크로 프로세서 응용" 프로젝트 

<!-- ABOUT THE PROJECT -->
<h2 id="about-the-project"> :pencil: About The Project</h2>

<p align="justify"> 

 - STM32F767 마이크로 컨트롤러를 활용한 무인 화재 감지 시스템 개발

 - Advanced Timer 기능을 활용한 PWM 기반 모터 제어, 엔코더 활용 기능

 - 무인 감시에 초점을 맞춘 ADC 기능 지원. Scan mode, Analog-watchdog, Continuous conversion

 - 엔코더 위치 절대적 기준값으로 GPIO와 Magnet sensor 사용, Internal pull down 적용

 - AWD 인터럽트, EXTI 인터럽트로 시스템 효율 개선

</p>

<h2 id="project-files-description"> :floppy_disk: Files Description</h2>

<ul>
  <li><b>MIPROJECT_MAIN.c</b> - main</li>
  <li><b>miproject7.h</b> - 어플리케이션에 필요한 모든 함수와 변수 선언</li>
  <li><b>miproject7.c</b> - 프로젝트 오리지널 함수 구현부</li>
  <li><b>variable.h</b> - 코드 가독성을 높이기 위한 상수 정의</li>
</ul>

## Code Description
- [Main](#main)
- [Original Function](#original-function)
- [Register Coniguration](#register-coniguration)

## Main
```c
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

```
레지스터 설정 함수들을 모두 호출하여 시스템의 기본 설정을 완료

ADC 연속변환 시작하고 초기 범위를 0~180도로 설정, 모터를 멈춤

key 1번을 눌러 align으로 위치정렬이 필요한 시점

while문으로 들어가게 되면 기본적인 동작이 시작

모터가 정방향일때 우측 제한값을 넘어가면 방향이 역방향으로

모터가 역방향일때 좌측 제한값을 넘어가면 방향이 정방향으로

이 좌우측 제한값은 업데이트 플래그가 뜨면 업데이트

그리고 소화모드로 운전모드를 전환

아래 인터럽트 참고

```c
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
```

불꽃이 감지되면 인터럽트 서비스 루틴에 진입

불꽃을 처음으로 감지한곳에서 엔코더(새로운 좌측 혹은 우측 제한값) 값을 저장

그 후 같은 방향에서는 인터럽트가 발생해도 아무 일도 하지 않음

방향을 바꿔서 오다가 불꽃을 다시 감시하면 엔코더값(새로운 반대쪽 제한값)을 저장

그 후 업데이트 플래그를 1로 set

소화모드에 돌입하면 불꽃을 계속 보고있을것이므로 다음 사용을 위해 변수를 초기화

이외에는 아무것도 하지 않음 (water pump 부재)

```c
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
```
키 1번~4번 까지 인터럽트 서비스 루틴

1번: motor align (original funtion 부분 참고)

2번: start (현재 제한값으로 운전을 시작)

3번: stop

4번: 초기화 (소화모드에서 감시모드로 돌아오고, 감시범위가 다시 180도로 변경)

## Original Function
```c
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
```

현재 모터의 회전 방향을 반환합니다

```c
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

```

방향을 입력받아서 모터의 회전 방향을 바꿉니다

50% 듀티 기준으로 (멈춤)

10%에 speed factor값을 곱한 값이 가감되어 duty 변경

60% (정방향)

40% (역방향)

```c
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

```
방향과 각도를 입력받아서 모터를 지정한 방향으로 각도만큼 회전시킵니다

1도당 펄스 PPD를 기준으로 각도를 계산하여 지정한 펄스를 넘을때 멈춤

```c
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
```

모터가 즉시 앞으로 10도 뒤로 20도 앞으로 30도... 반복하며 자석센서를 찾습니다

센서가 발견되면 엔코더를 초기화하고 멈춥니다.

정렬시에는 회전 속도가 기존 운전보다 빨라집니다 (change direction 함수 참고)

```c
void reset_encoder()
{
  TIM4->CNT = 0;
}
```
엔코더의 카운트값을 초기화합니다

```c
unsigned int read_encoder()
{
  return TIM4->CNT;
}
```
엔코더의 현재 카운트값을 반환합니다

```c
void update_limit(unsigned int A, unsigned int B)
{
  firstLimit = A;
  secondLimit = B;
}
```
모터 운전시 양쪽 제한값을 업데이트 합니다. (메인함수 참고)

```c
int isMagnetOn()
{
  return ((GPIOE->IDR & 0x2000) != 0) ? 1 : 0;
}
```
자석 센서가 on이면 1을 반환합니다. 그 외에는 0을 반환합니다

## Register Coniguration
Timer:

```c
void InitTIM1(void) 
{
  GPIOE->MODER &= 0xFFF0FFFF;   // PE8,9 = alternate function mode
  GPIOE->MODER |= 0x000A0000;
  GPIOE->AFR[1] &= 0xFFFFFF00;  // PE8,9 = TIM1_CH1N, TIM1_CH1
  GPIOE->AFR[1] |= 0x00000011;
  RCC->APB2ENR |= 0x00000001;   // enable TIM1 clock
}
```
상보적인 2개 출력을 지원하는 고성능 타이머로 TIM1을 사용

사용 전 MODER를 통해 포트를 alternative mode로 설정

AFR을 통해 여러 기능들중 타이머를 선택

APB2ENR에서 클락을 공급

```c
void InitPwm(void) 
{
  PwmMaxCnt = ((unsigned int) (TIM_CLK/Fsw))>>1;  // Carrier switching frequency 
  PwmMaxCnt = ((PwmMaxCnt >> 1) << 1) - 1;  // Get ARR
  PwmHalfCnt = (PwmMaxCnt + 1) >> 1;
  ConfigPwm(PwmMaxCnt);
  PWM = TIM1->CCR1;                     
  tenpercent = (PwmMaxCnt + 1) * 0.1;   // ARR(MAX COUNT) 10%
}
```
타이머 클락 = 108Mhz 그대로 사용 (분주 x)

ARR (max값)을 구하기 위해 타이머 클락을 캐리어 주파수로 나누고, 

그 후 >>1 로 2로 다시 나눠준다. (오른쪽으로 1비트 이동하면 2진수에서는 2로 나눠짐)

구한 ARR값으로 PWM 설정함수를 호출

```c
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
}
```
PSC = 0 분주 안함

ARR에 매개변수로 받은 max값 입력

CCR(듀티)에 max값의 절반 입력

CCMR1로 PWM 모드 설정, output 설정

CCER에서 output 활성화

BDTR에서 MOE 메인 아웃풋 활성화

CR1에서 삼각파 설정, 프리로드 버퍼 활성화

```c
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
```
엔코더로 사용하는 TIM4의 설정

분주 안함

ARR = PPR ( 한 바퀴에 나오는 총 펄스를 max로 설정)

SMCR에서 4체배 설정 (펄스의 rising, falling edge를 모두 카운트)

ADC:

```c
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
```
아날로그 와치독 사용, 스캔 모드, 연속 변환 설정

스캔모드는 시퀸스의 채널을 순서대로 변환하고 다시 처음으로 돌아가서 시작

채널은 7번 채널 단 하나 사용 (계속 변환) 

```c
void setAWDTH(unsigned int HTR, unsigned int LTR)
{
  ADC1->HTR = HTR;    // higher threshold register
  ADC1->LTR = LTR;    // lower threshold register
}
```
아날로그 와치독 범위 지정 (벗어나면 인터럽트)

불꽃 센서는 전압이 낮아지게 반응하므로 LTR (1300 define) 설정

GPIO:

```c
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
```
PC4, PE14 상시 1 출력

PE13은 자석센서 감지용으로 input 설정

floating 상태 방지를 위해 내부 풀다운 (40k) 설정

GPIO:
```c
void InitINT(void)
{
  SYSCFG->EXTICR[0] = 0x00002222;	// EXTI3,2,1,0 = PC3,2,1,0(KEY4,3,2,1)
  EXTI->IMR = 0x0000000F;			    // enable EXTI3,2,1,0 interrupt
  EXTI->RTSR = 0x00000000;			  // disable EXTI3,2,1,0 rising edge
  EXTI->FTSR = 0x0000000F;			  // enable EXTI3,2,1,0 falling edge
  NVIC->ISER[0] = 0x000003C0;			// enable (9)EXTI3 ~ (6)EXTI0 interrupt

  NVIC->ISER[0] |= 0x00040000;    // enable ADC1 interrupt enable 
}
```
키 입력 외부 인터럽트

ADC 인터럽트 활성화
