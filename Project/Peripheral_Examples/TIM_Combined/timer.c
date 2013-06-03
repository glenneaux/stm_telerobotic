#include "timer.h"

extern void TIM_Config(void)
{
  /* TIM1 Configuration ---------------------------------------------------
   Generate 3 combined PWM signals:
   TIM1 input clock (TIM1CLK) is set to APB2 clock (PCLK2)    
    => TIM1CLK = PCLK2 = SystemCoreClock
   TIM1CLK = SystemCoreClock, Prescaler = 0, TIM1 counter clock = SystemCoreClock
   SystemCoreClock is set to 72 MHz for STM32F30x devices
   
   The objective is to generate 3 combined PWM signal at 8.78 KHz (in center aligned mode):
     - TIM1_Period = (SystemCoreClock / (8.78*2)) - 1
   The channel 1  duty cycle is set to 50%
   The channel 2  duty cycle is set to 37.5%
   The channel 3  duty cycle is set to 25%
   The Timer pulse is calculated as follows:
     - ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100

   The channel 5  is used in PWM2 mode with duty cycle set to 6.22%

   The 3 resulting signals are made of an AND logical combination of two reference PWMs:
    - Channel 1 and Channel 5
    - Channel 2 and Channel 5
    - Channel 3 and Channel 5

   Note: 
    SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f30x.c file.
    Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
    function to update SystemCoreClock variable value. Otherwise, any configuration
    based on this variable will be incorrect. 
  ----------------------------------------------------------------------- */
  /* Compute the value to be set in ARR regiter to generate signal frequency at 17.57 Khz */
  //TimerPeriod = (SystemCoreClock /  ) - 1;
  /* Compute CCR1 value to generate a duty cycle at 50% for channel 1 */
  //Channel1Pulse = (uint16_t) (((uint32_t) 7.5 * (TimerPeriod - 1)) / 10);
  /* Compute CCR2 value to generate a duty cycle at 37.5%  for channel 2 */
  //Channel2Pulse = (uint16_t) (((uint32_t) 375 * (TimerPeriod - 1)) / 1000);
	
	
	//config outputs.
  GPIO_InitTypeDef GPIO_InitStructure;
	
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;

  /* GPIOA, GPIOB and GPIOE Clocks enable */
  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
  
  /* GPIOA Configuration: Channel 1, 2, 3 and 4 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_6);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_6);
	
	
	/* TIM1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
  
  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 23;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 62609;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* Channel 1, 2,3 and 4 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1;


  TIM_OC1Init(TIM1, &TIM_OCInitStructure);

	// channel 2
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);



  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);

  /* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
  
}
