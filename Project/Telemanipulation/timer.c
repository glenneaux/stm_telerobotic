#include "timer.h"

extern void TIM_Config(void)
{
	// Configure TIM1 module for PWM output.
	
	// Module configuration structures.
  GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

  // Enable GPIOA and GPIOB clocks.
	// GPIOA is for USART pins.
	// GPIOB is for servo outputs.
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
  
  // GPIO Configuration.
	// Pin 8 and 9 are for PWM outputs.
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	// Alternate Function.
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // Output type Push Pull
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ; // Pull up Pull down
  GPIO_Init(GPIOA, &GPIO_InitStructure); // Initialize GPIOA

	// Configure PIN Alternate functions.
	// GPIO Alternate Functions for Pins 8 and 9 are for Timer2 (TIM2) functions.
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_6);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_6);
	
	// Enable the clock for TIM2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	// Configure TIM2 interupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	// Initialize TIM2 interrupt.
	NVIC_Init(&NVIC_InitStructure);
	
	// Configure TIM1
	// Enable TIM1 Interrupt
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
  
	// Configure TIM1 module for PWM output.
	// I am aiming for 100hz update, ~20ms.
	// Clock runs at 72MHz
	// Prescale by 24-1 (23) ~3.13MHz
	// Pulse period 62610 cycles (~20ms)
  TIM_TimeBaseStructure.TIM_Prescaler = 23;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 62609;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	// Initialize Timer.
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	// Configure Timer modules
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1;

	// Initialize TIM1, Channel 1;
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	// Initialize TIM1, Channel 2;
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);

	// Configure TIM1
	TIM_Cmd(TIM1, ENABLE);

  // Enable TIM1 PWM outputs
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
  
	//set up TIM2 - .5second interrupts
	// Setup TIM2 for 0.5second interrupts for detecting stale data from remote controller.
	// Clock at 72mhz
	// Prescaler 42, 1.714MHz
	// Period 1000000 cycles.
	TIM_TimeBaseStructure.TIM_Period = 1000000-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 42;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	// Initialize TIM2 module.
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	// TIM2 interrupt overflow enable.
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
	//Enable TIM2 module.
	TIM_Cmd(TIM2, ENABLE);
}
