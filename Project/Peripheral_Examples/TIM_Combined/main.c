// Code originally based on ST Micro example code.

#include "main.h"
#include "sensor.h"
#include "timer.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#define PI                         (float)     3.14159265f
uint8_t rx_buffer[RX_BUFFER_LENGTH];
uint8_t rx_counter = 0;


uint16_t TimerPeriod = 0;
uint16_t Channel1Pulse = 0, Channel2Pulse = 0, Channel3Pulse = 0, Channel5Pulse = 0;

float AccBuffer[3] = {0.0f};

struct outputSettings //Values in microseconds (uS)
{
	int neutral;
	int min;
	int max;
	int current;
};

struct outputSettings outputChannels[2];


float fNormAcc,fSinRoll,fCosRoll,fSinPitch,fCosPitch = 0.0f, RollAng = 0.0f, PitchAng = 0.0f;

int main(void)
{
	int i=0;
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f30x.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f30x.c file
     */ 
	outputChannels[0].neutral = 1500;
	outputChannels[0].current = outputChannels[0].neutral;
	outputChannels[0].min = 1000;
	outputChannels[0].max = 2000;
	outputChannels[1].neutral = 1500;
	outputChannels[1].current = outputChannels[1].neutral;
	outputChannels[1].min = 1000;
	outputChannels[1].max = 2000;

   /* TIM Configuration */
  TIM_Config();
  	
	Demo_CompassConfig();

  /* Infinite loop */
	//Read Buffer
	//Convert reading to angle. Clip according to +/- 15deg
	//Output reading to serial port
	//Scale reading according to servo range/output requirements
	//Set Servo positions
	// Possibly implement delay? Should run this loop on an interrupt, maybe in the future.
	
	

	//Set servo output
	
  while (1)
  {
		// Read accelerometer buffer
		Demo_CompassReadAcc(AccBuffer);
      
		//values are 16bit 2s complement
		
		// Convert reading to angle.
      for(i=0;i<3;i++)
        AccBuffer[i] /= 100.0f;
		
		RollAng = (atan2(-AccBuffer[1], AccBuffer[2])*180.0)/PI;
		PitchAng =  (atan2(AccBuffer[0], sqrt(AccBuffer[1]*AccBuffer[1] + AccBuffer[2]*AccBuffer[2]))*180.0)/PI;
    
		if(PitchAng>MAX_ANGLE)
			PitchAng=MAX_ANGLE;
		if(PitchAng<-MAX_ANGLE)
			PitchAng=-MAX_ANGLE;
		if(RollAng>MAX_ANGLE)
			RollAng=MAX_ANGLE;
		if(RollAng<-MAX_ANGLE)
			RollAng=-MAX_ANGLE;
		
		//TODO Output reading to serial port
		
		
		//Calculate desired channel outpus in uS reading according to servo range/output requirements
		updateChan(PitchAng, 0);
		updateChan(RollAng, 1);
		
		//Set Servo timer output pulse widths
		TIM_SetCompare1(TIM1, (int)(outputChannels[0].current*3.13));
		TIM_SetCompare2(TIM1, (int)(outputChannels[1].current*3.13));
		
		// Possibly implement delay? Should run this loop on an interrupt, maybe in the future.
		


  }
}


void updateChan(float angle, int chan)
{
		angle = angle / MAX_ANGLE;
		if(angle > 0)
		{
			outputChannels[chan].current = outputChannels[chan].neutral + angle * (outputChannels[chan].max - outputChannels[chan].neutral);
		}else if(angle < 0)
		{
			outputChannels[chan].current = outputChannels[chan].neutral + angle * (outputChannels[chan].neutral - outputChannels[chan].min);
		}else{
			/// Pitch ang = 0deg
			outputChannels[chan].current = outputChannels[chan].neutral;
		}
		return;
}

/**
  * @brief  Configure the TIM IRQ Handler.
  * @param  None
  * @retval None
  */

#ifdef  USE_FULL_ASSERT
/*
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t LSM303DLHC_TIMEOUT_UserCallback(void)
{
  return 0;
}
/**
  * @}
  */ 

/**
 * @brief  This function initializes USART2 module
 * @param  speed: baudrate
 *          inten: enable interrupt
 * @retval None
 */
void USART2_Init(uint32_t speed, uint8_t inten)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable GPIO clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    /* Enable USART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* Connect PXx to USARTx_Tx */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_7);

    /* Connect PXx to USARTx_Rx */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_7);

    /* Configure USART Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure USART Rx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART configuration */
    USART_InitStructure.USART_BaudRate = 57600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    if(inten)
    {
        /* Enable the USART2 Receive interrupt: this interrupt is generated when the
                     USART2 receive data register is not empty */
        USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
        //USART_ITConfig(USART2, USART_IT_TXNE, ENABLE);

        /* Enable the USART2 Interrupt */
        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }

    /* Enable USART */
    USART_Cmd(USART2, ENABLE);
}


/**
 * @brief  This function handles USART2 global interrupt request.
 * @param  None
 * @retval None
 */
void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        /* Read one byte from the receive data register */
        rx_buffer[rx_counter] = (USART_ReceiveData(USART2) & 0x7F);

        if(rx_counter + 1 == RX_BUFFER_LENGTH ||
                rx_buffer[rx_counter] == '\n' || rx_buffer[rx_counter] == '\r')
        {
            printf("%s\n\r", rx_buffer);
            memset(rx_buffer, 0, RX_BUFFER_LENGTH);
            rx_counter = 0;
        }
        else
        {
            rx_counter++;
        }
    }
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
