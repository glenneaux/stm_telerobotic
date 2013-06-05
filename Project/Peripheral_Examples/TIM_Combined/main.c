// Code originally based on ST Micro example code.


/*  SERIAL PROTOCOL
Serial settings - 8 bit, 57600 BAUD, No parity, 1 stop bit, 1 start bit


A three byte packet including - Header byte, Tilt Angle, Roll Angle

The header byte will be 255 so that value should be reserved

Tilt and Roll angles will have the format


MSB (bit 7) = sign bit where '0' denotes negative and '1' denotes positive

Bits 6-0 represent the angles of the range -15 degrees to + 15 degrees for each Roll and Tilt


The max decimal number we represent with bits 6-0 is 120

So for example a packet 11111111 (Header) 01111000 (Tilt angle of -(15*120/120)= -15 degrees) 10111100 (Roll angle of + (15*60/120) = +7.5 
degrees)
*/


#include "main.h"
#include "sensor.h"
#include "timer.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#define PI                         (float)     3.14159265f
uint8_t rx_buffer[RX_BUFFER_LENGTH];
uint8_t rx_counter = 0, rxBuf;
uint8_t rxState = 0;


uint16_t TimerPeriod = 0;
uint16_t Channel1Pulse = 0, Channel2Pulse = 0, Channel3Pulse = 0, Channel5Pulse = 0;
int remoteStale = 1;
float AccBuffer[3] = {0.0f};
	int valNeg = 0;
	uint8_t temp;
struct outputSettings //Values in microseconds (uS)
{
	int neutral;
	int min;
	int max;
	int current;
};

struct outputSettings outputChannels[2];


float fNormAcc,fSinRoll,fCosRoll,fSinPitch,fCosPitch = 0.0f, RollAng = 0.0f, PitchAng = 0.0f, writeRoll, writePitch;
int externalRollAng = 0, externalPitchAng = 0;

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
  	USART2_Init();
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
					PitchAng=clampVal(PitchAng);
			RollAng=clampVal(RollAng);
		writeRoll = RollAng;
		writePitch = PitchAng;
		
		// check is remote data is stale. if it's NOT stale add it to computer angled
		//if(remoteStale == 0)
		//{
    PitchAng+=(float)externalPitchAng;
		RollAng+=(float)externalRollAng;
		//}
		
		// Clamp values to +/- maximum angle (MAX_ANGLE)
		/*if(PitchAng>MAX_ANGLE)
			PitchAng=MAX_ANGLE;
		if(PitchAng<-MAX_ANGLE)
			PitchAng=-MAX_ANGLE;
		if(RollAng>MAX_ANGLE)
			RollAng=MAX_ANGLE;
		if(RollAng<-MAX_ANGLE)
			RollAng=-MAX_ANGLE;*/
			PitchAng=clampVal(PitchAng);
			RollAng=clampVal(RollAng);
			

		//TODO Output reading to serial port
		// sending will block the main program which is acceptable because we have lots of time.
		
		
		//Calculate desired channel outpus in uS reading according to servo range/output requirements
		updateChan(PitchAng, 0);
		updateChan(RollAng, 1);
		writeSerial();
		//Set Servo timer output pulse widths
		TIM_SetCompare1(TIM1, (int)(outputChannels[0].current*3.13));
		TIM_SetCompare2(TIM1, (int)(outputChannels[1].current*3.13));
		
		// Possibly implement delay? Should run this loop on an interrupt, maybe in the future.
		// Timer is run at 100hz

		//int i=0;
		//for (i=1 ; i<600000 ; i++) ;
		//for (i=1 ; i<600000 ; i++) ;

  }
}
void writeSerial(void)
{
	// send header, tilt, roll.
	// header is 0xff or 255.
	// Find Abs of Angle.
			// abs(RollAng)
	// Scale angle to 0-1 based on MAX_ANGLE
	// Multiple Scaled Angle by 120, casting at (char)
	// Set negative bit
	// Output;
	char Pitchoutput, Rolloutput;
	int i=0;
	Pitchoutput = (char)((sqrt(writePitch*writePitch)/MAX_ANGLE)*120);
	//output = (char)abs(writeRoll);
 	if (writePitch < 0)
		Pitchoutput |= 1 << 7;
	
	Rolloutput = (char)((sqrt(writeRoll*writeRoll)/MAX_ANGLE)*120);
	//output = (char)abs(writeRoll);
 	if (writeRoll < 0)
		Rolloutput |= 1 << 7;
		
	//write packet header
	// Need to find out how to wait for tx to happen.
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
	USART_SendData(USART2, 255);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
	USART_SendData(USART2, Pitchoutput);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
	USART_SendData(USART2, Rolloutput);

	
	
	
//USART_SendData(USART2, 'X');
	
	//while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
		
	//USART_SendData(USART2, 'A');
	
	//while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
    return;
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
void USART2_Init(void)
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

    //if(inten)
    //{
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
    //}

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
				
			//rx_buffer[rx_counter] = (USART_ReceiveData(USART2));
			rxBuf = USART_ReceiveData(USART2);
			//USART_SendData(USART2, rxBuf);
			
			if(rxBuf == 0xFF)
			{
				// got header
				rxState = 1;
				rx_counter = 0;
			}else{
				// if rxState = 0, do nothing.
				if(rxState == 1)
				{
					// rxState = 0. We have header. 
					// Write byte, increment counter.
					rx_buffer[rx_counter] = rxBuf;
					rx_counter++;
					
					
				}
			}
			//check if rx_counter is 1. If it is process the 2 bytes rx_buffer[0] and rx_buffer[1].
			//set rx_state -1.
			if(rx_counter==2)
			{
				// have data process 2 bytes.
				processRx(); 
				
				//clear rx_state
				rxState = 0;
			}
			
			// check rx state.
			// -1 means no header
			// 0 means got header
			// 1 means got first byte
			// 2 means got 2nd byte
			/*
        if(rx_counter + 1 == RX_BUFFER_LENGTH ||
                rx_buffer[rx_counter] == '\n' || rx_buffer[rx_counter] == '\r')
        {
            //printf("%s\n\r", rx_buffer);
            //memset(rx_buffer, 0, RX_BUFFER_LENGTH);
            rx_counter = 0;
					rxState = -1;
        }
        else if(rx_buffer[rx_counter] == 0xFF)
				{
					// got header
					//USART_SendData(USART2, 'X');
					
					rxState = 0;
					rx_counter++;
				}
				
				else
        {
					if(rxState >= 0)
					{
						rxState++;
					}
          if(rxState == 2)
					{
						processRx(); 
						rxState = -1;
					}						
					
					rx_counter++;
        }*/
    }
}
float clampVal(float val)
		{
			if(val>MAX_ANGLE)
			val=MAX_ANGLE;
			if(val<-MAX_ANGLE)
			val=-MAX_ANGLE;
			
			return val;
		}
void processRx(void)
{

	//So for example a packet 11111111 (Header) 01111000 (Tilt angle of -(15*120/120)= -15 degrees) 10111100 (Roll angle of + (15*60/120) = +7.5 
	//Check pos/neg bit.
	//Clear bit, translate 120number into angle, set sign.
	valNeg = !!(rx_buffer[0] & (1 << 7));

	if(valNeg == 1){
		valNeg = -1;
	}else{
		valNeg =1;
	}
	
	temp = rx_buffer[0]; //import tilt angle
	temp &= ~(1 << 7); //clear bit
	//temp is now 0-120 (should be)
	externalPitchAng = (float)((MAX_ANGLE*temp)/120)*valNeg;
	
	valNeg = !!(rx_buffer[1] & (1 << 7));
	if(valNeg == 1){
		valNeg = -1;
	}else{
		valNeg =1;
	}
	
	temp = rx_buffer[1]; //import roll angle
	temp &= ~(1 << 7); //clear bit
	//temp is now 0-120 (should be)
	externalRollAng = (float)((MAX_ANGLE*temp)/120)*valNeg;
	return;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
