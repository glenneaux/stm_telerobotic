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



#define PI                         (float)     3.14159265f
uint8_t rx_buffer[RX_BUFFER_LENGTH];
uint8_t rx_counter = 0, rxBuf;
uint8_t rxState = 0;

int remoteStale = 1;
float AccBuffer[3] = {0.0f};
	int valNeg = 0;


// This structure holds all of the servo position settings in terms of a pulsewidth in uS
// There was no calibration routine for this, just constants that had to be set by trial and error.
// After the platform was constructed these values could values could have been measured using
// another controller and a CRO.
struct outputSettings //Values in microseconds (uS)
{
	int neutral; // The neutral position, this should hold that axis in the horizontal plane
	int min; // platform at -15degrees
	int max; // platform at +15degrees
	int current; // The current commanded position/pulsewidth.
};

struct outputSettings outputChannels[2];

// commandedAngle[0] = Tilt angle
// commandedAngle[1] = Roll Angle
float commandedAngle[2] = {0.0f,0.0f};

// Angles from the remote
int externalRollAng = 0, externalTiltAng = 0;
int i=0;

int main(void)
{
	 /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f30x.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f30x.c file
     */ 
	
	outputChannels[0].neutral = 1700;
	outputChannels[0].current = outputChannels[0].neutral;
	outputChannels[0].min = 1500;
	outputChannels[0].max = 1900;
	outputChannels[1].neutral = 1500;
	outputChannels[1].current = outputChannels[1].neutral;
	outputChannels[1].min = 1700;
	outputChannels[1].max = 1300;

	// Configure Accelerometer.
	ConfigureAccelerometer();

	// Configure Serial interface
	USART2_Init();
  
	// Configure Timers for PWM output and RX age timer.
  TIM_Config();
  
	
	
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
		ReadAcc(AccBuffer);
      
 		// Values are 16bit 2s complement
		// Convert reading to angle.
    for(i=0;i<3;i++)
			AccBuffer[i] /= 100.0f;
		
		commandedAngle[1] = (atan2(-AccBuffer[1], AccBuffer[2])*180.0)/PI;
		commandedAngle[0] =  (atan2(AccBuffer[0], sqrt(AccBuffer[1]*AccBuffer[1] + AccBuffer[2]*AccBuffer[2]))*180.0)/PI;
		
		// Clamp values to +/- Max Angle.
		commandedAngle[0]=clampVal(commandedAngle[0]);
		commandedAngle[1]=clampVal(commandedAngle[1]);
				
		// Write angles to output
		writeSerial();
		// check is remote data is stale. if it's NOT stale add it to computer angled
		if(remoteStale == 0)
		{
			// Add values from external controller to own angles.
			commandedAngle[0]=(float)externalTiltAng;
			commandedAngle[1]=(float)externalRollAng;
		}
		
		//Update output pulsewidths based on the desired angle.
		updateChan(commandedAngle[0], 0);
		updateChan(commandedAngle[1], 1);
		
		//Set Servo timer output pulse widths
		// outputChannels min/max/current/neutral values are stored in uS.
		// 3.13 is a constant to get the correct value for the timer system.
		TIM_SetCompare1(TIM1, (int)(outputChannels[0].current*3.13));
		TIM_SetCompare2(TIM1, (int)(outputChannels[1].current*3.13));
		
		// TODO - Most of the main code should be in an interrupt instead of running ASAP in main loop.

  }
}
void writeSerial(void)
{
	// send header, tilt, roll.
	// header is 0xff or 255.
	// Find Abs of Angle.
	// abs(commandedAngle[1]) (sqrt(commandedAngle[1]*commandedAngle[1]) - Some unresolved issue with abs() function, work around.
	// Normalize angle to 0-1 based on MAX_ANGLE
	// Multiple Scaled Angle by 120, casting as (char)
	// Set negative bit (set MSB (7) to 1. value |= 1 << 7;
	// Bang header and both values to output;
	char Tiltoutput, Rolloutput;

	// Had issues with abs() not working.
	// Using sqrt(val^2) instead for now.
	// Value is normalized then scaled to our range of possible output vales (120 in this case)
	Tiltoutput = (char)((sqrt(commandedAngle[0]*commandedAngle[0])/MAX_ANGLE)*120);
	
	// if value is negative (<0) then we must set Bit 7 of the output packet to 1 to signify as such.
 	if (commandedAngle[0] < 0)
		Tiltoutput |= 1 << 7;
	
	Rolloutput = (char)((sqrt(commandedAngle[1]*commandedAngle[1])/MAX_ANGLE)*120);

 	if (commandedAngle[1] < 0)
		Rolloutput |= 1 << 7;
	
	// When outputting the values this could be rewritten into a loop
	// ouput values could be stored in a buffer, or string.
	// write packet header
	// wait for TX to be ready
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
	USART_SendData(USART2, txHeader);
	
	// wait for tx to be ready, write Pitch value to UART
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
	USART_SendData(USART2, Tiltoutput);
	
	// wait for tx to be ready, write Roll value to UART.
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
	USART_SendData(USART2, Rolloutput);

  return;
}

void updateChan(float angle, int chan)
{
		// Update the servo channel pulse widths
		// This function is slightly complicated because I added a min and max servo angle.
		// In theory this is redundant due to the servo having even movement throughout the allowed range.
		
		// Normalize the angle.
		// The angle is clamped to MAX_ANGLE so this (should) never be > 1.
		angle = angle / MAX_ANGLE;
		if(angle > 0)
		{
			outputChannels[chan].current = outputChannels[chan].neutral + angle * (outputChannels[chan].max - outputChannels[chan].neutral);
		}else if(angle < 0)
		{
			outputChannels[chan].current = outputChannels[chan].neutral + angle * (outputChannels[chan].neutral - outputChannels[chan].min);
		}else{
			/// Pitch ang = 0deg
			// This is somewhat redudant as one of the above could be >= or <=.
			// Included only for simplicities sake.
			outputChannels[chan].current = outputChannels[chan].neutral;
		}
		return;
}


// Retained error handling from program template.
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

// Retained erorr handling from program template.
/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t LSM303DLHC_TIMEOUT_UserCallback(void)
{
  return 0;
}

// Timer 2 is used to detect is received data is stale
void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		// if data is > 0.5 second old we consider the remote tx disconnected.
		// When the module receives data it will clear the remoteStale data.
		// We set stale to 1 and let the receiving function clear it each time it receives.
		if(remoteStale == 0){
			remoteStale = 1;
		}
		
  }
}

// Initialize Serial module
void USART2_Init(void)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable GPIO clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	
    /* Enable USART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* Connect GPIO A, Pin 2 to USARTx_Tx (Alternat Function 7)*/
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_7);

    /* Connect GPIO A, Pin 3 to USARTx_Rx (Alternate Function 7)*/
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

		// Configure & Enable Interrupt.
		// For simplicities sake this could have been configured to interupt when TX is ready.
		// Given that the function only sends 3 bytes at a time I let this happen in the main loop.
		// The program sends data as fast as it can, if the update rate was fixed, it could trigger a data
		// transmission and let the interupt handle it.
		
    // Enable RX Not Empty interrupt. (Interrupt when data is received)
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    /* Enable the USART2 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Enable USART module
    USART_Cmd(USART2, ENABLE);
}


// IRQ handler for USART2 module
void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {		
			rxBuf = USART_ReceiveData(USART2);
			
			if(rxBuf == 0xFF)
			{
				// got header
				rxState = 1;
				// Reset buffer to position 0.
				rx_counter = 0;
			}else{
				// if rxState = 0, do nothing.
				if(rxState == 1)
				{
					// rxState = 1. We have header. 
					// Write byte from rx to buffer, increment counter.
					rx_buffer[rx_counter] = rxBuf;
					rx_counter++;
				}
			}
			//check if rx_counter is 2. If it is process the 2 bytes rx_buffer[0] and rx_buffer[1].
			//set rx_state 0.
			if(rx_counter==2)
			{
				// At this stage we should have received the header 0xff and 2 bytes, our pitch and roll angles.
				processRx(); 
				
				//clear rx_state ready for next data packet.
				rxState = 0;
			}
    }
		return;
}

// Clamp the values to our MAX_ANGLE.
float clampVal(float val)
{
	if(val>MAX_ANGLE)
		val=MAX_ANGLE;
	if(val<-MAX_ANGLE)
		val=-MAX_ANGLE;
	
	return val;
}

// After receiving the header and then 2 bytes.
// This function will process the values to our actual angle values.
void processRx(void)
{
	uint8_t temp;
	// we are processing received data so remote data is not stale any more
	remoteStale = 0;
	
	//Example packet from Ben's post. 
	//11111111 (Header) 
	//01111000 (Tilt angle of -(15*120/120)= -15 degrees) 
	//10111100 (Roll angle of + (15*60/120) = +7.5 
	
	//Check pos/neg bit.
	temp = rx_buffer[0]; //import tilt angle
	// Store MSB in valNeg
	valNeg = !!(temp & (1 << 7));

	// This value is used to multiply the value in bits 6-0
	// if bit 7 is 0, the value is postiive and the value in bits 6-0 should be multiplied by 1.
	// if bit 7 is 1, the value is negative and should be multiplied by -1;
	if(valNeg == 1){
		valNeg = -1;
	}else{
		valNeg = 1;
	}
	
	//Clear bit, translate 120number into angle, set sign.
	temp &= ~(1 << 7); //clear MSB bit
	//temp is now 0-120 (should be)
	// multiple value by our MAX_ANGLE and divide by 120 to scale back to angle value.
	// Multiple by 1 or -1 depending on if our original value had MSB bit 7 set to 1.
	externalTiltAng = (float)((MAX_ANGLE*temp)/120)*valNeg;
	
	temp = rx_buffer[1]; //import roll angle
	valNeg = !!(temp & (1 << 7));
	if(valNeg == 1){
		valNeg = -1;
	}else{
		valNeg = 1;
	}

	temp &= ~(1 << 7); //clear bit
	//temp is now 0-120 (should be)

	externalRollAng = (float)((MAX_ANGLE*temp)/120)*valNeg;
	return;
}
