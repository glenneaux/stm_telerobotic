/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H


#include "stm32f30x.h"
#include "stm32f3_discovery.h"
#include "stm32f30x_it.h"
#include "stm32f3_discovery_lsm303dlhc.h"
#include <stdint.h>
#include "sensor.h"
#include "timer.h"
#include "math.h"

#define RX_BUFFER_LENGTH        (40) 
#define txHeader								0xFF

extern void USART2_Init(void);
extern void USART2_IRQHandler(void);
extern uint8_t USART2_ReadChar(void);
void writeSerial(void);
static float MAX_ANGLE=15.0f;
void updateChan(float angle, int chan);
void processRx(void);
float clampVal(float val);


#endif /* __MAIN_H */
