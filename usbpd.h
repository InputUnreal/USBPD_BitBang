#ifndef _USBPD_H
#define _USBPD_H

#include "main.h"   //needed for #include "stm32fxxx_hal.h"

#define SOP 0x18C71UL /*right aligned*/
#define EOP 0x16UL    /*right aligned*/

typedef struct
{
    uint16_t numOfObjects; /*a valid object should always store its object count*/
    uint16_t header;
    uint32_t dataObjects[9]; /*CRC is located after the last data object*/
} USBPD_Message;

typedef struct
{
    uint8_t notConnected;            /*/*should not be accessed by user - 0 indicates connected, anything else indicates NOT connected*/
    uint8_t prescaler;               /*timer prescaler sets timer frequecny*/
    uint16_t reloadValue;            /*timer reload value deltaT = 1/(timerFreq/reloadValue), timeout = deltaT*8, interFrameGap = deltaT*16 */
    GPIO_TypeDef *GPIO_Port;         /*GPIO port for input and output pins*/
    uint8_t CC1SensePin;             /*pin number ex PX0 = 0*/
    uint8_t CC1OuptutPin;            /*pin number ex PX1 = 1*/
    uint8_t CC2SensePin;             /*pin number ex PX2 = 2*/
    uint8_t CC2OuptutPin;            /*pin number ex PX3 = 3*/
    uint16_t outputPin;              /*should not be accessed by user*/
    uint16_t messageID;              /*should not be accessed by user- keeps track of the message number (range 0-7)*/
    TIM_TypeDef *TIMER;              /*timer for capture compare*/
    DMA_Channel_TypeDef *DMAChannel; /*DMA channel used for output*/
    uint32_t DMA_mem;                /*should not be accessed by user*/
    uint32_t buffer[16];             /*should not be accessed by user (used for input and output)*/
    USBPD_Message outputMessage;     /*should not be accessed by user*/
    USBPD_Message inputMessage;      /*should not be accessed by user*/
} USBPD_InitTypeDef;

typedef enum
{ /*this should be enough*/
  GoodCRC = 0,
  RequestMessage = 1
} MessageType;

/**** Initialization ****/
void USBPD_Init(USBPD_InitTypeDef *usbpd); /*currently only user accessible function*/

/*** Handler ****/
void USBPD_Handler(USBPD_InitTypeDef *usbpd); /*called by the interrupt*/

#endif