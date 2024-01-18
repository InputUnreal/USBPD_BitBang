#include "usbpd.h"

//waits for update event
#define WAITFORUEV while(~(usbpd->TIMER->SR | ~0x1UL)){}

/*start of encode4b5b arr*/
static const uint8_t encode4b5b[16] = {
    /*inverted MSB is LSB*/
    0x78UL, /*0*/
    0x90UL, /*1*/
    0x28UL, /*2*/
    0xa8UL, /*3*/
    0x50UL, /*4*/
    0xd0UL, /*5*/
    0x70UL, /*6*/
    0xf0UL, /*7*/
    0x48UL, /*8*/
    0xc8UL, /*9*/
    0x68UL, /*a*/
    0xe8UL, /*b*/
    0x58UL, /*c*/
    0xd8UL, /*d*/
    0x38UL, /*e*/
    0xb8UL  /*f*/
};
/*end of encode4b5b arr*/

/*start of decode5b4b arr*/
static const uint8_t decode5b4b_0[16] = {
    /*inverted MSB is LSB*/
    0x00, /*0-NULL*/
    0x00, /*1-NULL*/
    0x00, /*2-NULL*/
    0x00, /*3-NULL*/
    0x00, /*4-NULL*/
    0x02, /*5-0x2*/
    0x00, /*6-NULL*/
    0x0E, /*7-0xE*/
    0x00, /*8-NULL*/
    0x08, /*9-NULL*/
    0x04, /*10-0x4*/
    0x0C, /*11-0xC*/
    0x00, /*12-NULL*/
    0x0A, /*13-0xA*/
    0x06, /*14-0x6*/
    0x00  /*15-0x0*/
};

static const uint8_t decode5b4b_1[16] = {
    /*inverted MSB is LSB*/
    0x00, /*0-NULL*/
    0x00, /*1-NULL*/
    0x01, /*2-0x1*/
    0x00, /*3-NULL*/
    0x00, /*4-NULL*/
    0x03, /*5-0x3*/
    0x00, /*6-NULL*/
    0x0f, /*7-0xf*/
    0x00, /*8-NULL*/
    0x09, /*9-0x9*/
    0x05, /*10-0x5*/
    0x0d, /*11-0xd*/
    0x00, /*12-NULL*/
    0x0b, /*13-0xb*/
    0x07, /*14-0x7*/
    0x00  /*15-NULL*/
};
/*end of decode5b4 arr*/

/*start of setDelay*/
static void setDelay(USBPD_InitTypeDef *usbpd, uint16_t reloadValue)
{
    usbpd->TIMER->ARR = reloadValue; /*load new reload value*/
    usbpd->TIMER->EGR = 1UL;         /*update shadow registers*/
    usbpd->TIMER->SR = 0;
}
/*end of setDelay*/

/*start of decode - CRC check should be here*/
static uint8_t decode(USBPD_InitTypeDef *usbpd)
{
    uint8_t *pointer = ((uint8_t *)&usbpd->inputMessage) + 2;
    uint8_t offset = 2;
    uint8_t lower;
    uint16_t upper;
    uint8_t index = 2; /*buffer index*/

    while (index < 15)
    {
        upper = (uint16_t)((usbpd->buffer[index] & (0x3FF00000UL >> (10 * offset))) >> (10 * (2 - offset)));
        lower = upper & 0x1FUL;
        upper = upper >> 5;

        if (lower == EOP || upper == EOP)
        {
            usbpd->inputMessage.numOfObjects = (uint16_t)(((index * 3) - 14UL + offset) >> 2); /*set the num of dataObjects*/
            return 1UL;                                                                        /*EOP reached return 1 "success"*/
        }

        *pointer = (uint8_t)(((lower & 0x10UL) ? decode5b4b_1[lower & 0xFUL] : decode5b4b_0[lower & 0xFUL]) << 4); /*bits 7:4*/
        *pointer |= (upper & 0x10UL) ? decode5b4b_1[upper & 0xFUL] : decode5b4b_0[upper & 0xFUL];                  /*bits 3:0*/
        pointer++;

        if (++offset > 2)
        {
            offset = 0;
            index++;
        }
    }
    /*CRC check should go here*/
    return 0UL; /*decode failed return 0 "failed"*/
}
/*end of decode*/

/*start of calculateCRC*/
static uint32_t calculateCRC(USBPD_Message *message)
{
    uint8_t i;
    CRC->CR |= 1UL; /*reset the CRC module*/

    for (i = 0; i < ((message->numOfObjects << 2) + 2); i++)
    {
        /*write BYTES to the CRC module*/
        *(__IO uint8_t *)(__IO void *)(&CRC->DR) = *(((uint8_t *)&message->header) + i);
    }

    return (CRC->DR ^ 0xFFFFFFFFUL);
}
/*end of calculateCRC*/

/*start of blender - frame termination should be here*/
static uint16_t blender(USBPD_InitTypeDef *usbpd)
{
    /*NOTE right align might be better*/
    uint8_t i;
    uint8_t temp;
    uint8_t bufferIndex = 2; /*bufferIndex is in size of 4 bytes*/
    uint8_t bufferPos = 2;

    usbpd->buffer[0] = 0x55555555UL;                                                                           /*append preamble*/
    usbpd->buffer[1] = 0x55555555UL;                                                                           /*append preamble*/
    usbpd->buffer[bufferIndex] = SOP;                                                                          /*append SOP*/
    usbpd->outputMessage.dataObjects[usbpd->outputMessage.numOfObjects] = calculateCRC(&usbpd->outputMessage); /*calculate the CRC*/

    for (i = 0; i < ((usbpd->outputMessage.numOfObjects << 2) + 6); i++)
    { /*i is in bytes of message*/
        temp = *(((uint8_t *)&usbpd->outputMessage.header) + i);
        /*make space and append 2 encodes to outputbufffer*/
        usbpd->buffer[bufferIndex] = (usbpd->buffer[bufferIndex] << 10) | (uint32_t)(encode4b5b[(temp) & 0xFUL] << 2) | (uint32_t)(encode4b5b[(temp & 0xF0UL) >> 4] >> 3);
        if (++bufferPos > 2)
        {                                                                 /*increment then check*/
            bufferPos = 0;                                                /*reset the bufferPos*/
            usbpd->buffer[bufferIndex] = usbpd->buffer[bufferIndex] << 2; /*left aligned*/
            bufferIndex++;                                                /*increment the index*/
        }
    }
    /*refer to figure5-18-21 pg126-127 for frame termination*/
    usbpd->buffer[bufferIndex] = ((usbpd->buffer[bufferIndex] << 5) | EOP) << (((3 - bufferPos) * 10) - 3);                /*add EOP and left align*/
    return (uint16_t)((usbpd->outputMessage.numOfObjects * 80) + 298); /*return the number of clock cycles in the buffer*/ /*298 = (PREAMBLE + SOP + HEADER + CRC + EOP)x2*/
}
/*end of blender*/

/*start of USBPD_sendData (implement tDrive properly)*/
static void USBPD_sendData(USBPD_InitTypeDef *usbpd)
{
    uint8_t outputBufferIndex = 0UL;
    uint8_t dataMaskCounter = 0UL;
    uint16_t clockCounter = blender(usbpd); /*blend and CRC*/

    /*configure hardware*/
    usbpd->TIMER->ARR = usbpd->reloadValue; /*load new reload value*/
    usbpd->TIMER->EGR = 1UL;                /*update shadow registers*/
    usbpd->TIMER->DIER = 1UL << 8;          /*DMA transfer on overflow*/
    usbpd->DMAChannel->CCR |= 1UL;          /*enable the DMA channel*/

    usbpd->GPIO_Port->BSRR = (1UL << usbpd->outputPin) << 16; /*reset output pins since message always start with "0"*/

    /*start of transmitter*/
    while (clockCounter)
    {
        if (DMA1->ISR)
        {                                                                                                                                                                              /*check the dma flags*/
            usbpd->GPIO_Port->MODER |= (0x1UL << (usbpd->outputPin << 1)); /*configure output pins as output*/                                                                         /*fix this*/
            usbpd->DMA_mem = usbpd->GPIO_Port->ODR ^ ((uint32_t)((usbpd->buffer[outputBufferIndex] & (0x80000000UL >> dataMaskCounter)) || (clockCounter & 1UL)) << usbpd->outputPin); /*toggle the CC pin*/
            if ((dataMaskCounter += (clockCounter & 1UL)) > (outputBufferIndex >> 1 ? 29 : 31))
            {
                dataMaskCounter = 0UL;
                outputBufferIndex++;
            }
            clockCounter--;
            DMA1->IFCR = 0x0FFFFFFFUL; /*clear the dma flags*/
        }
    }
    /*end of transmitter*/

    /*temporary - refer to figure5-18-21 pg126-127*/
    if ((usbpd->GPIO_Port->ODR & (1UL << usbpd->outputPin)) == 0)
    {
        usbpd->DMA_mem = usbpd->GPIO_Port->ODR | (1UL << usbpd->outputPin); /*high*/
        while (1)
        {
            if (DMA1->ISR)
            {
                DMA1->IFCR = 0x0FFFFFFFUL; /*clear the dma flags*/
                break;
            }
        }
        while (1)
        {
            if (DMA1->ISR)
            {
                DMA1->IFCR = 0x0FFFFFFFUL; /*clear the dma flags*/
                break;
            }
        }
        usbpd->DMA_mem = usbpd->GPIO_Port->ODR & ~(1UL << usbpd->outputPin); /*low*/
        while (1)
        {
            if (DMA1->ISR)
            {
                DMA1->IFCR = 0x0FFFFFFFUL; /*clear the dma flags*/
                break;
            }
        }
        while (1)
        {
            if (DMA1->ISR)
            {
                DMA1->IFCR = 0x0FFFFFFFUL; /*clear the dma flags*/
                break;
            }
        }
    }
    else
    {
        while (1)
        {
            if (DMA1->ISR)
            {
                DMA1->IFCR = 0x0FFFFFFFUL; /*clear the dma flags*/
                break;
            }
        }
        while (1)
        {
            if (DMA1->ISR)
            {
                DMA1->IFCR = 0x0FFFFFFFUL; /*clear the dma flags*/
                break;
            }
        }
    }
    /*fix me pls*/

    usbpd->GPIO_Port->MODER &= ~(0x3UL << (usbpd->outputPin << 1)); /*configure output pin as input(HiZ)*/
    usbpd->DMAChannel->CCR &= ~(1UL);                               /*disable the DMA channel*/
}
/*end of USBPD_sendData*/

/*start of formatDataObject*/
static void formatDataObject(USBPD_InitTypeDef *usbpd)
{
    usbpd->outputMessage.dataObjects[0] = (uint32_t)(usbpd->inputMessage.numOfObjects << 28) |
                                          ((usbpd->inputMessage.dataObjects[usbpd->inputMessage.numOfObjects - 1] & 0x3FF) << 10) |
                                          (usbpd->inputMessage.dataObjects[usbpd->inputMessage.numOfObjects - 1] & 0x3FF);
}
/*end of formatDataObject*/

/* MessageHeader
bit 15 -> Reserved set to 0b
bit 14-12 -> # of Data Objects (000b control message)
bit 11-9 -> Message ID
bit 8 -> Port Power Role (0b sink)
bit 7-6 -> Specification Revision (00b REV1) (01b REV2) using REV2
bit 5 -> Port Data Role (0b UFP) (1b DFP) SINK->(UFP)
bit 4 -> Reserved set to 0b
bit 3-0 -> Message Type (Control Messages and Data Messages have different codes)
Control Message -> (GoodCRC 0001b)
Data Message -> (Request 0010b) (Source_Capabilities 0001b)
*/
/*start of formatHeader*/
static void formatHeader(USBPD_InitTypeDef *usbpd, MessageType type)
{
    if (type == GoodCRC)
    {
        usbpd->outputMessage.header = (uint16_t)((usbpd->inputMessage.header & 0xE00UL) | 0x41UL); /*get the message ID from inputMessage*/
        usbpd->outputMessage.numOfObjects = 0;
    }
    else
    {                                                                               /*Request Message*/
        usbpd->outputMessage.header = (uint16_t)(usbpd->messageID << 9) | 0x1042UL; /*one data object*/
        usbpd->outputMessage.numOfObjects = 1;
        usbpd->messageID = (usbpd->messageID > 6U) ? 0 : usbpd->messageID + 1UL;
        formatDataObject(usbpd);
    }
}
/*end of formatHeader*/

/*start of USBPD_Init*/
void USBPD_Init(USBPD_InitTypeDef *usbpd)
{
    /*start of GPIO section*/
    __HAL_RCC_GPIOB_CLK_ENABLE(); /*this needs to be port specific*/

    /*BSRR (reset all pins)*/
    usbpd->GPIO_Port->BSRR = ((1UL << usbpd->CC1OuptutPin) | (1UL << usbpd->CC2OuptutPin) | (1UL << usbpd->CC1SensePin) | (1UL << usbpd->CC2SensePin)) << 16;

    /*OSPEEDR (low speed for every pin)*/
    usbpd->GPIO_Port->OSPEEDR &= ~((0x3UL << (usbpd->CC1OuptutPin << 1)) | (0x3UL << (usbpd->CC2OuptutPin << 1)) |
                                   (0x3UL << (usbpd->CC1SensePin << 1)) | (0x3UL << (usbpd->CC2SensePin << 1)));

    /*OTYPER (push-pull for every pin)*/
    usbpd->GPIO_Port->OTYPER &= ~((1UL << usbpd->CC1OuptutPin) | (1UL << usbpd->CC2OuptutPin) | (1UL << usbpd->CC1SensePin) | (1UL << usbpd->CC2SensePin));

    /*PUPDR (no pullup/pulldown for all used pins)*/
    usbpd->GPIO_Port->PUPDR &= ~((0x3UL << (usbpd->CC1OuptutPin << 1)) | (0x3UL << (usbpd->CC2OuptutPin << 1)) |
                                 (0x3UL << (usbpd->CC1SensePin << 1)) | (0x3UL << (usbpd->CC2SensePin << 1)));

    /*AFR (sense pins AF1)*/
    usbpd->GPIO_Port->AFR[0] &= ~((0xFUL << (usbpd->CC1SensePin << 2)) | (0xFUL << (usbpd->CC2SensePin << 2)));
    usbpd->GPIO_Port->AFR[0] |= (0x1UL << (usbpd->CC1SensePin << 2)) | (0x1UL << (usbpd->CC2SensePin << 2));

    /*MODER*/
    usbpd->GPIO_Port->MODER &= ~((0x3UL << (usbpd->CC1OuptutPin << 1)) | (0x3UL << (usbpd->CC2OuptutPin << 1)) |
                                 (0x3UL << (usbpd->CC1SensePin << 1)) | (0x3UL << (usbpd->CC2SensePin << 1)));

    usbpd->GPIO_Port->MODER &= ~((0x3UL << (usbpd->CC1OuptutPin << 1)) | (0x3UL << (usbpd->CC2OuptutPin << 1))); /*configure output pins as input(HiZ)*/
    usbpd->GPIO_Port->MODER |= (0x2UL << (usbpd->CC1SensePin << 1)) | (0x2UL << (usbpd->CC2SensePin << 1));      /*configure sense pins as AF*/
    /*end of GPIO section*/

    /*start of timer section*/
    __HAL_RCC_TIM3_CLK_ENABLE(); /*this needs to be timer specific*/

    /*enable auto-reload preload enable, set URS so UEV is only generated when over/underflow*/
    usbpd->TIMER->CR1 = (1UL << 7) | (1UL << 2);
    usbpd->TIMER->PSC = usbpd->prescaler;                    /*set the prescaler*/
    usbpd->TIMER->ARR = (uint16_t)(usbpd->reloadValue << 4); /*set the auto-reload-value*/
    usbpd->TIMER->EGR = 1UL;                                 /*generate an UEV*/

    usbpd->TIMER->CCMR1 = (0x1UL) | (0xFUL << 4) | (1UL << 8) | (0xFUL << 12); /*set IC1->TI1, set IC2->TI2, IC1F and IC2F to max */
    usbpd->TIMER->CCER = (0x1UL) | (0x5UL << 1) | (0x1UL << 4) | (0x5UL << 5); /*capture on both edges enable capture channel 1 and 2*/
    usbpd->TIMER->DIER = (1UL << 1) | (1UL << 2);                              /*enable interrupt for CC1 and CC2*/
    usbpd->TIMER->CR1 |= 1UL;                                                  /*enable the counter*/
    /*end of timer section*/

    /*start of DMA section*/
    __HAL_RCC_DMA1_CLK_ENABLE(); /*DMA clock enable*/

    usbpd->DMAChannel->CPAR = (uint32_t)&usbpd->GPIO_Port->ODR; /*set the peripheral address*/
    usbpd->DMAChannel->CMAR = (uint32_t)&usbpd->DMA_mem;        /*set the memory address*/
    usbpd->DMAChannel->CNDTR = 1UL;                             /*set the number of data to be transfered*/

    /*very high priority, MSIZE and PSIZE 32bit, CIRC mode, data direction read memory*/
    usbpd->DMAChannel->CCR = (3UL << 12) | (2UL << 10) | (2UL << 8) | (1UL << 5) | (1UL << 4);
    /*DMA is still disabled*/
    /*end of DMA section*/

    /*start of CRC section*/
    __HAL_RCC_CRC_CLK_ENABLE();         /*CRC clock enable*/
    CRC->CR |= (1UL << 5) | (1UL << 7); /*set input/output direction */
    /*end of CRC section*/

    /*start of module config*/
    usbpd->notConnected = 1UL;
    usbpd->messageID = 0;
    /*end of module config*/

    /*NVIC ENABLE*/
    HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}
/*end of USBPD_Init*/

/*start of handler*/ /*this gets called from the TIMER interrupt*/
void USBPD_Handler(USBPD_InitTypeDef *usbpd)
{
    setDelay(usbpd, (uint16_t)(usbpd->reloadValue << 5)); /*interframeGap not perfect*/

    if (((usbpd->buffer[2] & 0x3FFFFC00UL) >> 10) == SOP)
    { /*message is SOP, preamble check ???*/
        if (decode(usbpd))
        { /*decode is valid*/
            if ((usbpd->inputMessage.header & 0x700FUL) != 0x1UL)
            { /*message isnt GoodCRC*/
                formatHeader(usbpd, GoodCRC);
                WAITFORUEV /*wait from frameGap*/
                    USBPD_sendData(usbpd);
                setDelay(usbpd, (uint16_t)(usbpd->reloadValue << 5)); /*interframeGap*/
                if ((0xF000UL & usbpd->inputMessage.header) != 0 && (0x000FUL & usbpd->inputMessage.header) == 0x1UL)
                { /*reply with request message*/
                    formatHeader(usbpd, RequestMessage);

                    WAITFORUEV /*wait from frameGap*/
                        USBPD_sendData(usbpd);
                }
            }
        }
    }

    setDelay(usbpd, (uint16_t)(usbpd->reloadValue << 4));
    WAITFORUEV /*wait so interrupt doesnt trigger when Hi-Z is set by Rp/Rd*/
               /*return to interrupt handler*/
}
/*end of handler*/
