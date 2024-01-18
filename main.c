/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbpd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

static USBPD_InitTypeDef usbpd;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void TIM3_IRQHandler(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    /* USER CODE BEGIN 2 */

    usbpd.prescaler = 0UL;
    usbpd.reloadValue = 88UL; /*determines output frequency*/
    usbpd.GPIO_Port = GPIOB;
    usbpd.CC1SensePin = 4UL;
    usbpd.CC1OuptutPin = 6UL;
    usbpd.CC2SensePin = 5UL;
    usbpd.CC2OuptutPin = 7UL;
    usbpd.TIMER = TIM3;
    usbpd.DMAChannel = DMA1_Channel3;

    USBPD_Init(&usbpd);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /*PA5 output*/
    GPIOA->BSRR = (0x1UL << 20); /*reset*/
    GPIOA->MODER &= ~(0x3UL << 8);
    GPIOA->MODER |= (0x1UL << 8);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        HAL_Delay(1);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/*start of TIM3_IRQHandler*/
void TIM3_IRQHandler(void)  /*entry point of the application*/
{                              
    uint8_t bufferIndex = 0;   /*buffer index only 30bits are used after the preamble so 6 5b4b encodes per 32bits*/
    uint8_t bufferCounter = 0; /*counter to determine number of bits*/
    uint16_t onescounter = 1;  /*determines if the last bit was a 1*/

    /*start of detection phase*/
    if (usbpd.notConnected)
    {
        /*to modify the input for CCR1 the channel_1 needs to be disabled additionally also disable channel 2 since it is no longer used*/
        usbpd.TIMER->CCER &= ~((0x1UL << 4) | 0x1UL);

        if (usbpd.TIMER->SR & 0x2UL)
        {                                                                                                   /*capture on channel_1*/
            usbpd.TIMER->CCMR1 = (usbpd.TIMER->CCMR1 & ~(0x3UL)) | 0x1UL;                                   /*channel_1 TI1 input*/
            usbpd.TIMER->SMCR = (usbpd.TIMER->SMCR & ~(0x7UL << 4)) | (0x5UL << 4); /*trigger mode TI1FP1*/ /*SMS must equal 000*/
            usbpd.outputPin = usbpd.CC1OuptutPin;                                                           /*set the output pin*/
        }
        else
        {                                                                                                   /*capture on channel 2*/
            usbpd.TIMER->CCMR1 = (usbpd.TIMER->CCMR1 & ~(0x3UL)) | 0x2UL;                                   /*channel_1 TI2 input*/
            usbpd.TIMER->SMCR = (usbpd.TIMER->SMCR & ~(0x7UL << 4)) | (0x6UL << 4); /*trigger mode TI2FP2*/ /*SMS must equal 000*/
            usbpd.outputPin = usbpd.CC2OuptutPin;                                                           /*set the output pin*/
        }

        usbpd.TIMER->SMCR |= 0x4UL;                            /*slave mode controller reset mode*/
        usbpd.TIMER->CCMR1 &= ~((0xFUL << 4) | (0xFUL << 12)); /*disable the input capture filter for both channels*/
        usbpd.TIMER->CCER |= 0x1UL;                            /*re-enable capture on channel_1 */
        usbpd.TIMER->SR = 0;                                   /*reset the SR register*/
        usbpd.notConnected = 0;                                /*usb cable now connected*/
        return;                                                /*exit so handler doesn't get called*/
    }
    /*end of detection phase*/

    /*start of capture loop*/
    /*if counter overflow, either the end of message transmit or cable has been unplugged*/
    usbpd.TIMER->DIER = 0; /*disable all interrupts*/
    usbpd.TIMER->SR = 0;   /*reset the SR register so the first capture is disregarded */

    while (~(usbpd.TIMER->SR | ~0x1UL))
    { /*stop on counter overflow*/
        if (usbpd.TIMER->SR & 0x2UL)
        { /*capture occured*/
            if (bufferCounter > ((bufferIndex >> 1) ? 29 : 31))
            {
                bufferIndex++;
                bufferCounter = 0;
            }
            if (usbpd.TIMER->CCR1 > 120)
            { /*capture was a zero*/
                usbpd.buffer[bufferIndex] = usbpd.buffer[bufferIndex] << 1;
                bufferCounter++;
            }
            else
            { /*capture was a one*/
                usbpd.buffer[bufferIndex] = (usbpd.buffer[bufferIndex] << (onescounter & 0x1UL)) | (onescounter & 0x1UL);
                bufferCounter += (onescounter & 0x1UL);
                onescounter++;
            }
        }
    }
    /*end of caputer loop*/

    usbpd.TIMER->SMCR &= ~(0x7UL);                                                 /*disable slave mode (prevent reset of counter)???*/
    usbpd.buffer[bufferIndex] = usbpd.buffer[bufferIndex] << (30 - bufferCounter); /*left align - 2 bits*/

    if (bufferIndex)
    {                                     /*buffer is > 0*/
        USBPD_Handler(&usbpd);            /*call handler*/
        usbpd.TIMER->SMCR |= 0x4UL;       /*set the slave mode controller to reset mode*/
        usbpd.TIMER->SR = 0;              /*reset the SR register*/
        usbpd.TIMER->DIER = (0x1UL << 1); /*enable interrupt on capture/compare_1*/
    }
    else
    {                                                                             /*cable unplugged*/
        usbpd.TIMER->CCER = 0;                                                    /*needs to 0 so CCxS is writable*/
        usbpd.TIMER->CCMR1 = (0x1UL) | (0xFUL << 4) | (1UL << 8) | (0xFUL << 12); /*set IC1->TI1, set IC2->TI2, IC1F and IC2F to max*/
        usbpd.TIMER->CCER = (0x1UL) | (0x5UL << 1) | (0x1UL << 4) | (0x5UL << 5); /*capture on both edges enable capture channel 1 and 2*/
        usbpd.TIMER->SMCR = 0;
        usbpd.TIMER->SR = 0; /*reset the SR register*/
        usbpd.TIMER->DIER = (1UL << 1) | (1UL << 2);
        usbpd.notConnected = 1UL;
    }
}
/*end of TIM3_IRQHandler*/

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM2 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM2)
    {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */