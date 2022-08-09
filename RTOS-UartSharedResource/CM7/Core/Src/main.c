/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

#define TX_BUFFER_SIZE 		20
#define TX_MAX_DELAY		0xffffffff
#define PERIODICITY_TASK1 	4
#define PERIODICITY_TASK2 	3
#define PERIODICITY_TASK3 	2
#define PERIODICITY_TASK4 	1

#define USE_SEMAPHORE		0
#define USE_MUTEX			1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart3;

/* Definitions for Task1 */
osThreadId_t Task1Handle;
const osThreadAttr_t Task1_attributes = {
		.name = "Task1",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task2 */
osThreadId_t Task2Handle;
const osThreadAttr_t Task2_attributes = {
		.name = "Task2",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for Task3 */
osThreadId_t Task3Handle;
const osThreadAttr_t Task3_attributes = {
		.name = "Task3",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for Task4 */
osThreadId_t Task4Handle;
const osThreadAttr_t Task4_attributes = {
		.name = "Task4",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityLow3,
};
/* Definitions for Mutex1 */
osMutexId_t Mutex1Handle;
const osMutexAttr_t Mutex1_attributes = {
		.name = "Mutex1"
};
/* Definitions for Semaphore1 */
osSemaphoreId_t Semaphore1Handle;
const osSemaphoreAttr_t Semaphore1_attributes = {
		.name = "Semaphore1"
};
/* USER CODE BEGIN PV */
char* RTOS_TxBufferu_ac = NULL;
char RTOS_TaskMsg1_ac[TX_BUFFER_SIZE] = "Hello from task 1  \n";
char RTOS_TaskMsg2_ac[TX_BUFFER_SIZE] = "Hello from task 2  \n";
char RTOS_TaskMsg3_ac[TX_BUFFER_SIZE] = "Hello from task 3  \n";
char RTOS_TaskMsg4_ac[TX_BUFFER_SIZE] = "Hello from task 4  \n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
void Task1Function(void *argument);
void Task2Function(void *argument);
void Task3Function(void *argument);
void Task4Function(void *argument);

/* USER CODE BEGIN PFP */
HAL_StatusTypeDef FEKI_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
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
	/* USER CODE BEGIN Boot_Mode_Sequence_0 */
	/* USER CODE END Boot_Mode_Sequence_0 */

	/* USER CODE BEGIN Boot_Mode_Sequence_1 */
	/* USER CODE END Boot_Mode_Sequence_1 */
	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();
	/* USER CODE BEGIN Boot_Mode_Sequence_2 */
	/* USER CODE END Boot_Mode_Sequence_2 */

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */
	//  FEKI_UART_Transmit(&huart3, (uint8_t*)RTOS_TaskMsg4_ac, TX_BUFFER_SIZE, TX_MAX_DELAY);
	//  while(1);
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();
	/* Create the mutex(es) */
	/* creation of Mutex1 */
	Mutex1Handle = osMutexNew(&Mutex1_attributes);

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* creation of Semaphore1 */
	Semaphore1Handle = osSemaphoreNew(1, 1, &Semaphore1_attributes);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of Task1 */
	Task1Handle = osThreadNew(Task1Function, NULL, &Task1_attributes);

	/* creation of Task2 */
	Task2Handle = osThreadNew(Task2Function, NULL, &Task2_attributes);

	/* creation of Task3 */
	Task3Handle = osThreadNew(Task3Function, NULL, &Task3_attributes);

	/* creation of Task4 */
	Task4Handle = osThreadNew(Task4Function, NULL, &Task4_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
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

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
			|RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 1200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
HAL_StatusTypeDef FEKI_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	const uint8_t  *pdata8bits;
	const uint16_t *pdata16bits;
	uint32_t tickstart;

	/* Check that a Tx process is not already ongoing */

	if ((pData == NULL) || (Size == 0U))
	{
		return  HAL_ERROR;
	}

	//__HAL_LOCK(huart);

	huart->ErrorCode = HAL_UART_ERROR_NONE;
	huart->gState = HAL_UART_STATE_BUSY_TX;

	/* Init tickstart for timeout management */
	tickstart = HAL_GetTick();

	huart->TxXferSize  = Size;
	huart->TxXferCount = Size;

	/* In case of 9bits/No Parity transfer, pData needs to be handled as a uint16_t pointer */
	if ((huart->Init.WordLength == UART_WORDLENGTH_9B) && (huart->Init.Parity == UART_PARITY_NONE))
	{
		pdata8bits  = NULL;
		pdata16bits = (const uint16_t *) pData;
	}
	else
	{
		pdata8bits  = pData;
		pdata16bits = NULL;
	}

	//__HAL_UNLOCK(huart);

	while (huart->TxXferCount > 0U)
	{
		if (UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_TXE, RESET, tickstart, Timeout) != HAL_OK)
		{
			return HAL_TIMEOUT;
		}
		if (pdata8bits == NULL)
		{
			huart->Instance->TDR = (uint16_t)(*pdata16bits & 0x01FFU);
			pdata16bits++;
		}
		else
		{
			huart->Instance->TDR = (uint8_t)(*pdata8bits & 0xFFU);
			pdata8bits++;
		}
		huart->TxXferCount--;
	}

	//    if (UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_TC, RESET, tickstart, Timeout) != HAL_OK)
	//    {
	//      return HAL_TIMEOUT;
	//    }

	/* At end of Tx process, restore huart->gState to Ready */
	//huart->gState = HAL_UART_STATE_READY;

	return HAL_OK;

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Task1Function */
/**
 * @brief  Function implementing the Task1 thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task1Function */
void Task1Function(void *argument)
{
	/* USER CODE BEGIN 5 */
	uint8_t occurence1 = 0;
	/* Infinite loop */
	for(;;)
	{
		osDelay(PERIODICITY_TASK1);
		occurence1 ++ ;
#if USE_SEMAPHORE
		if (osOK == osSemaphoreAcquire(Semaphore1Handle, portMAX_DELAY))
#endif
#if USE_MUTEX
			if (osOK == osMutexAcquire(Mutex1Handle, portMAX_DELAY))
#endif
			{
				RTOS_TaskMsg1_ac[18] = (char)(occurence1 + 48);
				FEKI_UART_Transmit(&huart3, (uint8_t*)RTOS_TaskMsg1_ac, TX_BUFFER_SIZE, TX_MAX_DELAY);
#if USE_SEMAPHORE
				osSemaphoreRelease(Semaphore1Handle);
#endif

#if USE_MUTEX
				osMutexRelease(Mutex1Handle);
#endif
			}
			else
			{
				/* do nothing */
			}
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Task2Function */
/**
 * @brief Function implementing the Task2 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task2Function */
void Task2Function(void *argument)
{
	/* USER CODE BEGIN Task2Function */
	uint8_t occurence2 = 0;
	/* Infinite loop */
	for(;;)
	{
		osDelay(PERIODICITY_TASK2);
		occurence2 ++ ;
#if USE_SEMAPHORE
		if (osOK == osSemaphoreAcquire(Semaphore1Handle, portMAX_DELAY))
#endif
#if USE_MUTEX
			if (osOK == osMutexAcquire(Mutex1Handle, portMAX_DELAY))
#endif
			{
				RTOS_TaskMsg2_ac[18] = (char)(occurence2 + 48);
				FEKI_UART_Transmit(&huart3, (uint8_t*)RTOS_TaskMsg2_ac, TX_BUFFER_SIZE, TX_MAX_DELAY);
#if USE_SEMAPHORE
				osSemaphoreRelease(Semaphore1Handle);
#endif

#if USE_MUTEX
				osMutexRelease(Mutex1Handle);
#endif
			}
			else
			{
				/* do nothing */
			}

	}
	/* USER CODE END Task2Function */
}

/* USER CODE BEGIN Header_Task3Function */
/**
 * @brief Function implementing the Task3 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task3Function */
void Task3Function(void *argument)
{
	/* USER CODE BEGIN Task3Function */
	uint8_t occurence3 = 0;
	/* Infinite loop */
	for(;;)
	{
		osDelay(PERIODICITY_TASK3);
		occurence3 ++ ;
#if USE_SEMAPHORE
		if (osOK == osSemaphoreAcquire(Semaphore1Handle, portMAX_DELAY))
#endif
#if USE_MUTEX
			if (osOK == osMutexAcquire(Mutex1Handle, portMAX_DELAY))
#endif
			{
				RTOS_TaskMsg3_ac[18] = (char)(occurence3 + 48);
				FEKI_UART_Transmit(&huart3, (uint8_t*)RTOS_TaskMsg3_ac, TX_BUFFER_SIZE, TX_MAX_DELAY);
#if USE_SEMAPHORE
				osSemaphoreRelease(Semaphore1Handle);
#endif

#if USE_MUTEX
				osMutexRelease(Mutex1Handle);
#endif
			}
			else
			{
				/* do nothing */
			}

	}
	/* USER CODE END Task3Function */
}

/* USER CODE BEGIN Header_Task4Function */
/**
 * @brief Function implementing the Task4 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task4Function */
void Task4Function(void *argument)
{
	/* USER CODE BEGIN Task4Function */
	uint8_t occurence4 = 0;
	/* Infinite loop */
	for(;;)
	{
		osDelay(PERIODICITY_TASK4);
		occurence4 ++ ;
#if USE_SEMAPHORE
		if (osOK == osSemaphoreAcquire(Semaphore1Handle, portMAX_DELAY))
#endif
#if USE_MUTEX
			if (osOK == osMutexAcquire(Mutex1Handle, portMAX_DELAY))
#endif
			{
				RTOS_TaskMsg4_ac[18] = (char)(occurence4 + 48);
				FEKI_UART_Transmit(&huart3, (uint8_t*)RTOS_TaskMsg4_ac, TX_BUFFER_SIZE, TX_MAX_DELAY);
#if USE_SEMAPHORE
				osSemaphoreRelease(Semaphore1Handle);
#endif

#if USE_MUTEX
				osMutexRelease(Mutex1Handle);
#endif
			}
			else
			{
				/* do nothing */
			}
	}
	/* USER CODE END Task4Function */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
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

#ifdef  USE_FULL_ASSERT
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
