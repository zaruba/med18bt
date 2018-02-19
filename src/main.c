/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "discovery.h"
#include "mpu.h"
//#include "lpms-me1.h"
#include "ahrs.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId bltTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define UART_PACKET_OK 0
#define UART_PACKET_TOO_LONG 1
#define UART_PACKET_CORRUPT 2
#define UART_BUSY_FLAG 1

volatile uint8_t mUartTxBusyFlag = 0;

#define UART_TX_MAXBUF_SZ 64
#define UART_RX_MAXBUF_SZ 64

char mUartRxString[UART_RX_MAXBUF_SZ];

uint8_t mUartRxBuffer = '\000';
uint8_t mUartNewMessage = 0;

char mUartTxbuffer[UART_TX_MAXBUF_SZ+1];
int mUartTxBufSz;
	

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	
	printf("DEBUG: Application started\r\n");

//	LPMS_ME1_Init( &hi2c1 );
	
	BSP_LED_Init(LED3);
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
	BSP_LED_Init(LED6);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of bltTask */
  osThreadDef(bltTask, StartDefaultTask, osPriorityNormal, 0, 128);
  bltTaskHandle = osThreadCreate(osThread(bltTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*** ================================================================================================ ***/
/*** ==== BLUETOOTH ================================================================================= ***/
/*** ================================================================================================ ***/

HAL_StatusTypeDef UART_WaitIfBusy() {
	for (int i = 0; mUartTxBusyFlag && i < 100; i++)
		HAL_Delay(1);
	
	if (mUartTxBusyFlag)
		return HAL_ERROR;
	
	mUartTxBusyFlag  = 1;
	
	return HAL_OK;
}

void UART_Send (const char message[])
{
	if (UART_WaitIfBusy() != HAL_OK)
		return;
		
	mUartTxBufSz = strlen(message);
	if (mUartTxBufSz > UART_TX_MAXBUF_SZ)
		mUartTxBufSz = UART_TX_MAXBUF_SZ-1;
		
	strncpy(mUartTxbuffer, message, mUartTxBufSz);

	if (strlen(message) > UART_TX_MAXBUF_SZ) {
		mUartTxbuffer[UART_TX_MAXBUF_SZ-2] = '\r';
		mUartTxbuffer[UART_TX_MAXBUF_SZ-1] = '\n';
		mUartTxbuffer[UART_TX_MAXBUF_SZ] = '\0';
	}
	
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)mUartTxbuffer, mUartTxBufSz);
}

void UART_SendChar( const int ch ) {
	if (UART_WaitIfBusy() != HAL_OK)
		return;
	
	mUartTxBufSz = 1;
	mUartTxbuffer[0] = (char) ch;
	
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)mUartTxbuffer, mUartTxBufSz);
}
	

void UART_CommandProcessor (void)
{
	printf("DEBUG: CMD received: '%s'\r\n", mUartRxString);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
//	for(int i = 0; i < 10; i++) {
//		BSP_LED_Toggle(LED3);
//		BSP_LED_Toggle(LED4);
//		BSP_LED_Toggle(LED5);
//		BSP_LED_Toggle(LED6);
//		HAL_Delay(100);
//	}
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart1)
{
	mUartTxBusyFlag  = 0;
	// BSP_LED_Toggle(LED4);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static short int tUartRxIndex = 0;	
	static uint8_t tUartErrorFlag = UART_PACKET_OK;
	
	// if(UartHandle->Instance==USART1){
//	BSP_LED_Toggle(LED4);
//	BSP_LED_Toggle(LED6);
	
	if (mUartRxBuffer == '\n') // If Enter
	{
		if (tUartRxIndex && tUartErrorFlag == UART_PACKET_OK)
		{
			mUartRxString[tUartRxIndex] = '\000';
			tUartRxIndex = 0;
			mUartNewMessage  = 1;
		
		} else {
			// UART_Send("ERROR: UART1 packet too long\r\n");
			tUartErrorFlag = UART_PACKET_OK; // reset error state
		}

	} else {

		if (mUartRxBuffer != '\r' && tUartErrorFlag == UART_PACKET_OK) // Ignore return
		{
			mUartRxString[tUartRxIndex] = mUartRxBuffer; // Add that character to the string
			tUartRxIndex++;
			
			if (tUartRxIndex >= UART_RX_MAXBUF_SZ) // User typing too much, we can't have commands that big
			{
				tUartErrorFlag = UART_PACKET_TOO_LONG;
				tUartRxIndex = 0;
				mUartRxString[tUartRxIndex] = '\000';
			}
		}	
	}

	HAL_UART_Receive_IT(&huart1, &mUartRxBuffer, 1);
}

/*** ================================================================================================ ***/
/*** ==== I2C ======================================================================================= ***/
/*** ================================================================================================ ***/

/********************************************************************************************************/

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
	float accel_data[3];
	float gyro_data[3];
	float magn_data[3];
	float temp_data;
	AHRS_t m_AHRS;

	uint8_t mpu_init_required = 1;
	uint32_t m_started_ts = 0, m_reporter_ts = 0, m_data_counter = 0;

	AHRS_Init(&m_AHRS, 40, 0.5, 1);

	HAL_UART_Receive_IT(&huart1, &mUartRxBuffer, 1);

  /* Infinite loop */
  for(;;)
  {
		BSP_LED_Toggle(LED3);
		
		if (mpu_init_required) {
			
			// hard_reset_required
			if (MPU_Init( &hi2c1, (mpu_init_required > 1 ? 1 : 0) ) != HAL_OK) {
				printf("ERROR: can't init MPU\r\n");
				HAL_Delay(500);
				
				mpu_init_required = 2; // hard reset required
				continue;
			}
			
			mpu_init_required = 0;
			m_started_ts = HAL_GetTick();
			m_reporter_ts = 0;
			m_data_counter = 0;
		}
	
		if(mUartNewMessage)	
		{
//			UART_CommandProcessor();
//			mUartNewMessage  = 0;
		}
		
/*		// reset FIFO
		if (MPU_ResetFifo( &hi2c1 ) != HAL_OK) {
			// MPU reset required
			mpu_init_required = 1;
			continue;
		}
*/

/*		
		if (MPU_GetFifoFrameData( &hi2c1, accel_data, gyro_data, magn_data, &temp_data ) != HAL_OK) {
			// MPU reset required
			mpu_init_required = 1;
			continue;
		}
*/
		if (MPU_GetData( &hi2c1, accel_data, gyro_data, magn_data, &temp_data ) != HAL_OK) {
			// MPU reset required
			mpu_init_required = 1;
			continue;
		}
		m_data_counter++;
/*		
		AHRS_UpdateIMU(&m_AHRS, 
			accel_data[0], accel_data[1], accel_data[2], 
			AHRS_DEG2RAD(gyro_data[0]), AHRS_DEG2RAD(gyro_data[1]), AHRS_DEG2RAD(gyro_data[2]));
		*/

		AHRS_UpdateAHRS(&m_AHRS, 
			accel_data[0], accel_data[1], accel_data[2], 
			AHRS_DEG2RAD(gyro_data[0]), AHRS_DEG2RAD(gyro_data[1]), AHRS_DEG2RAD(gyro_data[2]), 
			magn_data[0], magn_data[1], magn_data[2]);


		if (HAL_GetTick() - m_reporter_ts > 1000) {
			m_reporter_ts = HAL_GetTick();

			//printf("P=%0.02f R=%0.02f Y=%0.02f\r\n", m_AHRS.Pitch, m_AHRS.Roll, m_AHRS.Yaw);
			
			printf("A: %02.02f %02.02f %02.2f mg\r\n", accel_data[0], accel_data[1], accel_data[2]);
			printf("G: %02.02f %02.02f %02.2f deg/s\r\n", gyro_data[0], gyro_data[1], gyro_data[2]);
			printf("M: %0.02f %0.02f %0.2f mG\r\n", magn_data[0], magn_data[1], magn_data[2]);
			
			//float temp = MPU_get_temp( &hi2c1 );
			//printf("T %0.02f\r\n", temp_data);
			
			//printf("FREQ=%02.02f\r\n", (float)m_data_counter * 1000.0f / (HAL_GetTick() - m_started_ts)); 
		}
		
    osDelay(20);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
