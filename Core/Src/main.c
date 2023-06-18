/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 2
#define DISPADDR   0x3C << 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

osThreadId WatchdogHandle;
osThreadId ECUTaskHandle;
osThreadId InputTaskHandle;
osMessageQId ECUQueueHandle;
osSemaphoreId ECU_ACKHandle;
/* USER CODE BEGIN PV */
uint8_t dataBuffer[BUFFER_SIZE] = { 0x00, 0x00 };
uint8_t testBuffer[BUFFER_SIZE] = {0x01, 0x01};

struct InputEvent {
	uint8_t group;
	uint8_t button;
};

enum buttonGroups {
	noGroup = 0x00, JOY = 0x01, BTN = 0x02
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void StartWatchdog(void const * argument);
void StartECUTask(void const * argument);
void StartInputTask(void const * argument);

/* USER CODE BEGIN PFP */
void lcd_send_data(char data);
void lcd_send_cmd(char cmd);
void lcd_clear();
void lcd_string(char string[80]);
void lcd_init();
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
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of ECU_ACK */
  osSemaphoreDef(ECU_ACK);
  ECU_ACKHandle = osSemaphoreCreate(osSemaphore(ECU_ACK), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of ECUQueue */
  osMessageQDef(ECUQueue, 10, struct InputEvent);
  ECUQueueHandle = osMessageCreate(osMessageQ(ECUQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Watchdog */
  osThreadDef(Watchdog, StartWatchdog, osPriorityNormal, 0, 128);
  WatchdogHandle = osThreadCreate(osThread(Watchdog), NULL);

  /* definition and creation of ECUTask */
  osThreadDef(ECUTask, StartECUTask, osPriorityNormal, 0, 128);
  ECUTaskHandle = osThreadCreate(osThread(ECUTask), NULL);

  /* definition and creation of InputTask */
  osThreadDef(InputTask, StartInputTask, osPriorityNormal, 0, 128);
  InputTaskHandle = osThreadCreate(osThread(InputTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 64;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DEBUG_LED_Pin|DISP_RES_Pin|DISP_SA0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(INTERRUPT_LINE_GPIO_Port, INTERRUPT_LINE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : JOY_PRESS_Pin */
  GPIO_InitStruct.Pin = JOY_PRESS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(JOY_PRESS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_LEFT_DOWN_Pin BTN_LEFT_UP_Pin JOY_DOWN_Pin JOY_RIGHT_Pin
                           JOY_UP_Pin JOY_LEFT_Pin */
  GPIO_InitStruct.Pin = BTN_LEFT_DOWN_Pin|BTN_LEFT_UP_Pin|JOY_DOWN_Pin|JOY_RIGHT_Pin
                          |JOY_UP_Pin|JOY_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DEBUG_LED_Pin DISP_RES_Pin DISP_SA0_Pin */
  GPIO_InitStruct.Pin = DEBUG_LED_Pin|DISP_RES_Pin|DISP_SA0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : INTERRUPT_LINE_Pin */
  GPIO_InitStruct.Pin = INTERRUPT_LINE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(INTERRUPT_LINE_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void lcd_send_data(char data) {
	uint8_t data_t[] = { 0x40, data };
	HAL_StatusTypeDef status;
	status = HAL_I2C_Master_Transmit(&hi2c1, DISPADDR, data_t, sizeof(data_t),
	HAL_MAX_DELAY);
	if (status == HAL_OK) {
		return;
	} else {
		return;
		// error handling
	}
}

void lcd_send_cmd(char cmd) {
	uint8_t data_t[] = { 0x80, cmd };
	HAL_StatusTypeDef status;
	status = HAL_I2C_Master_Transmit(&hi2c1, DISPADDR, data_t, sizeof(data_t),
	HAL_MAX_DELAY);

	if (status == HAL_OK) {
		return;
	} else {
		// error handling
	}
}

void lcd_clear() {
	lcd_send_cmd(0x01); // clear display
	lcd_send_cmd(0x80); // set DDRAM address to 0x00}
}

void lcd_string(char string[80]) {
	uint8_t length = strlen(string);
	for (uint8_t i = 0; i < length; i++) {
		lcd_send_data(string[i]);
	}
}

void lcd_init() {
	// make sure that RES and SA0 pins are named correctly
	HAL_GPIO_WritePin(DISP_RES_GPIO_Port, DISP_RES_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DISP_SA0_GPIO_Port, DISP_SA0_Pin, GPIO_PIN_RESET);
#if 0
	lcd_send_cmd(0x2A);  // function set (extended command set)
	lcd_send_cmd(0x71);  // function selection A
	lcd_send_data(0x00); // disable internal VDD regulator (2.8V I/O). data(0x5C) = enable regulator (5V I/O)
	lcd_send_cmd(0x28);  // function set (fundamental command set)
	// not turning display off means it doesn't blink on re-init, so is silent error.
	// lcd_send_cmd(0x08); //display off, cursor off, blink off
	lcd_send_cmd(0x2A);  // function set (extended command set)
	lcd_send_cmd(0x79);  // OLED command set enabled
	lcd_send_cmd(0xD5);  // set display clock divide ratio/oscillator frequency
	lcd_send_cmd(0x70);  // set display clock divide ratio/oscillator frequency
	lcd_send_cmd(0x78);  // OLED command set disabled
	lcd_send_cmd(0x09);  // extended function set (4-lines)
	lcd_send_cmd(0x06);  // COM SEG direction
	lcd_send_cmd(0x72);  // function selection B
	lcd_send_data(0x00); // ROM CGRAM selection
	lcd_send_cmd(0x2A);  // function set (extended command set)
	lcd_send_cmd(0x79);  // OLED command set enabled
	lcd_send_cmd(0xDA);  // set SEG pins hardware configuration
	lcd_send_cmd(0x10);  // set SEG pins hardware configuration
	lcd_send_cmd(0xDC);  // function selection C
	lcd_send_cmd(0x00);  // function selection C
	lcd_send_cmd(0x81);  // set contrast control
	lcd_send_cmd(0x7F);  // set contrast control
	lcd_send_cmd(0xD9);  // set phase length
	lcd_send_cmd(0xF1);  // set phase length
	lcd_send_cmd(0xDB);  // set VCOMH deselect level
	lcd_send_cmd(0x40);  // set VCOMH deselect level
	lcd_send_cmd(0x78);  // OLED command set disabled
	lcd_send_cmd(0x28);  // function set (fundamental command set)
	// don't clear display, w're writing the whole thing anyway
	lcd_send_cmd(0x01); // clear display
	lcd_send_cmd(0x80); // set DDRAM address to 0x00
	lcd_send_cmd(0x0C); // display ON
#endif
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	HAL_GPIO_WritePin(INTERRUPT_LINE_GPIO_Port, INTERRUPT_LINE_Pin,
				GPIO_PIN_RESET);

	xSemaphoreGiveFromISR(ECU_ACKHandle, &xHigherPriorityTaskWoken);

	HAL_I2C_Slave_Transmit_IT(&hi2c1, dataBuffer, BUFFER_SIZE);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


struct InputEvent checkJoystick() {
	struct InputEvent e;
	if (HAL_GPIO_ReadPin(JOY_UP_GPIO_Port, JOY_UP_Pin) == GPIO_PIN_RESET) {
		while (HAL_GPIO_ReadPin(JOY_UP_GPIO_Port, JOY_UP_Pin) == GPIO_PIN_RESET) {
			vTaskDelay(30);
		}
		e.button = 0x01;
		e.group = JOY;
	} else if (HAL_GPIO_ReadPin(JOY_RIGHT_GPIO_Port, JOY_RIGHT_Pin)
			== GPIO_PIN_RESET) {
		while (HAL_GPIO_ReadPin(JOY_RIGHT_GPIO_Port, JOY_RIGHT_Pin)
				== GPIO_PIN_RESET) {
			vTaskDelay(30);
		}
		e.button = 0x02;
		e.group = JOY;
	} else if (HAL_GPIO_ReadPin(JOY_DOWN_GPIO_Port, JOY_DOWN_Pin)
			== GPIO_PIN_RESET) {
		while (HAL_GPIO_ReadPin(JOY_DOWN_GPIO_Port, JOY_DOWN_Pin)
				== GPIO_PIN_RESET) {
			vTaskDelay(30);
		}
		e.button = 0x03;
		e.group = JOY;
	} else if (HAL_GPIO_ReadPin(JOY_LEFT_GPIO_Port, BTN_LEFT_UP_Pin)
			== GPIO_PIN_RESET) {
		while (HAL_GPIO_ReadPin(JOY_LEFT_GPIO_Port, BTN_LEFT_UP_Pin)
				== GPIO_PIN_RESET) {
			vTaskDelay(30);
		}
		e.button = 0x04;
		e.group = JOY;
	} else if (HAL_GPIO_ReadPin(JOY_PRESS_GPIO_Port, JOY_PRESS_Pin)
			== GPIO_PIN_RESET) {

		while (HAL_GPIO_ReadPin(JOY_PRESS_GPIO_Port, JOY_PRESS_Pin)
				== GPIO_PIN_RESET) {
			vTaskDelay(30);
		}
		e.button = 0x05;
		e.group = JOY;
	} else {
		e.button = 0x00;
		e.group = noGroup;
	}
	return e;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartWatchdog */
/**
 * @brief  Function implementing the Watchdog thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartWatchdog */
void StartWatchdog(void const * argument)
{
  /* USER CODE BEGIN 5 */
	lcd_init();
	UBaseType_t queueLength = 0;

	for (;;) {
		HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
		queueLength = uxQueueMessagesWaiting(ECUQueueHandle);
		if (queueLength > 0) {
			osDelay(200);
		} else {
			osDelay(1000);
		}
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartECUTask */
/**
 * @brief Function implementing the ECUTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartECUTask */
void StartECUTask(void const * argument)
{
  /* USER CODE BEGIN StartECUTask */
	struct InputEvent e;
	BaseType_t ecuACK;
	HAL_I2C_Slave_Transmit_IT(&hi2c1, dataBuffer, BUFFER_SIZE); // Start listening for requests
	/* Infinite loop */
	for (;;) {

		if (xQueueReceive(ECUQueueHandle, &(e), (TickType_t) 10) == pdPASS) {
			dataBuffer[0] = e.group;
			dataBuffer[1] = e.button;

			HAL_GPIO_WritePin(INTERRUPT_LINE_GPIO_Port, INTERRUPT_LINE_Pin,
					GPIO_PIN_SET);

			ecuACK = xSemaphoreTake(ECU_ACKHandle, (TickType_t ) 1000);

			if(ecuACK == pdFALSE) {
				xQueueReset(ECUQueueHandle);
			}

			vTaskDelay(100);
			dataBuffer[0] = 0x00;
			dataBuffer[1] = 0x00;
		}

		osDelay(50);
	}
  /* USER CODE END StartECUTask */
}

/* USER CODE BEGIN Header_StartInputTask */
/**
 * @brief Function implementing the InputTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartInputTask */
void StartInputTask(void const * argument)
{
  /* USER CODE BEGIN StartInputTask */
	struct InputEvent input;
	input.group = 0x00;
	input.button = 0x00;

	/* Infinite loop */
	for (;;) {

		input = checkJoystick();
		if (input.group != noGroup) {
			xQueueSend(ECUQueueHandle, (void* ) &input, (TickType_t ) 0);
		}
		osDelay(50);
	}
  /* USER CODE END StartInputTask */
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
	while (1) {
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
