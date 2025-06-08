/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <float.h>
#include <dht22.h>
#include "4ilo/4ilo_ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for dht22Task */
osThreadId_t dht22TaskHandle;
const osThreadAttr_t dht22Task_attributes = {
  .name = "dht22Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for BleTask */
osThreadId_t BleTaskHandle;
const osThreadAttr_t BleTask_attributes = {
  .name = "BleTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for CanReceiveTask */
osThreadId_t CanReceiveTaskHandle;
const osThreadAttr_t CanReceiveTask_attributes = {
  .name = "CanReceiveTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for displayTask */
osThreadId_t displayTaskHandle;
const osThreadAttr_t displayTask_attributes = {
  .name = "displayTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
uint8_t canReceived;
uint8_t canMsgCount = 0;
CAN_RxHeaderTypeDef canRcvRxHeader;
uint8_t canRcvData[8];
SemaphoreHandle_t xSemaphore = NULL;
CAN_FilterTypeDef canFilter;
QueueHandle_t dht22Queue = NULL;
QueueHandle_t spiQueue = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void *argument);
void StartDHT22Task(void *argument);
void StartBleTask(void *argument);
void StartCanReceiveTask(void *argument);
void StartDisplayTask(void *argument);

/* USER CODE BEGIN PFP */
static void CAN_Config(void);

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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  CAN_Config();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  xSemaphore = xSemaphoreCreateBinary();
  if(xSemaphore != NULL) {
    if(xSemaphoreGive(xSemaphore) != pdTRUE) {
      printf("[%d] Failed to give semaphore\n\r", __LINE__);
      // We would expect this call to fail because we cannot give
      // a semaphore without first "taking" it!
    }
  }
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  setTimer(&htim6);
  HAL_TIM_Base_Start(&htim6);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  dht22Queue = xQueueCreate(10, sizeof(DHT_DataTypedef));
  if (dht22Queue == NULL) {
    printf("Failed to create dht22Queue\n");
    while(1) {
      osDelay(1);
    }
  }
  spiQueue = xQueueCreate(10, sizeof(DHT_DataTypedef));
  if (spiQueue == NULL) {
    printf("Failed to create spiQueue\n");
    while(1) {
      osDelay(1);
    }
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of dht22Task */
  dht22TaskHandle = osThreadNew(StartDHT22Task, NULL, &dht22Task_attributes);

  /* creation of BleTask */
  BleTaskHandle = osThreadNew(StartBleTask, NULL, &BleTask_attributes);

  /* creation of CanReceiveTask */
  CanReceiveTaskHandle = osThreadNew(StartCanReceiveTask, NULL, &CanReceiveTask_attributes);

  /* creation of displayTask */
  displayTaskHandle = osThreadNew(StartDisplayTask, NULL, &displayTask_attributes);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;    // CPOL=0
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;     // CPHA=0 (Mode 0)
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 89;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port, SPI2_CS1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS1_Pin */
  GPIO_InitStruct.Pin = SPI2_CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DHT22_Pin LD2_Pin */
  GPIO_InitStruct.Pin = DHT22_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

	return ch;
}

/**
  * @brief  Rx Fifo 0 message pending callback
  * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance != CAN1)
  {
	  // printf("CAN1 RX\n\r");
    return;
  }

  /* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &canRcvRxHeader, canRcvData) != HAL_OK)
  {
    /* Reception Error */
    // printf("CAN1 receive message error\n\r");
    return;
  }

  canReceived = 1;
}

static void CAN_Config(void)
{
  canFilter.FilterBank = 0;
  canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
  canFilter.FilterIdHigh = 0;
  canFilter.FilterIdLow = 0;
  canFilter.FilterMaskIdHigh = 0x0000;
  canFilter.FilterMaskIdLow = 0x0000;
  canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilter.FilterActivation = ENABLE;
  canFilter.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan1, &canFilter);

  HAL_CAN_Start(&hcan1);

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    printf("CAN1 activate notification error\n\r");
  }
}

void vMainCanTransmit(CAN_HandleTypeDef *handle, DHT_DataTypedef* dhtData)
{
  CAN_TxHeaderTypeDef txHeader;
  uint32_t txMailbox;
  uint8_t txData[5];
  uint8_t intPart, fractPart;

  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.StdId = 0x030;
  txHeader.ExtId = 0x03;
  txHeader.TransmitGlobalTime = DISABLE;

  txHeader.DLC = 5;
  intPart = (uint8_t)dhtData->Humidity;
  fractPart = (uint8_t)(dhtData->Humidity*100 - intPart*100);
  txData[0] = intPart;
  txData[1] = fractPart;
  intPart = (uint8_t)dhtData->Temperature;
  fractPart = (uint8_t)(dhtData->Temperature*100 - intPart*100);
  txData[2] = intPart;
  txData[3] = fractPart;
  txData[4] = ++canMsgCount;
	printf("CAN [%02d %02d %02d %02d %02d]\n\r",
    txData[0], txData[1], txData[2], txData[3], txData[4]);

  // check if mailboxes are free
  uint32_t timeout = pdMS_TO_TICKS(100);
  TickType_t start = xTaskGetTickCount();
  while(HAL_CAN_GetTxMailboxesFreeLevel(handle) != 3) {
      if((xTaskGetTickCount() - start) > timeout) {
          // timeout
          printf("CAN transmit timeout\n\r");
          break;
      }
      taskYIELD();
  }

  if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox) != HAL_OK)
  {
    /* Transmission request Error */
    printf("CAN transmit error\n\r");
    return;
  }

  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartDHT22Task */
/**
* @brief Function implementing the dht22Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDHT22Task */
void StartDHT22Task(void *argument)
{
  /* USER CODE BEGIN StartDHT22Task */
  vTaskDelay(pdMS_TO_TICKS(1000)); // wait for display task
  DHT_DataTypedef dht22Data;

  /* Infinite loop */
  for(;;)
  {
    dht22Data.Humidity = FLT_MAX;
    dht22Data.Temperature = FLT_MIN;
    if(DHT_GetData(&dht22Data) == 0) {
      printf("DHT_GetData() error\n\r");
    } else {
      printf("\n\rDHT T:%.2f H:%.2f\n\r", dht22Data.Temperature, dht22Data.Humidity);
      if(xQueueSend(dht22Queue, &dht22Data, pdMS_TO_TICKS(100)) != pdPASS) {
        printf("Failed to send data to queue\n\r");
      }
      if(xQueueSend(spiQueue, &dht22Data, pdMS_TO_TICKS(100)) != pdPASS) {
        printf("Failed to send data to queue\n\r");
      }
	}
	vTaskDelay(pdMS_TO_TICKS(3000));
  }
  /* USER CODE END StartDHT22Task */
}

/* USER CODE BEGIN Header_StartBleTask */
/**
* @brief Function implementing the BleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBleTask */

static void SPI_CS_Select(void)
{
  HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port, SPI2_CS1_Pin, GPIO_PIN_RESET);
}

static void SPI_CS_Deselect(void)
{
  HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port, SPI2_CS1_Pin, GPIO_PIN_SET);
}

void StartBleTask(void *argument)
{
  /* USER CODE BEGIN StartBleTask */
  uint8_t txData[4];
  DHT_DataTypedef dht22Data;
  uint8_t intPart, fractPart;

  for(;;)
  {
    if (xQueueReceive(spiQueue, &dht22Data, pdMS_TO_TICKS(100)) == pdPASS)
    {
      intPart = (uint8_t)dht22Data.Temperature;
      fractPart = (uint8_t)(dht22Data.Temperature*100 - intPart*100);
      txData[0] = intPart;
      txData[1] = fractPart;
      intPart = (uint8_t)dht22Data.Humidity;
      fractPart = (uint8_t)(dht22Data.Humidity*100 - intPart*100);
      txData[2] = intPart;
      txData[3] = fractPart;

      printf("SPI [%02x %02x %02x %02x]\n\r", txData[0], txData[1], txData[2], txData[3]);

      SPI_CS_Select();

      HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi2, txData, 4, HAL_MAX_DELAY);

      SPI_CS_Deselect();

      if (status != HAL_OK)
      {
        printf("HAL_SPI_Transmit() failed\n\r");
        Error_Handler();
      }
    }
  }
  /* USER CODE END StartBleTask */
}

/* USER CODE BEGIN Header_StartCanReceiveTask */
/**
* @brief Function implementing the CanReceiveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanReceiveTask */
void StartCanReceiveTask(void *argument)
{
  /* USER CODE BEGIN StartCanReceiveTask */

  /* Infinite loop */
  for(;;)
  {
    if(canReceived == 1) {
      canReceived = 0;
      printf("CAN received [");
      // printf("StdId=0x%lx \n\r", canRcvRxHeader.StdId);
      // printf("RTR=0x%lx \n\r", canRcvRxHeader.RTR);
      // printf("IDE=0x%lx \n\r", canRcvRxHeader.IDE);
      // printf("DLC=0x%lx \n\r", canRcvRxHeader.DLC);
      for (int i=0; i<canRcvRxHeader.DLC-1; i++) {
        printf("%02x ", canRcvData[i]);
      }
      printf("%02x]\n\r", canRcvData[canRcvRxHeader.DLC-1]);
    }

    vTaskDelay(pdMS_TO_TICKS(300));
  }
  /* USER CODE END StartCanReceiveTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the displayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN StartDisplayTask */
  DHT_DataTypedef dht22Data;
  char humidity[16];
  char temperature[16];

  // Init lcd using one of the stm32HAL i2c typedefs
  if (ssd1306_Init(&hi2c1) != 0) {
    Error_Handler();
  }
  vTaskDelay(pdMS_TO_TICKS(100));

  /* Infinite loop */
  for(;;)
  {
    if (xQueueReceive(dht22Queue, &dht22Data, pdMS_TO_TICKS(100)) == pdPASS)
    {
      vMainCanTransmit(&hcan1, &dht22Data);
      memset(humidity, 0, 16);
      memset(temperature, 0, 16);
      snprintf(humidity, 16, "H %.2f", dht22Data.Humidity);
      snprintf(temperature, 16, "T %.2f", dht22Data.Temperature);
      printf("LCD [%s %s]\n\r", temperature, humidity);

      ssd1306_Fill(Black);
      ssd1306_UpdateScreen(&hi2c1);

      // Write data to local screenbuffer
      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString(temperature, Font_7x10, White);

      ssd1306_SetCursor(0, 12);
      ssd1306_WriteString(humidity, Font_7x10, White);

      // Copy all data from local screenbuffer to the screen
      ssd1306_UpdateScreen(&hi2c1);
    }
    // osDelay(1);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  /* USER CODE END StartDisplayTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
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
