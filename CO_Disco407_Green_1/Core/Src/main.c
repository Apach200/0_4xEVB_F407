/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  *		Discovery_F407green__EvolutionBoard_Original
  *
  *			NodeID = 0x3E
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CO_app_STM32.h"
#include "301/CO_SDOclient.h"
#include "CANopen.h"
#include "OD.h"
#include "format_out.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CO_Aliex_Disco407green	0x3A
#define CO_Disco407_Blue		0x3b
#define CO_Lower__f407xx		0x3c
#define CO_Upper_F407XX			0x3d
#define CO_Disco407_Green_1		0x3e

#define Make_Read_SDO			1
#define TerminalInterface		huart2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

uint8_t Tx_Array[16]={0x5a,0x6b,0x7c,0x86,0x5f,0x43,0x8e,0x58,0x69,0x70,0x81,0x92,0xf3,0xc4,0xc5,0xc3};
uint8_t Rx_Array[16]={0x43,0x8e,0x58,0x69,0x70};
uint32_t Array_32u[16]={0};
uint8_t Array_8u[16]={0x54,0x34,0x21,0xea,0xf3,0x7a,0xd4,0x46};
char Message_to_Terminal[128]={};
uint8_t Length_of_Message;
uint8_t Length_of_Ext_Var=0;
uint8_t Local_Count=0;



CO_SDO_abortCode_t  Code_return_SDO;
CAN_TxHeaderTypeDef Tx_Header;
uint32_t            TxMailbox;
uint32_t            tmp32u_1   = 0x1e1f1a1b;
uint32_t            tmp32u_0   = 0x0e0f0a0b;
uint64_t            tmp64u_0   = 0x0e1f1a1b56789a;
uint64_t            tmp64u_1   = 0x0e1f1a1b56789a;
uint32_t            Ticks;
char String_L[]={"String_for_Test_UART_"};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM14_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void CAN_interface_Test(void);
void UART_interface_Test(void);
void GPIO_Blink_Test(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t Count_of_Blink, uint16_t Period_of_blink_ms);
void Board_Name_to_Terminal(void);
void CO_Init_Return_State(uint16_t Returned_Code);
void SDO_abortCode_ASCII_to_Terminal(void);

CO_SDO_abortCode_t	read_SDO	(
								  CO_SDOclient_t* SDO_C,
								  uint8_t nodeId, 	//Remote_NodeID
								  uint16_t index,	//OD_Index_of_entire_at_Remote_NodeID
								  uint8_t subIndex, // OD_SubIndex_of_entire_at_Remote_NodeID
								  uint8_t* buf, 	//Saved_Data_Array
								  size_t bufSize, 	//Number_of_Bytes_Read_from_Remote_NodeID
								  size_t* readSize 	//pointer_at_Number_of_Bytes_to_save
								  );


CO_SDO_abortCode_t	write_SDO 	(
								CO_SDOclient_t* SDO_C,
								uint8_t nodeId, 	//Remote_NodeID
								uint16_t index,	//OD_Index_of_entire_at_Remote_NodeID
								uint8_t subIndex, // OD_SubIndex_of_entire_at_Remote_NodeID
								uint8_t* data,	//Data_Array_to_write_into_entire_at_Remote_NodeID
								size_t dataSize	//Number_of_Bytes_write_into_entire_at_Remote_NodeID
								);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Timer interrupt function executes every 1 ms */
void
HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == canopenNodeSTM32->timerHandle) {
        canopen_app_interrupt();

    }
}


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
  MX_DMA_Init();
  MX_TIM14_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* CANHandle : Pass in the CAN Handle to this function and it wil be used for all CAN Communications.
   * It can be FDCan or CAN and CANOpenSTM32 Driver will take of care of handling that
   *
   * HWInitFunction : Pass in the function that initialize the CAN peripheral, usually MX_CAN_Init
   *
   * timerHandle : Pass in the timer that is going to be used for generating 1ms interrupt for tmrThread function,
   * please note that CANOpenSTM32 Library will override HAL_TIM_PeriodElapsedCallback function,
   * if you also need this function in your codes, please take required steps
   *
   * desiredNodeID : This is the Node ID that you ask the CANOpen stack to assign to your device, although it might not always
   * be the final NodeID, after calling canopen_app_init() you should check ActiveNodeID of CANopenNodeSTM32 structure for assigned Node ID.
   *
   * baudrate: This is the baudrate you've set in your CubeMX Configuration
   *
   */





//  UART_interface_Test(); while(1){;}
//  CAN_interface_Test();

//   GPIO_Blink_Test(GPIOA, GPIO_PIN_7|GPIO_PIN_6, 25, 33); //for_STM32F4XX_Ali_pcb
  GPIO_Blink_Test(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, 25, 33);// blink_at_Discovery_EVB


  Board_Name_to_Terminal();


   HAL_TIM_Base_Start_IT(&htim4);

   CANopenNodeSTM32 canOpenNodeSTM32;
   canOpenNodeSTM32.CANHandle = &hcan1;
   canOpenNodeSTM32.HWInitFunction = MX_CAN1_Init;
   canOpenNodeSTM32.timerHandle = &htim4;
   canOpenNodeSTM32.desiredNodeID = CO_Disco407_Green_1;//		0x3e
   canOpenNodeSTM32.baudrate = 125*4;
   uint16_t Ret_value = canopen_app_init(&canOpenNodeSTM32);
   CO_Init_Return_State(Ret_value);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

#if Make_Read_SDO
   canopen_app_process();

	  read_SDO (
			    canOpenNodeSTM32.canOpenStack->SDOclient,
				CO_Disco407_Blue,							//Remote_NodeID=0x3b
				0x600E,										//Index_of_OD_variable_Disco_Blue_VAR64_600e_TX at_remote_Node
				0,											//Sub_Index_of_OD_variable
				Rx_Array,									//Saved_Received_Data
				4,											//Number_of_Byte_to_read
				(size_t*)&Length_of_Ext_Var );  HAL_Delay(100);


	  write_SDO(
			    canOpenNodeSTM32.canOpenStack->SDOclient,
				CO_Disco407_Blue,							//Remote_NodeID=0x3b
				0x600E,										//Index_of_OD_variable_Disco_Blue_VAR64_600e_TX at_remote_Node
				0,											//Sub_Index_of_OD_variablee
				Array_8u,									//Data_Source
				4);	  HAL_Delay(100);



	  read_SDO (
			    canOpenNodeSTM32.canOpenStack->SDOclient,
				CO_Disco407_Blue,							//Remote_NodeID=0x3b
				0x600E,										//Index_of_OD_variable_Disco_Blue_VAR64_600e_TX at_remote_Node
				0,											//Sub_Index_of_OD_variable
				Rx_Array,									//Saved_Received_Data
				4,											//Number_of_Byte_to_read
				(size_t*)&Length_of_Ext_Var );  HAL_Delay(100);


	  TerminalInterface.gState = HAL_UART_STATE_READY;
	  HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Rx_Array, 8);
      HAL_Delay(100);
#endif

      Local_Count=0;
		  OD_PERSIST_COMM.x6000_ALiex_Disco_VAR32_6000=0;

		  while (1)
		  {
			 // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, !canOpenNodeSTM32.outStatusLEDGreen);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, !canOpenNodeSTM32.outStatusLEDGreen);
			 // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, !canOpenNodeSTM32.outStatusLEDRed  );//yellow
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, !canOpenNodeSTM32.outStatusLEDRed  );
//		         HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6 );//LED1_Pin___//LED1_GPIO_Port//green

			canopen_app_process();

//			if(tmp32u_1 != OD_PERSIST_COMM.x6001_nucleo_VAR32_6001)
//				{
//				tmp32u_1 = OD_PERSIST_COMM.x6001_nucleo_VAR32_6001;
//				TerminalInterface.gState = HAL_UART_STATE_READY;
//				HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)(&tmp32u_1), 4);
//				}


			  if(HAL_GetTick() - Ticks>749)
			  {
				Ticks = HAL_GetTick();

				OD_PERSIST_COMM.x6000_ALiex_Disco_VAR32_6000++;

				tmp32u_0 = OD_PERSIST_COMM.x6000_ALiex_Disco_VAR32_6000;

				//CO_TPDOsendRequest(&canOpenNodeSTM32.canOpenStack->TPDO[0] );

				TerminalInterface.gState = HAL_UART_STATE_READY;
				HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)( &tmp32u_0 ), 4);

				Local_Count++;
				Local_Count = Local_Count%4;

				if(Local_Count==0){
									OD_PERSIST_COMM.x6039_ALiex_Disco_Array[0]++;
								  }

				CO_TPDOsendRequest(&canOpenNodeSTM32.canOpenStack->TPDO[Local_Count] );
			  }

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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 167;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 167;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_2;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

CO_SDO_abortCode_t	read_SDO	(
								  CO_SDOclient_t* SDO_C,
								  uint8_t nodeId, 	//Remote_NodeID
								  uint16_t index,	//OD_Index_of_entire_at_Remote_NodeID
								  uint8_t subIndex, // OD_SubIndex_of_entire_at_Remote_NodeID
								  uint8_t* buf, 	//Saved_Data_Array
								  size_t bufSize, 	//Number_of_Bytes_Read_from_Remote_NodeID
								  size_t* readSize 	//pointer_at_Number_of_Bytes_to_save
								  )
{
    CO_SDO_return_t SDO_ret;

    // setup client (this can be skipped, if remote device don't change)
    SDO_ret = CO_SDOclient_setup (
    								SDO_C, CO_CAN_ID_SDO_CLI + nodeId,
									CO_CAN_ID_SDO_SRV + nodeId,
									nodeId);

    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) { return CO_SDO_AB_GENERAL; }



    // initiate upload
    SDO_ret = CO_SDOclientUploadInitiate ( SDO_C,
    										index,
											subIndex,
											1000,
											false);

    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) { return CO_SDO_AB_GENERAL; }

    // upload data
    do 	{
        uint32_t timeDifference_us = 10000;
        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

        SDO_ret = CO_SDOclientUpload(SDO_C, timeDifference_us, false, &abortCode, NULL, NULL, NULL);

        if (SDO_ret < 0) {  return abortCode;  }

        HAL_Delay(timeDifference_us/1000);// sleep_us(timeDifference_us);

    	} while (SDO_ret > 0);


    // copy data to the user buffer (for long data function must be called several times inside the loop)
    *readSize = CO_SDOclientUploadBufRead(SDO_C, buf, bufSize);

    return CO_SDO_AB_NONE;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////


CO_SDO_abortCode_t	write_SDO 	(
								CO_SDOclient_t* SDO_C,
								uint8_t nodeId, 	// Remote_NodeID
								uint16_t index,		// OD_Index_of_entire_at_Remote_NodeID
								uint8_t subIndex, 	// OD_SubIndex_of_entire_at_Remote_NodeID
								uint8_t* data,		// Data_Array_to_write_into_entire_at_Remote_NodeID
								size_t dataSize		// Number_of_Bytes_write_into_entire_at_Remote_NodeID
								)
{
    CO_SDO_return_t SDO_ret;
    bool_t bufferPartial = false;

    // setup client (this can be skipped, if remote device is the same)
    SDO_ret = CO_SDOclient_setup (	SDO_C,
    								CO_CAN_ID_SDO_CLI + nodeId,
									CO_CAN_ID_SDO_SRV + nodeId,
									nodeId);

    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) { return -1; }



    // initiate download
    SDO_ret = CO_SDOclientDownloadInitiate(SDO_C, index, subIndex, dataSize, 1000, false);

    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) /**< Success, end of communication. SDO client: uploaded data must be read. */
    	{ return -1; }



    // fill data
    size_t nWritten = CO_SDOclientDownloadBufWrite(SDO_C, data, dataSize);

    if (nWritten < dataSize) { bufferPartial = true; } // If SDO Fifo buffer is too small, data can be refilled in the loop.




    // download data
    do {
        uint32_t timeDifference_us = 10000;
        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

        SDO_ret = CO_SDOclientDownload (	SDO_C,
        									timeDifference_us,
											false, bufferPartial,
											&abortCode,
											NULL,
											NULL
										);

        if (SDO_ret < 0) {  return abortCode;}

        HAL_Delay(timeDifference_us/1000); //sleep_us(timeDifference_us);

       } while (SDO_ret > 0);

    return CO_SDO_AB_NONE;
}

//////////////////////////////////////////////////////////
void CAN_interface_Test(void)
{

 Tx_Header.IDE    = CAN_ID_STD;
 Tx_Header.ExtId  = 0;
 Tx_Header.DLC    = 8;
 Tx_Header.StdId  = 0x7EC;
 Tx_Header.RTR    = CAN_RTR_DATA;
 HAL_CAN_Start(&hcan1);  HAL_Delay(1500);

 if(HAL_CAN_AddTxMessage( &hcan1,
   		               &Tx_Header,
							Tx_Array, &TxMailbox )==HAL_OK
 	 )
	  {  /* Wait transmission complete */
	  //while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {}
		  for(uint8_t cnt=0;cnt<22;cnt++)
		  {
		   HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);//LED2_Pin//yellow
		   HAL_Delay(46);
		  }
	  }
}

//////////////////////////////////////////////

void UART_interface_Test(void)
{
	// Test_Terminal__ASCII
	  Length_of_Message = sprintf( Message_to_Terminal,
			  	  	  	  	  	  	  "Rx_Array[0]=0x%x, Rx_Array[1]= 0x%x, Rx_Array[2]= 0x%x, Rx_Array[3]= 0x%x \n\r",
									   Rx_Array[0],Rx_Array[1],Rx_Array[2],Rx_Array[3]
								 );
	  TerminalInterface.gState = HAL_UART_STATE_READY;
	  HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message);

//    Test_Terminal__HEX
//
//	  HAL_Delay(500);
//	  Local_Count = sizeof String_L;
//	  String_L[Local_Count-1] = 0x0d;
//	  TerminalInterface.gState = HAL_DMA_STATE_READY;
//	  HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)(String_L), Local_Count);

}

//////////////////////////////////////////////

void GPIO_Blink_Test(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t Count_of_Blink, uint16_t Period_of_blink_ms)
{
	for(uint8_t cnt=0;cnt<Count_of_Blink;cnt++)
	{
	HAL_GPIO_TogglePin(GPIOx, GPIO_Pin );
	HAL_Delay(Period_of_blink_ms);
	}
	  //HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

//////////////////////////////////////////////////
void Board_Name_to_Terminal(void)
{
	const char Message_0[]={"   ******************************************\n\r"};
//	const char Message_1[]={"*  Upper Blackboard  STM32F4XX___Ali     *\n\r"};
//	const char Message_1[]={"*  Lower Blackboard  STM32F4XX___Ali     *\n\r"};
	//const char Message_1[]={"*  STM32F4DISCOVERY Green_board China    *\n\r"};
	//	const char Message_1[]={"*  STM32F4DISCOVERY Blue_board Original  *\n\r"};
		const char Message_1[]={"*  STM32F4DISCOVERY Green_board Original *\n\r"};
	char Array_for_Messages[128]={};
	uint16_t Msg_Length;
//	uint32_t Chip_ID_96bit[4]={};
//	uint16_t  *pChip_ID_96bit =(uint16_t*)Chip_ID_96bit ;

//	Chip_ID_96bit[0] = HAL_GetUIDw0();
//	Chip_ID_96bit[1] = HAL_GetUIDw1();
//	Chip_ID_96bit[2] = HAL_GetUIDw2();

	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	Msg_Length = sizeof(Message_0);
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_0, Msg_Length);

	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	Msg_Length = sizeof(Message_1);
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_1, Msg_Length);

	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	Msg_Length = sprintf( Array_for_Messages,
			  	  	  	  "*  SystemClock = %d MHz                 *\n\r",
						  (uint16_t)(HAL_RCC_GetSysClockFreq()/1000000)
						);
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Array_for_Messages, Msg_Length);

	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	Msg_Length = sprintf( Array_for_Messages,
			  	  	  	  "   *  Unical_ID %X%X%X%X%X%X        *\n\r",
						  (uint16_t)(HAL_GetUIDw2()>>16),(uint16_t)(HAL_GetUIDw2() & 0x0000FFFF),
						  (uint16_t)(HAL_GetUIDw1()>>16),(uint16_t)(HAL_GetUIDw1() & 0x0000FFFF),
						  (uint16_t)(HAL_GetUIDw0()>>16),(uint16_t)(HAL_GetUIDw0() & 0x0000FFFF)

						);
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Array_for_Messages, Msg_Length);

	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	Msg_Length = sprintf( Array_for_Messages,
			  	  	  	  "   *  Device identifier %X%X                *\n\r",
						  (uint16_t)(HAL_GetDEVID()>>16), (uint16_t)(HAL_GetDEVID() & 0x0000FFFF)
						);
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Array_for_Messages, Msg_Length);

	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	Msg_Length = sprintf( Array_for_Messages,
			  	  	  	  "   *  Device revision identifier %X%X      *\n\r",
						  (uint16_t)( HAL_GetREVID()>>16 ),
						  (uint16_t)( HAL_GetREVID() & 0x0000FFFF )
						);
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Array_for_Messages, Msg_Length);

	Msg_Length = sizeof(Message_0);
	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_0, Msg_Length);

	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	Array_for_Messages[0]=0x0a;		Array_for_Messages[1]=0x0d;
	Array_for_Messages[2]=0x0a;		Array_for_Messages[3]=0x0d;
	Array_for_Messages[4]=0x0a;		Array_for_Messages[5]=0x0d;
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)(Array_for_Messages), 6);
	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
}

//////////////////////////////////////////////////////
//////////////////////////////////////////////////
void CO_Init_Return_State(uint16_t Returned_Code)
{
	uint16_t Lngth_of_Message;
	char Msg_2_Terminal[400];
	   if (Returned_Code==0){
	   const uint8_t Msg_0[]="canopen_app_init OK\n\r";
	   Lngth_of_Message = sizeof(Msg_0);
	   	  while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	   	  HAL_UART_Transmit_DMA( &TerminalInterface, Msg_0, Lngth_of_Message);
	   	 }else if(Returned_Code==1) {
	   		const uint8_t Msg_1[]="Error: Can't allocate memory\n\r";
	   		Lngth_of_Message = sizeof(Msg_1);
	   		 while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	   		  HAL_UART_Transmit_DMA( &TerminalInterface, Msg_1, Lngth_of_Message);
	   		 }else if(Returned_Code==2) {
	   			const uint8_t Msg_2[]="Error: Storage %d\n\r";
	   			Lngth_of_Message = sizeof(Msg_2);
	   			 while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	   			  HAL_UART_Transmit_DMA( &TerminalInterface, Msg_2, Lngth_of_Message);
	   			 }else{;}

		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	   Lngth_of_Message = sprintf( Msg_2_Terminal,
		  	  	  "   *  CANopenNodeSTM32 canOpenNodeSTM32;        *\n\r"
		  	  	  "   *  .CANHandle = &hcan1;                      *\n\r"
		  	  	  "   *  .HWInitFunction = MX_CAN1_Init            *\n\r"
		  	  	  "   *  .timerHandle = &htim4                     *\n\r"
		  	  	  "   *  .baudrate = 500kbps                       *\n\r"
		  	  	  "   *  .desiredNodeID = CO_Disco407_Blue;//0x3b; *\n\r"
		  	  	  "   *  canopen_app_init(&canOpenNodeSTM32);      *\n\r\n\r\n\r"
			   );
		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Msg_2_Terminal, Lngth_of_Message);
		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
}

///////////////////////////////////////////////////

void SDO_abortCode_ASCII_to_Terminal(void)
{
	char Message_2_Terminal[128];
	uint8_t Message_Length;
while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
Message_Length = SDO_abortCode_to_String(Code_return_SDO,  Message_2_Terminal);
HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_2_Terminal, Message_Length);
}
//////////////////////////////////////////



/* USER CODE END 4 */

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
