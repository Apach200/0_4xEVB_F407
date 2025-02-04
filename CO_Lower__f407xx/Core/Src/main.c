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
  *			LOWER__PCBmodul__STM32_F4VE_V2.0-2
  *
  *			NodeID = 0x3C
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CO_app_STM32.h"
#include "301/CO_SDOclient.h"
#include "CANopen.h"
#include "OD.h"

#include "format_out.h"
#include "SDO_utils.h"
#include "lcd.h"
#include "Encoder.h"
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

/* USER CODE BEGIN PV */
uint8_t Tx_Array[16]={0x5a,0x6b,0x7c,0x86,0x5f,0x43,0x8e,0x58,0x69,0x70,0x81,0x92,0xf3,0xc4,0xc5,0xc3};
uint8_t Rx_Array[16]={0};
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
/* USER CODE BEGIN PFP */

void CAN_interface_Test(void);
void UART_interface_Test(void);
//void GPIO_Blink_Test(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t Count_of_Blink, uint16_t Period_of_blink_ms);
void Board_Name_to_Terminal(void);
void CO_Init_Return_State(uint16_t Returned_Code);



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
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_CAN1_Init();
  MX_RTC_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  /* CANHandle : Pass in the CAN Handle to this function and it wil be used for all CAN Communications. It can be FDCan or CAN
   * and CANOpenSTM32 Driver will take of care of handling that
   * HWInitFunction : Pass in the function that initialize the CAN peripheral, usually MX_CAN_Init
   * timerHandle : Pass in the timer that is going to be used for generating 1ms interrupt for tmrThread function,
   * please note that CANOpenSTM32 Library will override HAL_TIM_PeriodElapsedCallback function, if you also need this function
   * in your codes, please take required steps
   * desiredNodeID : This is the Node ID that you ask the CANOpen stack to assign to your device, although it might not always
   * be the final NodeID, after calling canopen_app_init() you should check ActiveNodeID of CANopenNodeSTM32 structure for assigned Node ID.
   * baudrate: This is the baudrate you've set in your CubeMX Configuration
   *
   */

     GPIO_Blink_Test(GPIOA, GPIO_PIN_7|GPIO_PIN_6, 25, 33); //for_STM32F4XX_Ali_pcb
//    GPIO_Blink_Test(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, 25, 33);// blink_at_Discovery_EVB

//    UART_interface_Test(); //while(1){;}
//  //  CAN_interface_Test();
    Board_Name_to_Terminal();
    OD_PERSIST_COMM.x1018_identity.serialNumber = HAL_GetUIDw0();

   HAL_TIM_Base_Start_IT(&htim4);

   CANopenNodeSTM32 canOpenNodeSTM32;
   canOpenNodeSTM32.CANHandle = &hcan1;
   canOpenNodeSTM32.HWInitFunction = MX_CAN1_Init;
   canOpenNodeSTM32.timerHandle = &htim4;
   //canOpenNodeSTM32.desiredNodeID = CO_Lower__f407xx;  //0x3c;
   canOpenNodeSTM32.desiredNodeID = Node_Unconfigured;
   canOpenNodeSTM32.baudrate = 125*4;
   uint16_t Ret_value = canopen_app_init(&canOpenNodeSTM32);
   CO_Init_Return_State(Ret_value);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

   canopen_app_process();



		Local_Count=0;
		  OD_PERSIST_COMM.x6000_lowerF_VAR32_6000=0;

		  while (1)
		  {
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, !canOpenNodeSTM32.outStatusLEDGreen);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, !canOpenNodeSTM32.outStatusLEDRed  );//yellow
//		         HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6 );//LED1_Pin___//LED1_GPIO_Port//green

			canopen_app_process();


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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

/* USER CODE BEGIN 4 */

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

///////////////////////////////////////////////////

void UART_interface_Test(void)
{
	// Test_Terminal__ASCII
	  Length_of_Message = sprintf( Message_to_Terminal,
			  	  	  	  	  	  	  "Rx_Array[0]=0x%x, Rx_Array[1]= 0x%x, Rx_Array[2]= 0x%x, Rx_Array[3]= 0x%x \n\r",
									   Rx_Array[0],Rx_Array[1],Rx_Array[2],Rx_Array[3]
								 );
	  TerminalInterface.gState = HAL_UART_STATE_READY;
	  //HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message);
	  HAL_UART_Transmit( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message,50);

//    Test_Terminal__HEX
//
//	  HAL_Delay(500);
//	  Local_Count = sizeof String_L;
//	  String_L[Local_Count-1] = 0x0d;
//	  TerminalInterface.gState = HAL_DMA_STATE_READY;
//	  HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)(String_L), Local_Count);

}
//////////////////////////////////////////////

//void GPIO_Blink_Test(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t Count_of_Blink, uint16_t Period_of_blink_ms)
//{
//  for(uint8_t cnt=0;cnt<Count_of_Blink;cnt++)
//  {
//  HAL_GPIO_TogglePin(GPIOx, GPIO_Pin );
//  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6 );
//  HAL_Delay(Period_of_blink_ms);
//  }
//  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
//  //HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
//}

//////////////////////////////////////////////////
void Board_Name_to_Terminal(void)
{
	const char Message_0[]={"   ******************************************\n\r"};
//	const char Message_1[]={"*  Upper Blackboard  STM32F4XX___Ali     *\n\r"};
	const char Message_1[]={"*  Lower Blackboard  STM32F4XX___Ali     *\n\r"};
//  const char Message_1[]={"*  STM32F4DISCOVERY Green_board China    *\n\r"};
//	const char Message_1[]={"*  STM32F4DISCOVERY Blue_board Original  *\n\r"};
//  const char Message_1[]={"*  STM32F4DISCOVERY Green_board Original *\n\r"};
	char Array_for_Messages[128]={};
	uint16_t Msg_Length;
//	uint32_t Chip_ID_96bit[4]={};
//	uint16_t  *pChip_ID_96bit =(uint16_t*)Chip_ID_96bit ;

//	Chip_ID_96bit[0] = HAL_GetUIDw0();
//	Chip_ID_96bit[1] = HAL_GetUIDw1();
//	Chip_ID_96bit[2] = HAL_GetUIDw2();

	Msg_Length = sizeof(Message_0);
	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_0, Msg_Length);

	Msg_Length = sizeof(Message_1);
	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
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

///////////////////////////////////////////////////////////////////////////////

void CO_Init_Return_State(uint16_t Returned_Code)
{
	uint16_t Lngth_of_Message;
	char Msg_2_Terminal[400];
	uint16_t Code;
	Code = Returned_Code;
	Lngth_of_Message = sprintf( Msg_2_Terminal,"Returned_Code = 0x%x, \n\r\n\r",Returned_Code);
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Msg_2_Terminal, Lngth_of_Message);
switch(Code)
{
case 0:
	  HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)"  canopen_app_init OK !\n\r\n\r",
													 sizeof"  canopen_app_init OK !\n\r\n\r");break;
case 1:
	  HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)"Error: Can't allocate memory!\n\r\n\r",
			  	  	  	  	  	  	  	  	  	  	 sizeof"Error: Can't allocate memory!\n\r\n\r");break;
case 2:
	  HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)"Error: Storage %d ! \n\r\n\r",
			  	  	  	  	  	  	  	  	  	  	 sizeof"Error: Storage %d ! \n\r\n\r");break;
default:
	  HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)"UNKNOWN code =0x%x \n\r",
			  	  	  	  	  	  	  	  	  	  	 sizeof"UNKNOWN code =0x%x \n\r");break;
} //switch

while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
HAL_Delay(50);
	Lngth_of_Message = sprintf( Msg_2_Terminal,
		  	  	  "   *  CANopenNodeSTM32 canOpenNodeSTM32;        *\n\r"
		  	  	  "   *  .CANHandle = &hcan1;                      *\n\r"
		  	  	  "   *  .HWInitFunction = MX_CAN1_Init            *\n\r"
		  	  	  "   *  .timerHandle = &htim4                     *\n\r"
		  	  	  "   *  .baudrate = 500kbps                       *\n\r"
		  	  	  "   *  .desiredNodeID = STLINK_4323;             *\n\r\n\r"
		  	  	  "   *  canopen_app_init(&canOpenNodeSTM32);      * \n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r"
			   );
		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Msg_2_Terminal, Lngth_of_Message);
		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
		HAL_Delay(50);

}// void CO_Init_Return_State

////////////////////////////////////////////////////////////////////////////////

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
