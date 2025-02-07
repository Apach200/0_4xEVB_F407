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
  *			Aliexpress_Disco407green__EvolutionBoard
  *
  *			NodeID = 0x3A
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
#include "CO_PDO.h"


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

RTC_DateTypeDef DateToUpdate;//  = {02, 01, 21, 25};  //; ={0,0,0};   //21jan2025
RTC_TimeTypeDef sTime;// = {19, 24, 0,0,0,0,0};; // ={0,0,0};      ///19h16min
//					    0,//uint8_t Hours; Max_Data=12 if the RTC_HourFormat_12; Max_Data=23 if the RTC_HourFormat_24
//						0,//uint8_t Minutes; Max_Data = 59
//						0,//uint8_t Seconds; Max_Data = 59 */
//						0,//uint8_t TimeFormat;Specifies the RTC AM/PM Time.
//						0,//uint32_t SecondFraction;parameter corresponds to a time unit range between [0-1] Second with [1 Sec / SecondFraction +1] granularity
//						0,//uint32_t DayLightSaving;  This interface is deprecated.
//						0//uint32_t StoreOperation;

uint8_t Tx_Array[16]={0x51,0x62,0x73,0x84,0x55,0x46,0x87,0x18,0x29,0x10,0x11,0x12,0x13,0x14,0x15,0x33};
uint8_t Rx_Array[16]={0};
uint32_t Array_32u[16]={0};
uint8_t Array_8u[16]={0x54,0x34,0x21,0xea,0xf3,0x7a,0xd4,0x46};
char Message_to_Terminal[128]={};
char Message_to_Terminal_1[128]={};
char Message_to_Terminal_2[128]={};
char Message_to_Terminal_3[128]={};
uint8_t Array_from_Terminal[128]={0};
uint8_t Length_of_Message;
uint8_t Length_of_Ext_Var=0;
uint8_t Local_Count=0;
uint64_t Count_of_while1=0;
float ChipTemperature;

CANopenNodeSTM32    canOpenNodeSTM32;
CO_SDO_abortCode_t  Code_return_SDO;
CAN_TxHeaderTypeDef Tx_Header;
uint32_t            TxMailbox;
uint32_t            tmp32u_1   = 0x1e1f1a1b;
uint32_t            tmp32u_0   = 0x0e0f0a0b;
uint64_t            tmp64u_0   = 0x0e1f1a1b56789a;
uint64_t            tmp64u_1   = 0x0e1f1a1b56789a;
uint32_t            Ticks;
uint32_t            Ticks_1;
char String_H[]={"String_for_Test_UART_"};
char String_L[]={"String_for_Test_UART_"};
char buff[16]={8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};
//const	char XPOMOC[] = "XPOMOC_CANOpen";
//const	char WC[] = "LSS_Master";
char String_LCD[32];
char String_2_UART[128];
uint16_t L_str;
int16_t currCounter=0 ;
int32_t prevCounter =0;
CO_ReturnError_t Err_return;

uint8_t Menu_step=0;
const uint16_t Datum[64]={0,0,
						  0x30,0x31, 0x30,0x32, 0x30,0x33, 0x30,0x34, 0x30,0x35, 0x30,0x36, 0x30,0x37, 0x30,0x38, 0x30,0x39, 0x31,0x30,
					      0x31,0x31, 0x31,0x32, 0x31,0x33, 0x31,0x34, 0x31,0x35, 0x31,0x36, 0x31,0x37, 0x31,0x38, 0x31,0x39, 0x32,0x30,
					      0x32,0x31, 0x32,0x32, 0x32,0x33, 0x32,0x34, 0x32,0x35, 0x32,0x36, 0x32,0x37, 0x32,0x38, 0x32,0x39, 0x33,0x30,
					      0x33,0x31
					     };



uint8_t Node_ID_Read=0xff;
uint32_t Data_u32;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CAN_interface_Test(void);
void UART_interface_Test(void);
void Board_Name_to_Terminal(void);
void CO_Init_Return_State(uint16_t Returned_Code);
void SDO_abortCode_ASCII_to_Terminal(void);
CO_SDO_abortCode_t SDO_Read_Write_Read(void);
void TPDO_send(uint32_t Period_ms);

void SDO_Test();

int16_t Encoder_to_LCD(void);
Encoder_Status encoderStatus;
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

  //	HAL_RTC_SetTime(&hrtc, &sTime,        RTC_FORMAT_BIN);
  //	HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
  HAL_RTC_GetTime(&hrtc, &sTime,        RTC_FORMAT_BIN);

  Encoder_Config();  // configure the encoders timer
  Encoder_Init();    // start the encoders timer
  LCD_ini();
  // Logo_to_1602LCD();
  Datum_to_1602LCD();
  //GPIO_Blink_Test(GPIOA, GPIO_PIN_7|GPIO_PIN_6, 25, 33); 						// for_STM32F4XX_Ali_pcb
   GPIO_Blink_Test(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, 25, 33);// blink_at_Discovery_EVB
  //UART_interface_Test(); //while(1){;}
  //CAN_interface_Test();

  HAL_TIM_Base_Start_IT(&htim8);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_UART_Receive_DMA(&huart2, Array_from_Terminal, sizeof Array_from_Terminal );
  //HAL_Delay(1500);
  Board_Name_to_Terminal();
  OD_PERSIST_COMM.x1018_identity.serialNumber = HAL_GetUIDw0();

  Message_2_UART_u16("TEST", 0xfede);

    /* CANHandle : Pass in the CAN Handle to this function and it wil be used for all CAN Communications.
     *             It can be FDCan or CAN and CANOpenSTM32 Driver will take of care of handling that
     *
     * HWInitFunction : Pass in the function that initialize the CAN peripheral, usually MX_CAN_Init
     *
     * timerHandle : Pass in the timer that is going to be used for generating 1ms interrupt for tmrThread function,
     *               please note that CANOpenSTM32 Library will override HAL_TIM_PeriodElapsedCallback function,
     *               if you also need this function in your codes, please take required steps
     *
     * desiredNodeID : This is the Node ID that you ask the CANOpen stack to assign to your device, although it might not always
     *                 be the final NodeID, after calling canopen_app_init()
     *                 you should check ActiveNodeID of CANopenNodeSTM32 structure for assigned Node ID.
     *
     * baudrate: This is the baudrate you've set in your CubeMX Configuration
     *
     */
   canOpenNodeSTM32.CANHandle = &hcan1;
   canOpenNodeSTM32.HWInitFunction = MX_CAN1_Init;
   canOpenNodeSTM32.timerHandle = &htim4;
   canOpenNodeSTM32.desiredNodeID = CO_Aliex_Disco407green;//	0x3A
   //canOpenNodeSTM32.desiredNodeID = Node_Unconfigured;
   canOpenNodeSTM32.baudrate = 125*4;
   uint16_t Ret_value = canopen_app_init(&canOpenNodeSTM32);
   CO_Init_Return_State(Ret_value );

   	 //SDO_Read_Write_Read();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


//   canopen_app_process();

#if Make_Read_SDO
#endif//Make_Read_SDO

		  HAL_Delay(50);
	  	  Local_Count=0;
      	  OD_PERSIST_COMM.x6000_ALiex_Disco_VAR32_6000=0;
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);//Green
		  				//tmp32u_0 = OD_PERSIST_COMM.x6000_lowerF_VAR32_6000;

//			CO_TPDOsendRequest(&canOpenNodeSTM32.canOpenStack->TPDO[0] );HAL_Delay(50);
//			CO_TPDOsendRequest(&canOpenNodeSTM32.canOpenStack->TPDO[1] );HAL_Delay(50);
//			CO_TPDOsendRequest(&canOpenNodeSTM32.canOpenStack->TPDO[2] );HAL_Delay(50);
//			CO_TPDOsendRequest(&canOpenNodeSTM32.canOpenStack->TPDO[3] );HAL_Delay(50);
			Ticks = HAL_GetTick();
		  while (1)
		  {
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, !canOpenNodeSTM32.outStatusLEDGreen);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, !canOpenNodeSTM32.outStatusLEDRed  );
			 // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, !canOpenNodeSTM32.outStatusLEDGreen);
			 // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, !canOpenNodeSTM32.outStatusLEDRed  );//yellow

			canopen_app_process();
			  Encoder_to_LCD();
			  RTC_update_and_Terminal(2000);


			  if(HAL_GetTick() - Ticks>399)
			  {
				Ticks = HAL_GetTick();

			if(tmp32u_1 != OD_PERSIST_COMM.x6001_ALiex_Disco_VAR32_6001)
				{
				tmp32u_1 = OD_PERSIST_COMM.x6001_ALiex_Disco_VAR32_6001;
				}

				OD_PERSIST_COMM.x6000_ALiex_Disco_VAR32_6000++;
				tmp32u_0 = OD_PERSIST_COMM.x6000_ALiex_Disco_VAR32_6000;
				CO_TPDOsendRequest(&canOpenNodeSTM32.canOpenStack->TPDO[0] );
				TerminalInterface.gState = HAL_UART_STATE_READY;
				HAL_UART_Transmit_IT( &TerminalInterface, (uint8_t*)( &tmp32u_0 ), 4);
				Local_Count++;Local_Count = Local_Count%4;
				if(Local_Count==0){OD_PERSIST_COMM.x6039_ALiex_Disco_Array[0]++;}
				CO_TPDOsendRequest(&canOpenNodeSTM32.canOpenStack->TPDO[Local_Count] );
				//LSS_Service_Info(canOpenNodeSTM32.canOpenStack->LSSslave->service);
				//LSS_Service_Info(canOpenNodeSTM32.canOpenStack->LSSslave->lssState);
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

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



//////////////////////////////////////////////////
void Board_Name_to_Terminal(void)
{
	const char Message_0[]={"   ******************************************\n\r"};
//	const char Message_1[]={"*  Upper Blackboard  STM32F4XX___Ali     *\n\r"};
//	const char Message_2[]={"*  Lower Blackboard  STM32F4XX___Ali     *\n\r"};
//	const char Message_3[]={"*  STM32F4DISCOVERY Green_board China    *\n\r"};
	const char Message_3[]={"*  STM32F4DISCO Greenboard_STLINK_4323   *\n\r"};
//	const char Message_3[]={"*  STM32F4DISCO Greenboard_STLINK_2734   *\n\r"};
//	const char Message_4[]={"*  STM32F4DISCOVERY Blue_board Original  *\n\r"};
//	const char Message_5[]={"*       *\n\r"};
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
////	HAL_UART_Transmit( &TerminalInterface, (uint8_t*)Message_0, Msg_Length,1);

	Msg_Length = sizeof(Message_3);
	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_3, Msg_Length);
////	HAL_UART_Transmit( &TerminalInterface, (uint8_t*)Message_3, Msg_Length,1);

	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	Msg_Length = sprintf( Array_for_Messages,
			  	  	  	  "*  SystemClock = %d MHz                 *\n\r",
						  (uint16_t)(HAL_RCC_GetSysClockFreq()/1000000)
						);
	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Array_for_Messages, Msg_Length);
////	HAL_UART_Transmit( &TerminalInterface, (uint8_t*)Array_for_Messages, Msg_Length,1);

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

/////////////////////////////////////////////////////////////////////

  int16_t Encoder_to_LCD(void)
  {
		encoderStatus = Encoder_Get_Status();

		  switch(encoderStatus) {
		    case Incremented:
		    	currCounter++;
				//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET  );
				sprintf(String_LCD,"%04d",currCounter);
				LCD_SetPos(11, 0);
				HAL_Delay(10);
				LCD_String(String_LCD);HAL_Delay(10);

				L_str=	snprintf(buff, sizeof(buff), "\n\r %04d ", currCounter);
				while(TerminalInterface.gState != HAL_UART_STATE_READY ){;}
				HAL_UART_Transmit_IT(&TerminalInterface, (uint8_t*)buff, L_str);
				//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET  );
				HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET  );
		      break;
		    case Decremented:
		    	currCounter--;
				//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET  );
				sprintf(String_LCD,"%04d",currCounter);
				LCD_SetPos(11,0);HAL_Delay(10);

				LCD_String(String_LCD);HAL_Delay(10);

				L_str=	snprintf(buff, sizeof(buff), "\n\r %04d ", currCounter);
				while(TerminalInterface.gState != HAL_UART_STATE_READY ){;}
				HAL_UART_Transmit_IT(&TerminalInterface, (uint8_t*)buff, L_str);
				//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET  );
				HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET  );
		      break;

		    case Neutral:

		    	break;
		    default: break;
		  }///switch(encoderStatus)

		    	if(HAL_GetTick()-Ticks_1 >1500){
											//Ticks_1=HAL_GetTick();
											//Get_Time_to_LCD(0,1);
		    								}

		    	// RTC_update_and_Terminal(1999);
		    	return (currCounter);
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
		  	  	  "  *  CANopenNodeSTM32 canOpenNodeSTM32;      *\n\r"
		  	  	  "  *  .CANHandle = &hcan1;                    *\n\r"
		  	  	  "  *  .HWInitFunction = MX_CAN1_Init          *\n\r"
		  	  	  "  *  .timerHandle = &htim4                   *\n\r"
		  	  	  "  *  .baudrate = 500kbps                     *\n\r"
	  	  	  	  "  *  .desiredNodeID = 0x%02X;                *\n\r\n\r"
	  	  	  	  "  *  .activeNodeID = 0x%02X;                *\n\r\n\r"
		  	  	  "  *  canopen_app_init(&canOpenNodeSTM32);* \n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r",
  	  	  	  	  	  	  	  canOpenNodeSTM32.desiredNodeID,
  	  	  	  	  	  	  	  canOpenNodeSTM32.activeNodeID
			   	   	   	   	   );
		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Msg_2_Terminal, Lngth_of_Message);
		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
		HAL_Delay(50);

}// void CO_Init_Return_State


  void TPDO_send(uint32_t Period_ms) {
		  if(HAL_GetTick() - Ticks>Period_ms)
		  {
			Ticks = HAL_GetTick();
			CO_TPDOsendRequest(&canOpenNodeSTM32.canOpenStack->TPDO[0] );
		  }
   }




  ///////////////////////////////////////////////////////////////////////////////
  void SDO_abortCode_ASCII_to_Terminal(void)
  {
  char Message_2_Terminal_0[128];
  uint16_t Message_Length;
  while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
  Message_Length = SDO_abortCode_to_String(Code_return_SDO,  Message_2_Terminal_0);
  HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_2_Terminal_0, Message_Length);
  while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
  }

  ///////////////////////////////////////////////////////////////////////////////

  CO_SDO_abortCode_t SDO_Read_Write_Read(void)
  {

  	Code_return_SDO = read_SDO (
  			    canOpenNodeSTM32.canOpenStack->SDOclient,
  			  	0x3d,										//remote desiredNodeID Upper_F407XX
  				0x6004,										//Index_of_OD_variable_at_remote_NodeID_6004_u64
  				0x00,										//Sub_Index_of_OD_variable
  				Rx_Array,									//Save_Received_Data_to Local_Array
  				8,											//Number_of_Bytes_to_read
  				(size_t*)&Length_of_Ext_Var ); HAL_Delay(100);


  #if 1

  		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
  		Length_of_Message = sprintf( Message_to_Terminal,
  		  	  	                     "\r EXECUTED read_SDO(...); for the first time\n\r "
                    "by canOpenNodeSTM32.canOpenStack->SDOclient\n\r");

  		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message);HAL_Delay(10);

  		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
  		Length_of_Message = sprintf( Message_to_Terminal_3,
                  "from RemoteNode=0x3d OD_Index=0x6004_u64 SubIndex=0x0\n\r and Save to \n\r"
  				"Rx_Array={0x%X,0x%X,0x%X,0x%X,0x%X,0x%X,0x%X,0x%X} \n\r",
  				Rx_Array[0],Rx_Array[1],Rx_Array[2],Rx_Array[3],
  				Rx_Array[4],Rx_Array[5],Rx_Array[6],Rx_Array[7]);
  		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal_3, Length_of_Message);
  		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
  		SDO_abortCode_ASCII_to_Terminal();		HAL_Delay(10);
  #endif


  #if 2
  		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
  		Length_of_Message = sprintf( Message_to_Terminal_2, "   \n\r Run write_SDO(.....);\n\r"
  														  " write_SDO  by "
                    	  	  	  	  	  	  	  	  	  	  "canOpenNodeSTM32.canOpenStack->SDOclient\n\r");
  		Message_to_Terminal_2[0]=0x08;
  		Message_to_Terminal_2[1]=0x08;
  		Message_to_Terminal_2[2]=0x08;
  		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal_2, Length_of_Message);HAL_Delay(10);

  		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
  		Length_of_Message = sprintf( Message_to_Terminal_1,
                  "Get Data \n\r from Local\n\r Array_8u={0x%X,0x%X,0x%X,0x%X,0x%X,0x%X,0x%X,0x%X} \n\r",
  				Array_8u[0],Array_8u[1],Array_8u[2],Array_8u[3],
  				Array_8u[4],Array_8u[5],Array_8u[6],Array_8u[7]);
  		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal_1, Length_of_Message);
  		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
  		HAL_Delay(10);

  		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
  		Length_of_Message = sprintf( Message_to_Terminal_2,
                  "and \n\r write to\n\r"
                  "OD_Index=0x6004 SubIndex=0x0E  @ Remote Node_0x3d\n\r");
  		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal_2, Length_of_Message);
  		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
  		HAL_Delay(10);

  #endif//2

  Code_return_SDO = write_SDO(
  				canOpenNodeSTM32.canOpenStack->SDOclient,
  				0x3d,										//remote desiredNodeID Upper_F407XX
  				0x6004,										//Index_of_OD_variable_at_remote_NodeID_6004_u64
  				0x00,										//Sub_Index_of_OD_variable
  				Array_8u,									//Source_of_data
  				4);
  HAL_Delay(50);

  while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
  SDO_abortCode_ASCII_to_Terminal();

  Code_return_SDO = read_SDO (
    			    canOpenNodeSTM32.canOpenStack->SDOclient,
  				0x3d,										//remote desiredNodeID Upper_F407XX
  				0x6004,										//Index_of_OD_variable_at_remote_NodeID_6004_u64
    				0x00,										//Sub_Index_of_OD_variable
    				Rx_Array,									//Saved_Received_Data
    				4,											//Number_of_Byte_to_read
    				(size_t*)&Length_of_Ext_Var );HAL_Delay(50);


  #if 3

  		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
  		Length_of_Message = sprintf( Message_to_Terminal_2,
  		  	  	                     "   \n\r EXECUTED read_SDO(...); for the SECOND time\n\r");
  		Message_to_Terminal_2[0]=0x08;
  		Message_to_Terminal_2[1]=0x08;
  		Message_to_Terminal_2[2]=0x08;
  		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal_2, Length_of_Message);

  		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
  		Length_of_Message = sprintf( Message_to_Terminal_1,
                  " Read NEW DATA from Node_0x3d OD_Index=0x6004 SubIndex=0x0E\n\r"
                  "and\n\r Save to\n\r");
  		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal_1, Length_of_Message);

  		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
  		Length_of_Message = sprintf( Message_to_Terminal,
                  "Rx_Array={0x%X,0x%X,0x%X,0x%X,0x%X,0x%X,0x%X,0x%X} \n\r",
  				Array_8u[0],Array_8u[1],Array_8u[2],Array_8u[3],
  				Array_8u[4],Array_8u[5],Array_8u[6],Array_8u[7]);
  		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message); HAL_Delay(10);

  SDO_abortCode_ASCII_to_Terminal();
   HAL_Delay(50);
  #endif//3
   return (Code_return_SDO);
  }
  /////////////////////////////////////////////////////////////////////
  void SDO_Test()
  {
	  Code_return_SDO = read_SDO (
	  			    canOpenNodeSTM32.canOpenStack->SDOclient,
	  				CO_Disco407_Blue,							//remote_NodeID_0x3b
	  				0x600E,										//Index_of_OD_variable_Disco_Blue_VAR64_600e_TX_at_remote_Node
	  				0,											//Sub_Index_of_OD_variable
	  				Rx_Array,									//Saved_Received_Data
	  				4,											//Number_of_Byte_to_read
	  				(size_t*)&Length_of_Ext_Var );

	  #if 1
	  	while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	  	Length_of_Message = SDO_abortCode_to_String(Code_return_SDO,  Message_to_Terminal);
	  	HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message);HAL_Delay(10);

	  		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	  		Length_of_Message = sprintf( Message_to_Terminal,
	  		  	  	                     " execute read_SDO(...); for the first time\n\r "
	                    "by canOpenNodeSTM32.canOpenStack->SDOclient\n\r");

	  		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message);HAL_Delay(10);

	  		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	  		Length_of_Message = sprintf( Message_to_Terminal,
	                  "from RemoteNode=0x3b OD_Index=0x600E SubIndex=0x00\n\r"
	                  "and Save to \n\r"
	                  "Rx_Array={0x%X,0x%X,0x%X,0x%X} \n\r",
	  				Rx_Array[0],Rx_Array[1],Rx_Array[2],Rx_Array[3]);
	  		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message); HAL_Delay(10);
	  #endif
	  HAL_Delay(10);

	  #if 2
	  		Length_of_Message = sprintf( Message_to_Terminal, "\n\r write_SDO(.....);\n\r"
	  														  " write_SDO  by "
	                    	  	  	  	  	  	  	  	  	  	  "canOpenNodeSTM32.canOpenStack->SDOclient\n\r");
	  		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	  		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message);HAL_Delay(10);


	  		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	  		Length_of_Message = sprintf( Message_to_Terminal,
	                  "Get Data from Local\n\r"
	                  "Array_8u={0x%X,0x%X,0x%X,0x%X} \n\r",
	  				Array_8u[0],Array_8u[1],Array_8u[2],Array_8u[3]);
	  		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message); HAL_Delay(10);



	  		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	  		Length_of_Message = sprintf( Message_to_Terminal,
	                  "and write to\n\r"
	                  "OD_Index=0x600e SubIndex=0x0  @ Remote Node_0x3b\n\r");
	  		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message);HAL_Delay(10);
	  #endif//2
	  Code_return_SDO = write_SDO(
	  					canOpenNodeSTM32.canOpenStack->SDOclient,
	  					CO_Disco407_Blue,							//remote_NodeID_0x3b
	  					0x600E,										//Index_of_OD_variable_Disco_Blue_VAR64_600e_TX_at_remote_Node
	  						0,										//Sub_Index_of_OD_variable
	  					Array_8u,									//Data_Source
	  					4);	  HAL_Delay(100);
	  while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	  Length_of_Message = SDO_abortCode_to_String(Code_return_SDO,  Message_to_Terminal);
	  HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message);HAL_Delay(10);


	  Code_return_SDO = read_SDO (
	  					canOpenNodeSTM32.canOpenStack->SDOclient,
	  					CO_Disco407_Blue,							//remote_NodeID_0x3b
	  					0x600E,										//Index_of_OD_variable_Disco_Blue_VAR64_600e_TX_at_remote_Node
	  					0,											//Sub_Index_of_OD_variable
	  					Rx_Array,									//Saved_Received_Data
	  					4,											//Number_of_Byte_to_read
	  					(size_t*)&Length_of_Ext_Var );  HAL_Delay(10);

	  #if 3
	  		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	  		Length_of_Message = SDO_abortCode_to_String(Code_return_SDO,  Message_to_Terminal);
	  		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message);HAL_Delay(10);

	  		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	  		Length_of_Message = sprintf( Message_to_Terminal,
	  		  	  	                     "\n\r EXECUTE read_SDO(...); for the SECOND time\n\r");
	  		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message);

	  		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	  		Length_of_Message = sprintf( Message_to_Terminal,
	                  " Read NEW DATA from Node_0x3b OD_Index=0x600e SubIndex=0x00\n\r"
	                  "and Save to\n\r");
	  		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message);

	  		while(TerminalInterface.gState != HAL_UART_STATE_READY){;}
	  		Length_of_Message = sprintf( Message_to_Terminal,
	                  					"Rx_Array={0x%X,0x%X,0x%X,0x%X} \n\r\n\r",
	  									Array_8u[0],Array_8u[1],Array_8u[2],Array_8u[3]);
	  		HAL_UART_Transmit_DMA( &TerminalInterface, (uint8_t*)Message_to_Terminal, Length_of_Message); HAL_Delay(10);

	  #endif//3

  }


  /////////////////////////////////////////////////////////////////////



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
