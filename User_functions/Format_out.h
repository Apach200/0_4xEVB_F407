/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Format_out.h
  * @brief          : Header for Format_out.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FORMAT_OUT
#define __FORMAT_OUT

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
uint32_t* pMessage SDO_abortCode_to_String(CO_SDO_abortCode_t Code);

/* Private defines -----------------------------------------------------------*/
//#define PC14_OSC32_IN_Pin GPIO_PIN_14

#ifdef __cplusplus
}
#endif

#endif /* __FORMAT_OUT */
