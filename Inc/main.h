/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

extern void getButtonEvent();
extern void getSerialEvent(uint8_t port, uint8_t uart_command[10]);
extern void performAction(int action);



/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IND_R_Pin GPIO_PIN_13
#define IND_R_GPIO_Port GPIOC
#define A1_Pin GPIO_PIN_0
#define A1_GPIO_Port GPIOC
#define A2_Pin GPIO_PIN_1
#define A2_GPIO_Port GPIOC
#define A3_Pin GPIO_PIN_2
#define A3_GPIO_Port GPIOC
#define A4_Pin GPIO_PIN_3
#define A4_GPIO_Port GPIOC
#define A5_Pin GPIO_PIN_0
#define A5_GPIO_Port GPIOA
#define A6_Pin GPIO_PIN_1
#define A6_GPIO_Port GPIOA
#define NSS_Pin GPIO_PIN_4
#define NSS_GPIO_Port GPIOA
#define BTN1_Pin GPIO_PIN_4
#define BTN1_GPIO_Port GPIOC
#define BTN2_Pin GPIO_PIN_5
#define BTN2_GPIO_Port GPIOC
#define BTN3_Pin GPIO_PIN_0
#define BTN3_GPIO_Port GPIOB
#define D8_Pin GPIO_PIN_1
#define D8_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_2
#define D7_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_12
#define D1_GPIO_Port GPIOB
#define D2_Pin GPIO_PIN_13
#define D2_GPIO_Port GPIOB
#define D3_Pin GPIO_PIN_14
#define D3_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_15
#define D4_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_8
#define D5_GPIO_Port GPIOC
#define D6_Pin GPIO_PIN_9
#define D6_GPIO_Port GPIOC
#define IND_G_Pin GPIO_PIN_8
#define IND_G_GPIO_Port GPIOA
#define RLY_Pin GPIO_PIN_15
#define RLY_GPIO_Port GPIOA
#define D9_Pin GPIO_PIN_3
#define D9_GPIO_Port GPIOB
#define D10_Pin GPIO_PIN_4
#define D10_GPIO_Port GPIOB
#define D11_Pin GPIO_PIN_5
#define D11_GPIO_Port GPIOB
#define D12_Pin GPIO_PIN_8
#define D12_GPIO_Port GPIOB
#define IND_B_Pin GPIO_PIN_9
#define IND_B_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define uart_command_length 20
#define uart_recv_length  1

//#define PUTCHAR_PROTOTYPE
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
