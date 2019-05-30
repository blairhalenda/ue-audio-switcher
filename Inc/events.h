/**
  ******************************************************************************
  * @file           : actions.h
  * @version        : v1.0_Cube
  * @brief          : Header for audio_conf.c file.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AUDIO_CONF__H__
#define __AUDIO_CONF__H__

#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ------------------------------------------------------------------*/
 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include "main.h"
#include "button_conf.h"
#include "actions.h"
 #include "stm32f2xx.h"
 #include "stm32f2xx_hal.h"

#define audio1 0x28
#define audio2 0x29
#define audio3 0x2A
#define audio4 0x2B
#define audio5 0x2C
#define audio6 0x2D

 extern uint32_t btnVal[3];
 extern int systemLogic[300][10];
 extern char serialRX[100][uart_command_length];
 extern void performAction(int action);


 void getButtonEvent();
 void getSerialEvent(uint8_t port, uint8_t uart_command[10]);


 #endif
