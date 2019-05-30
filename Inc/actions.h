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
 #include "stm32f2xx.h"
 #include "stm32f2xx_hal.h"
 #include "audio_conf.h"

 extern char serialTX[100][16];
 extern uint8_t currentPort;

 void getActions(int event);
 void performAction(int action);


 #endif
