/**
  ******************************************************************************
  * @file           : audio_conf.h
  * @version        : v1.0_Cube
  * @brief          : Header for audio_conf.c file.
  ******************************************************************************
  */

#define audio1 0x28
#define audio2 0x29
#define audio3 0x2A
#define audio4 0x2B
#define audio5 0x2C
#define audio6 0x2D

#define volatileWiper0     0x00
#define volatileWiper1     0x01
#define nonvolatileWiper0  0x02
#define nonvolatileWiper1  0x03
#define TCON               0x04
#define mcpSTATUS          0x05


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





 // Audio Variables
 extern I2C_HandleTypeDef hi2c1;
 int i;


 // Audio Functions
 void setDigitalPotWiper(uint8_t Address, uint8_t Register, int Value);
 void fadeOut(uint8_t Address, uint8_t Register1, uint8_t Register2);
 void fadeIn(uint8_t Address, uint8_t Register1, uint8_t Register2);
 void mutePorts();
 uint8_t switchPort(uint8_t currentPort, uint8_t Address);


 #endif
