/**
  ******************************************************************************
  * @file           : audio_conf.h
  * @version        : v1.0_Cube
  * @brief          : Header for audio_conf.c file.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SD_CONF__H__
#define __SD_CONF__H__

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
 #include "fatfs.h"


 // SD Variables
 extern FATFS fs;
 extern FATFS *pfs;
 extern FIL fil;
 extern FRESULT fres;
 extern DWORD fre_clust;
 extern uint32_t total,freespace;
 extern char buffer[100];
 extern volatile uint8_t Timer1, Timer2;

// SD Functions

 void sd_write(uint8_t* string, int length);
 FRESULT sd_append (FIL* fp,const char* path);
 //void sd_read();

 #endif
