/**
  ******************************************************************************
  * @file           : button_conf.h
  * @version        : v1.0_Cube
  * @brief          : Header for button_conf.c file.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUTTON_CONF__H__
#define __BUTTON_CONF__H__

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


 // Button Variables
 extern int buttonState;
 extern int currentState;
 extern int buttonPressed;
 extern int lastButtonPressed;
 extern int buttonConfidence;
 extern int buttonThreshold;

 // Button Functions
 //Reads Analog data and reads which button is being pressed
 int readButtons(uint32_t btnVal[3]);

 //Debouncing function that returns button number after a certain count threshold is met
 int getButtonState();


 #endif
