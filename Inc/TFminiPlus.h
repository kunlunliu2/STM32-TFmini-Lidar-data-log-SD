/*
library name: 	TFmini plus Lidar module
written by: 	Kunlun Liu
Date Written: 	Dec 12 2021
Last Modified: 	Jan 26 2022 by Kunlun Liu
Description: 	A data log code for determining the distance finded by TFmini plus lidar module. The data logs using FAT SD module by JIHOON LEE.
References:
				- JIHOON LEE FatFS library: https://github.com/eziya/STM32_SPI_SDCARD

* Copyright (C) 2022 - Kunlun Liu
   This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
   of the GNU General Public License as published by the Free Software Foundation.

   This software library is shared with puplic for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
   or indirectly by this software, read more about this on the GNU General Public License.

*/

#ifndef TFMINIPLUS_H_
#define TFMINIPLUS_H_

#ifndef __TFMINIPLUS_H
#define __TFMINIPLUS_H

#include "main.h"
#include "fatfs.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

#define TFMINI_DATA_Len 9
#define USART_BUF_SIZE 18			// USART_BUF_SIZE is doube the size of TFMINI_DATA_Len
#define TFMINT_DATA_HEAD 0x59
#define TFMINI_ACTION_DIST 1200
#define TFminiUart huart1
#define TFminiUart2 huart2
#define TFminiUartRx hdma_usart1_rx
#define TFminiUart2Tx hdma_usart2_tx
#define buffmax 1024
#define msglen 30
#define samp1  {0x5a, 0x06, 0x03, 0x01, 0x00, 0x64}
#define samp10  {0x5a, 0x06, 0x03, 0x0A, 0x00, 0x6D}
#define samp100  {0x5a, 0x06, 0x03, 0x64, 0x00, 0xC7}
#define samp500  {0x5a, 0x06, 0x03, 0xF4, 0x01, 0xF3}
#define samp1000  {0x5a, 0x06, 0x03, 0xE8, 0x03, 0x4E}
#define factoryReset  {0x5a, 0x04, 0x10, 0x6E}
#define saveConfig {0x5a,0x04,0x11,0x6f}

typedef struct {

	uint16_t NextMemPoint1;
	uint16_t CurMemPoint1;

    uint8_t g_usart1_rx_buf[TFMINI_DATA_Len];
    uint8_t USART_BUF[20];
    uint8_t buff[USART_BUF_SIZE];

    __IO ITStatus Buffersets;
    __IO ITStatus Uart2Ready;

    char msg[msglen];
    char membuffer[buffmax];
	uint16_t bufferPoint;
	uint8_t num;

} TFminiPlus_CONFIG;

void TFminiCallback(TFminiPlus_CONFIG *TFminiPlus);
void TFmini_RX_Proc(TFminiPlus_CONFIG *TFminiPlus);
void TFminiConfig(TFminiPlus_CONFIG *TFminiPlus);
void TFminiReset(TFminiPlus_CONFIG *TFminiPlus);
void TFminiinit(TFminiPlus_CONFIG *TFminiPlus);
void TFminiSaveConfig(TFminiPlus_CONFIG *TFminiPlus);

#endif

#endif
