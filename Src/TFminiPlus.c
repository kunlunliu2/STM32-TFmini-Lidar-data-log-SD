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

#include "TFminiPlus.h"
#include <stdio.h>
#include <string.h>

void TFminiinit(TFminiPlus_CONFIG *TFminiPlus)
{
	//Set the frame rate  to 100Hz

	TFminiPlus->NextMemPoint1 = USART_BUF_SIZE;
	TFminiPlus->CurMemPoint1 = 0;
	TFminiPlus->Buffersets = SET;
	TFminiPlus->Uart2Ready = RESET;

	TFminiPlus->bufferPoint = 0;
	TFminiPlus->num = 0;
}

void TFminiConfig(TFminiPlus_CONFIG *TFminiPlus)
{
	uint8_t buffin[6];
	uint8_t buffout[6]=samp500;
	uint8_t n1 = sizeof(buffout);

	buffout[5] = buffout[0] + buffout[1] + buffout[2] + buffout[3] + buffout[4];

	while(HAL_UART_Transmit(&TFminiUart, (uint8_t *) buffout, n1, 100)!=HAL_OK);
	HAL_UART_Receive(&TFminiUart, (uint8_t *) buffin, sizeof(buffin), 100);

	int num = 0;
	for (int i = 0; i < 6; i++){
		if(buffin[i] == buffout[i]) num++;
	}
	if ( num > 3){
		sprintf(TFminiPlus->msg, "sample rate is correct! \r\n");
		HAL_UART_Transmit(&TFminiUart, (uint8_t *) TFminiPlus->msg, sizeof(TFminiPlus->msg), 100);
	}
	TFminiSaveConfig(TFminiPlus);
}


void TFminiReset(TFminiPlus_CONFIG *TFminiPlus)
{
	uint8_t buffout[4]=factoryReset;
	uint8_t n1 = sizeof(buffout);

	while(HAL_UART_Transmit(&TFminiUart, (uint8_t *) buffout, n1, 100)!=HAL_OK);

}

void TFminiSaveConfig(TFminiPlus_CONFIG *TFminiPlus)
{
	uint8_t buffout1[4]=saveConfig;
	uint8_t n1 = sizeof(buffout1);

	while(HAL_UART_Transmit(&TFminiUart, (uint8_t *) buffout1, n1, 100)!=HAL_OK);

}


void TFminiCallback(TFminiPlus_CONFIG *TFminiPlus)
{
	int i;

	 __IO uint32_t tmp = TFminiUartRx.Instance->CNDTR;
	 int len1 = TFMINI_DATA_Len-tmp;

		 /* TFmini data stream contains 9 bytes starting with two TFMINT_DATA_HEAD 0x59,
		  * The following is used to find the point of TFMINT_DATA_HEAD in buff, named NextMemPoint1.
		  */

	for(i=0; i< len1-1; i++){
		if(TFminiPlus->g_usart1_rx_buf[i] == TFMINT_DATA_HEAD && TFminiPlus->g_usart1_rx_buf[i+1] == TFMINT_DATA_HEAD){
			TFminiPlus->NextMemPoint1 = i;
			 break;
		 }
	 }

		 /* The received data stream g_usart1_rx_buf through Uart_Receive_DMA receive, will be stored in a buffer block,
		  * which is double the size of g_usart1_rx_buf. The buff is decomposed into two blocks. Each of them will store
		  * the current receiver or previous one alternatively.
		  *
		  */

	 if(TFminiPlus->Buffersets == SET){
		 memcpy(TFminiPlus->buff, TFminiPlus->g_usart1_rx_buf, len1);
	 }else{
		 memcpy(TFminiPlus->buff+TFMINI_DATA_Len, TFminiPlus->g_usart1_rx_buf, len1);
		 TFminiPlus->NextMemPoint1 = TFminiPlus->NextMemPoint1 + TFMINI_DATA_Len;
	 }

	 if(TFminiPlus->NextMemPoint1>USART_BUF_SIZE-1 ){
		 TFminiPlus->Buffersets = SET;
		 TFminiPlus->NextMemPoint1 = 0;
	 }else if(TFminiPlus->NextMemPoint1 == 0 || TFminiPlus->NextMemPoint1 == TFMINI_DATA_Len ||
			 TFminiPlus->NextMemPoint1==TFminiPlus->CurMemPoint1+TFMINI_DATA_Len ||
			 TFminiPlus->CurMemPoint1==TFminiPlus->NextMemPoint1+TFMINI_DATA_Len)
	 {

		 TFmini_RX_Proc( TFminiPlus );
		 TFminiPlus->Buffersets = ~TFminiPlus->Buffersets;
	 }else {
		 TFminiPlus->Buffersets = ~TFminiPlus->Buffersets;
	 }
	 TFminiPlus->CurMemPoint1=TFminiPlus->NextMemPoint1;

}
/* Private user code ---------------------------------------------------------*/


void TFmini_RX_Proc(TFminiPlus_CONFIG *TFminiPlus)
{
	uint32_t i = 0;
	uint32_t j = 0;
	uint32_t i1 = 0;
	uint32_t buflen = USART_BUF_SIZE;
	uint8_t data[2];

	uint8_t chk_cal = 0;


	for(i = 0; i < (TFMINI_DATA_Len - 1); i++)
	{
		j = TFminiPlus->CurMemPoint1 + i;
		if(j > buflen) j = j - buflen;
		chk_cal += TFminiPlus->buff[j];
		if(i == 2 || i == 4) i1 = j;
		if(i == 3) data[0] = TFminiPlus->buff[i1] | (TFminiPlus->buff[j] << 8);
		if(i == 5) data[1] = TFminiPlus->buff[i1] | (TFminiPlus->buff[j] << 8);
	}

	j = TFminiPlus->NextMemPoint1 - 1;
	if(j < 0) j = buflen;

	/*
	 * If the sum of the first n-1 data equals to the last one, then this block of data is eligible to be used.
	 * The last data is buff(NextMemPoint1 - 1). The sum of first n-1 is chk_cal
	 */
	if(chk_cal == TFminiPlus->buff[j])
	{

		if(data[0] <= TFMINI_ACTION_DIST)
		{

			sprintf(TFminiPlus->msg, "%3d,T=%7ld,D=%4d,I=%4d\r\n", TFminiPlus->num, HAL_GetTick(), data[0], data[1]);
			i1 = strlen(TFminiPlus->msg);
			TFminiPlus->num = TFminiPlus->num + 1;
			if(TFminiPlus->num>99)TFminiPlus->num = 0;
			memcpy(TFminiPlus->membuffer+TFminiPlus->bufferPoint, TFminiPlus->msg, i1);
			TFminiPlus->bufferPoint = TFminiPlus->bufferPoint + i1;
			if(TFminiPlus->bufferPoint > buffmax - msglen){
				TFminiPlus->Uart2Ready = SET;
				TFminiPlus->bufferPoint = 0;
			}

			//copy data to USART2 for screen monitor
			HAL_DMA_Start_IT(&TFminiUart2Tx,  (uint32_t) TFminiPlus->msg,  (uint32_t) &TFminiUart2.Instance->TDR, i1);
			TFminiUart2.Instance->CR3 |= USART_CR3_DMAT;

		}

	}

}
