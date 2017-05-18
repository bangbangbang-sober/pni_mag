/**
  ******************************************************************************
  * File Name          : SPI.c
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "spi.h"
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */



/* SPI1 init function */
void MX_SPI1_Init(void)
{
  SPI_InitTypeDef  hspi1;
  hspi1.SPI_Mode = SPI_Mode_Master;
  hspi1.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  hspi1.SPI_DataSize = SPI_DataSize_16b;
  hspi1.SPI_CPOL = SPI_CPOL_High;
  hspi1.SPI_CPHA = SPI_CPHA_2Edge;
  hspi1.SPI_NSS = SPI_NSS_Hard;//SPI_NSS_Soft;
  hspi1.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  hspi1.SPI_FirstBit = SPI_FirstBit_MSB;
  //hspi1.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1,&hspi1);
	SPI_SSOutputCmd(SPI1, ENABLE);//SPI1_NSS(PA4)
	SPI_Cmd(SPI1,ENABLE);
	
}
/* Read and Write Funtion */
  uint16_t SPI1_ReadWrite16Bit(uint16_t Txdata)
{
	
  while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == RESET){}
	SPI_I2S_SendData(SPI1,Txdata);
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE) == RESET){}
  return SPI_I2S_ReceiveData(SPI1);
	 
}
	
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
