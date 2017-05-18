/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "system_stm32f4xx.h"
#include "can.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "i2c.h"
#include "stm32f4xx_it.h"
#include "..\source\UAVCAN\uavcan.h"
#include "..\source\ADIS16488A\ADIS16488A.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
typedef struct
{
   uint16_t  Magn_x;
	 uint16_t  Magn_y;
	 uint16_t  Magn_z;
	 uint16_t  Accl_x;
	 uint16_t  Accl_y;
	 uint16_t  Accl_z;
	 uint16_t  Gyro_x;
	 uint16_t  Gyro_y;
	 uint16_t  Gyro_z;
	 uint32_t  Quat_x;
	 uint32_t  Quat_y;
	 uint32_t  Quat_z;
	 uint32_t  Quat_w;
} Outdata;
	

union unionLong
{
  int32_t  tmp;
  uint8_t arry[4];
};

union unionLong2
{
  float  tmp;
  uint8_t arry[4];
};
union unionLong3
{
  float    tmp;
	uint32_t tmp1;
};

 SPI_TypeDef  spi1;
 Outdata  Outdatas;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void Delay_n(void);
void Delay_ms(void);
void Delay_us(uint32_t time);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int GAValid, BARValid, MAGNValid;
int DELTANGValid,DELTVELValid; 

/* USER CODE END 0 */

int main(void)
{

  /* MCU Configuration----------------------------------------------------------*/

  /* Configure the system clock */
  SystemClock_Config();

 /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
	MX_I2C_Init(); 
	Delay_n();
	PIN_Init();
  //MX_SPI1_Init();
  
	
  Delay_n();
	Delay_n();
	Delay_n();
	Delay_n();
	//PIN_Init();
	MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //adis16480_initial_setup();
	//uint32_t i;
	//TestCAN();
  /* USER CODE END 2 */
//  uint32_t gyro_id = 0x00000080; 
//	uint32_t accl_id = 0x00000081;
//	uint32_t magn_id = 0x00000082;
//	uint32_t Quat_id = 0x00000083;
	uint16_t Magndata_x,Magndata_y,Magndata_z;
  uint16_t Accldata_x,Accldata_y,Accldata_z;
	uint16_t Gyrodata_x,Gyrodata_y,Gyrodata_z;
	float Quardata_X,Quardata_Y,Quardata_Z,Quardata_W;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
   /*	
   Magndata_x = PNI_Readtwobyte(Magn_X,2);
	 Magndata_y = PNI_Readtwobyte(Magn_Y,2);
   Magndata_z = PNI_Readtwobyte(Magn_Z,2);		
	 TestCAN(magn_id,Magndata_x,Magndata_y,Magndata_z);
	 Delay_n();
   Accldata_x = PNI_Readtwobyte(Accl_X,2);
   Accldata_y = PNI_Readtwobyte(Accl_Y,2); 
   Accldata_z = PNI_Readtwobyte(Accl_Z,2); 	
   TestCAN(accl_id,Accldata_x,Accldata_y,Accldata_z);		
	 Delay_n();
	 Gyrodata_x = PNI_Readtwobyte(Velo_X,2); 	
   Gyrodata_y = PNI_Readtwobyte(Velo_Y,2);
 	 Gyrodata_z = PNI_Readtwobyte(Velo_Z,2);
	 TestCAN(gyro_id,Gyrodata_x,Gyrodata_y,Gyrodata_z);	
   Delay_n();
	 Outdatas.Quat_x = PNI_ReadLenByte(Quatern_X,4);
	 Outdatas.Quat_y = PNI_ReadLenByte(Quarern_Y,4);
	 Outdatas.Quat_z = PNI_ReadLenByte(Quarern_Z,4);
	 Outdatas.Quat_w = PNI_ReadLenByte(Quarern_W,4);
	 union unionLong3  int32tofloat;
	 int32tofloat.tmp1 = Outdatas.Quat_x;
	 Quardata_X = int32tofloat.tmp;
	 int32tofloat.tmp1 = Outdatas.Quat_y;
	 Quardata_Y = int32tofloat.tmp;
	 int32tofloat.tmp1 = Outdatas.Quat_z;
	 Quardata_Z = int32tofloat.tmp;
	 int32tofloat.tmp1 = Outdatas.Quat_w;
	 Quardata_W = int32tofloat.tmp;
	 Delay_n();
	 */
	 //TestCAN(Quat_id,Outdatas.Quat_x,Outdatas.Quat_y,Outdatas.Quat_z);
	}

}



void  Delay_n(void)
{
  int i,j;
	for(i=0;i<5000;i++)
	{
		for(j=0;j<2000;j++)
		{
		}
	}
}
void Delay_ms(void)
{
	if (SysTick_Config(SystemCoreClock / 1000))
  {
    while (1);
  }
}

void Delay_us(uint32_t time)
{   
	  uint32_t TimingDelay;
    if(SysTick_Config(SystemCoreClock/1000000))
    {
      while(1);
    } 
   TimingDelay = time; 
   while(TimingDelay != 0);
}
/** System Clock Configuration 
*/ 
void SystemClock_Config(void) 
{ 
    
 ErrorStatus HSEStartUpStatus; 
 uint32_t        PLL_M = 24;       
 uint32_t        PLL_N = 336; 
 uint32_t        PLL_P = 2; 
 uint32_t        PLL_Q = 7; 
 RCC_DeInit(); 
 RCC_HSEConfig(RCC_HSE_ON);                                     
 HSEStartUpStatus = RCC_WaitForHSEStartUp(); 
 if(HSEStartUpStatus == SUCCESS) 
 { 
    
    RCC_HCLKConfig(RCC_SYSCLK_Div1);                             
    RCC_PCLK2Config(RCC_HCLK_Div2); 
    RCC_PCLK1Config(RCC_HCLK_Div4); 
    
    FLASH_SetLatency(FLASH_Latency_5);           
    FLASH_PrefetchBufferCmd(ENABLE); 
    
    RCC_PLLConfig(RCC_PLLSource_HSE, PLL_M, PLL_N, PLL_P, PLL_Q);     
    RCC_PLLCmd(ENABLE); 
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)   
   {   
    
   }     
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); 
                                            
 } 

}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}
/* 
* ADIS16488A中断回调函数，读GYRO/ACCL/MAGN/DELTANG/DELVAL数据 
* 读状态寄存器，根据状态寄存器指示读取MAGN/BAROM
*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	unsigned int sta;
	
	readgyro();
	readaccl();
  readdeltang();
  readdeltvel();
	
	GAValid = 0xff;
	sta = adis16480_StatusReg();
	
	if(sta & 0x0200)
	{
	  readbarom(); 
		BARValid = 0xff;
	}
	
	if(sta & 0x0100)
	{
	  readmagn(); 
	  MAGNValid = 0xff;
	}
}	

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
