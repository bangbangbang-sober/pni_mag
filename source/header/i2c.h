/**
  ******************************************************************************
  * File Name          : i2c.h
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  */
#ifndef __i2c_H
#define __i2c_H
	
	/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

#define  IIC_SCL_H    GPIO_SetBits(GPIOB,GPIO_Pin_6)
#define  IIC_SCL_L    GPIO_ResetBits(GPIOB,GPIO_Pin_6)
#define  IIC_SDA_H    GPIO_SetBits(GPIOB,GPIO_Pin_7)
#define  IIC_SDA_L    GPIO_ResetBits(GPIOB,GPIO_Pin_7)
#define  IIC_SDA_Horl GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)

/*Define  PNI  data  register  address------------------------------------------------------*/
#define  Quatern_X  			   0x00    //float32
#define  Quarern_Y  				 0x04
#define  Quarern_Z 				   0x08
#define  Quarern_W 				   0x0C
#define  Magn_X    				   0x12    //int16
#define  Magn_Y    				   0x14
#define  Magn_Z   			     0x16
#define  Accl_X   				   0x1A    //int16
#define  Accl_Y   				   0x1C
#define  Accl_Z   				   0x1E
#define  Velo_X   				   0x22    //int16
#define  Velo_Y     				 0x24
#define  Velo_Z              0x26

#define  QRateDivisor        0x32
#define  EnableEvent         0x33
#define  HostControl         0x34
#define  EventStatus         0x35
#define  SensorStatus        0x36
#define  SentralStatus       0x37
#define  AlgorithmStatus     0x38

#define  ErrorRegister       0x50
#define  AlgorithmControl    0x54
#define  MagRate             0x55
#define  AccelRate           0x56
#define  GyroRate            0x57

#define  ResetReq            0x9B
#define  PassThroughStatus   0x9E
#define  PassThroughControl  0xA0


void MX_I2C_Init(void);
void IIC_Start(void);				
void IIC_Stop(void);	  			
void IIC_Send_Byte(u8 txd);			
u8 IIC_Read_Byte(unsigned char ack);
u8 IIC_Wait_Ack(void); 				
void IIC_Ack(void);					
void IIC_NAck(void);				
void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 
void delay_nus(uint8_t times);

uint8_t   PIN_Init(void);
uint8_t   PNI_Writeonebyte(uint8_t WriteAddr, uint8_t AddrData);
uint8_t   PNI_Writenonebyte(uint8_t WriteAddr, uint8_t AddrData[2]);
uint8_t   PNI_Readonebyte(uint8_t ReadAddr);
uint16_t  PNI_Readtwobyte(uint8_t ReadAddr,uint8_t Len);
uint32_t  PNI_ReadLenByte(uint8_t ReadAddr,uint8_t Len);
uint32_t  PNI_ReadQuaternion(uint8_t ReadAddr,uint8_t Len);
#endif
