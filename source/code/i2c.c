/**
  ******************************************************************************
  * File Name          : i2c.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances and the configuration of
	*                      the SENtral                       
  ******************************************************************************
  */
	#include "i2c.h"
	
	/** 
  * @brief  I2C  init Configuration 
  */ 
	void MX_I2C_Init(void)
	{
	  GPIO_InitTypeDef GPIO_InitStruture;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
		
		GPIO_InitStruture.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStruture.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStruture.GPIO_OType =GPIO_OType_OD;//GPIO_OType_PP;
		GPIO_InitStruture.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStruture.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOB,&GPIO_InitStruture);	
		IIC_SDA_H;
		IIC_SCL_H;
	}
	
	
/*
	//产生IIC起始信号
*/
void IIC_Start(void)
{

	IIC_SDA_H;	  	  
	IIC_SCL_H;
	delay_nus(4);
 	IIC_SDA_L;      //START:when CLK is high,DATA change form high to low 
	delay_nus(4);
	IIC_SCL_L;      //钳住I2C总线 
}	  
/*
 //产生IIC停止信号
*/
void IIC_Stop(void)
{
	IIC_SCL_L;
	IIC_SDA_L;      //STOP:when CLK is high DATA change form low to high
 	delay_nus(4);
	IIC_SCL_H; 
	IIC_SDA_H;      //发送I2C总线结束信号
	delay_nus(4);							   	
}
/*
* function:  wait Ack
//back:   1,failed
//        0，success
*/
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0; 
	IIC_SDA_H;
	delay_nus(1);	   
	IIC_SCL_H;
	delay_nus(1);	 
	while(IIC_SDA_Horl)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_L;	   
	return 0;  
} 

/*
*function:  IIC  Ack
*
*/
void IIC_Ack(void)
{
	IIC_SCL_L;
	IIC_SDA_L;
	delay_nus(2);
	IIC_SCL_H;
	delay_nus(2);
	IIC_SCL_L;
}
	    
void IIC_NAck(void)
{
	IIC_SCL_L;
	IIC_SDA_H;
	delay_nus(2);
	IIC_SCL_H;
	delay_nus(2);
	IIC_SCL_L;
}					 				 
/*
*  function: IIC send a byte
*1，有应答
*0，无应答			  
*/
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;       
    IIC_SCL_L;
    for(t=0;t<8;t++)
    {              
			if(((txd&0x80)>>7) !=0)
				IIC_SDA_H;
			else
			  IIC_SDA_L;
        txd<<=1; 	  
		delay_nus(2);   
		IIC_SCL_H;
		delay_nus(2); 
		IIC_SCL_L;	
		delay_nus(2);
    }	 
} 

/*
* function:  Read a byte;
*
*/
u8 IIC_Read_Byte(unsigned char ack)
{
	 unsigned char i,receive=0;
    for(i=0;i<8;i++ )
	 {
        IIC_SCL_L; 
        delay_nus(2);
		    IIC_SCL_H;
        receive<<=1;
        if(IIC_SDA_Horl)
					receive++;   
		    delay_nus(1); 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}
/*
*delay nus functin
*/
void delay_nus(uint8_t times)
{
	int16_t i,j;
  for(i=0;i<times;i++)
	{
	  for(j=0;j<168;j++)
		{}
	}	
}
/*
* writing data to registers
*/
uint8_t PNI_Writeonebyte(uint8_t WriteAddr, uint8_t AddrData)
{
  uint8_t temp;
	IIC_Start();
	IIC_Send_Byte(0x50);  //slave address 
	IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);
  IIC_Wait_Ack();
  IIC_Send_Byte(AddrData);
  IIC_Wait_Ack();
  IIC_Stop();
	return 1;
}
/*
* writing two datas to registers
*/
uint8_t PNI_Writenonebyte(uint8_t WriteAddr, uint8_t AddrData[2] )
{
	IIC_Start();
	IIC_Send_Byte(0x50); //slave address
	IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);
  IIC_Wait_Ack();
	IIC_Send_Byte(AddrData[0]);
	IIC_Wait_Ack();
	IIC_Send_Byte(AddrData[1]);
	IIC_Wait_Ack();
	IIC_Stop();
	return 1;
}

/*
* reading one  data from registers
*/
uint8_t PNI_Readonebyte(uint8_t ReadAddr)
{
  uint8_t temp;
  IIC_Start();
  IIC_Send_Byte(0x50);
  IIC_Wait_Ack();
  IIC_Send_Byte(ReadAddr);
  IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(0x51);
	IIC_Wait_Ack();
  temp = IIC_Read_Byte(0);
	IIC_Stop();
	return temp;
}
/*
//function   :Read data 
//ReadAddr   :Start Address
//return     :Data
//Len        : 2,4
*/
uint32_t  PNI_ReadLenByte(uint8_t ReadAddr,uint8_t Len)
{  	
	uint8_t t;
	uint32_t temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=PNI_Readonebyte(ReadAddr+Len-t-1); 	 				   
	}
	return temp;												    
}

uint16_t PNI_Readtwobyte(uint8_t ReadAddr,uint8_t Len)
{
  uint8_t t;
	uint16_t temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=PNI_Readonebyte(ReadAddr+Len-t-1); 	 				   
	}
	return temp;
}

/*
* function : Sentral Init
* send the datas to the register
*/
uint8_t PIN_Init(void)
{
   uint8_t temp;
	 uint8_t res_accl,res_gyro;
	 PNI_Writeonebyte(MagRate,0x0A); 		//频率100Hz
	 PNI_Writeonebyte(AccelRate,0x28);	//400Hz
	 PNI_Writeonebyte(GyroRate,0x28); 	//400Hz
	 PNI_Writeonebyte(QRateDivisor,0x01);//100Hz
	 PNI_Writeonebyte(AlgorithmControl,0x00);//Disable standby status
	 //PNI_Writeonebyte(AlgorithmControl,0x04);//Output Heading,Pitch,and roll 
	 PNI_Writeonebyte(EnableEvent,0x07);//interrupt to the host
	 PNI_Writeonebyte(HostControl,0x01); //run Enable
	 PNI_Writeonebyte(PassThroughControl,0x00);//Disable Pass-Through State
	 res_accl = PNI_Readonebyte(AccelRate);
	 res_gyro = PNI_Readonebyte(GyroRate);
	 temp = PNI_Readonebyte(SentralStatus);// SentralStatus
	 if((temp & 0x02) ==1)
	 {
			PNI_Writeonebyte(ResetReq,0x01);//Emulate a hard power down/power up
	 }
   return 1;
}

/*
* function: Read the Normalized Quaternion 
*  folat32
*/
uint32_t PNI_ReadQuaternion(uint8_t ReadAddr,uint8_t Len)
{
  uint8_t t;
	uint16_t temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=PNI_Readonebyte(ReadAddr+Len-t-1); 	 				   
	}
	return temp;
}
