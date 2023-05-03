

#ifndef __MYIIC_H
#define __MYIIC_H

#include "stm32f4xx_hal.h"
#include "delay.h"
//以下自己写两个延时函数
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);

#if 1

#define IIC_GPIO_PORT  GPIOB
#define IIC_SCL_PIN  GPIO_PIN_6
#define IIC_SDA_PIN  GPIO_PIN_7
#define IIC_RCC_ENABLE  __HAL_RCC_GPIOB_CLK_ENABLE();


//IO方向设置
//说明:0是输入，1是输出
#define IIC_SDA_IN()  {IIC_GPIO_PORT->MODER&=~(3<<(7*2));IIC_GPIO_PORT->MODER|=0<<7*2;}	//PB9输入模式
#define IIC_SDA_OUT() {IIC_GPIO_PORT->MODER&=~(3<<(7*2));IIC_GPIO_PORT->MODER|=1<<7*2;} //PB9输出模式

#define IIC_SCL_H  HAL_GPIO_WritePin(IIC_GPIO_PORT,IIC_SCL_PIN,GPIO_PIN_SET);
#define IIC_SCL_L  HAL_GPIO_WritePin(IIC_GPIO_PORT,IIC_SCL_PIN,GPIO_PIN_RESET);

#define IIC_SDA_H  HAL_GPIO_WritePin(IIC_GPIO_PORT,IIC_SDA_PIN,GPIO_PIN_SET);
#define IIC_SDA_L  HAL_GPIO_WritePin(IIC_GPIO_PORT,IIC_SDA_PIN,GPIO_PIN_RESET);

#define IIC_SDA_READ  HAL_GPIO_ReadPin(IIC_GPIO_PORT, IIC_SDA_PIN)

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
//void IIC_Send_Byte(uint8_t _ucByte);			//IIC发送一个字节
uint8_t IIC_Read_Byte(uint8_t ack);//IIC读取一个字节
uint8_t IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	 
#endif

#endif

#ifndef _IIC_H
#define _IIC_H

#include <inttypes.h>

#define IIC_WR	0		/* 写控制bit */
#define IIC_RD	1		/* 读控制bit */

void IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Byte(uint8_t _ucByte);
uint8_t IIC_Read_Byte(uint8_t ack);
uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
uint8_t IIC_CheckDevice(uint8_t _Address);
void IIC_Init(void);

#endif
