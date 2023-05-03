

#ifndef __MYIIC_H
#define __MYIIC_H

#include "stm32f4xx_hal.h"
#include "delay.h"
//�����Լ�д������ʱ����
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);

#if 1

#define IIC_GPIO_PORT  GPIOB
#define IIC_SCL_PIN  GPIO_PIN_6
#define IIC_SDA_PIN  GPIO_PIN_7
#define IIC_RCC_ENABLE  __HAL_RCC_GPIOB_CLK_ENABLE();


//IO��������
//˵��:0�����룬1�����
#define IIC_SDA_IN()  {IIC_GPIO_PORT->MODER&=~(3<<(7*2));IIC_GPIO_PORT->MODER|=0<<7*2;}	//PB9����ģʽ
#define IIC_SDA_OUT() {IIC_GPIO_PORT->MODER&=~(3<<(7*2));IIC_GPIO_PORT->MODER|=1<<7*2;} //PB9���ģʽ

#define IIC_SCL_H  HAL_GPIO_WritePin(IIC_GPIO_PORT,IIC_SCL_PIN,GPIO_PIN_SET);
#define IIC_SCL_L  HAL_GPIO_WritePin(IIC_GPIO_PORT,IIC_SCL_PIN,GPIO_PIN_RESET);

#define IIC_SDA_H  HAL_GPIO_WritePin(IIC_GPIO_PORT,IIC_SDA_PIN,GPIO_PIN_SET);
#define IIC_SDA_L  HAL_GPIO_WritePin(IIC_GPIO_PORT,IIC_SDA_PIN,GPIO_PIN_RESET);

#define IIC_SDA_READ  HAL_GPIO_ReadPin(IIC_GPIO_PORT, IIC_SDA_PIN)

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(uint8_t txd);			//IIC����һ���ֽ�
//void IIC_Send_Byte(uint8_t _ucByte);			//IIC����һ���ֽ�
uint8_t IIC_Read_Byte(uint8_t ack);//IIC��ȡһ���ֽ�
uint8_t IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	 
#endif

#endif

#ifndef _IIC_H
#define _IIC_H

#include <inttypes.h>

#define IIC_WR	0		/* д����bit */
#define IIC_RD	1		/* ������bit */

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
