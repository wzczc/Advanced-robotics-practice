#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK NANO STM32F103������
//MPU6050 IIC���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2018/7/28
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2018-2028
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
	   		   
//IO��������
#define MPU_SDA_IN()  {GPIOA->CRL&=0XFFFF0FFF;GPIOA->CRL|=8<<12;}
#define MPU_SDA_OUT() {GPIOA->CRL&=0XFFFF0FFF;GPIOA->CRL|=3<<12;}

//IO��������	 
#define MPU_IIC_SCL    PAout(2) 		//SCL
#define MPU_IIC_SDA    PAout(3) 		//SDA	 
#define MPU_READ_SDA   PAin(3) 		//����SDA 

//IIC���в�������
void MPU_IIC_Delay(void);				//MPU IIC��ʱ����
void MPU_IIC_Init(void);                //��ʼ��IIC��IO��				 
void MPU_IIC_Start(void);				//����IIC��ʼ�ź�
void MPU_IIC_Stop(void);	  			//����IICֹͣ�ź�
void MPU_IIC_Send_Byte(uint8_t txd);			//IIC����һ���ֽ�
uint8_t MPU_IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
uint8_t MPU_IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void MPU_IIC_Ack(void);					//IIC����ACK�ź�
void MPU_IIC_NAck(void);				//IIC������ACK�ź�

void IMPU_IC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t MPU_IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	  
#endif
















