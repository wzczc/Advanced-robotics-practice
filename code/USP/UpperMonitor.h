/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    UpperMonitor.c
  * @author  LiangHong Lin(������) 
  * @brief   Code for Upper monitor supported by Mr.Lin in STM32F4.
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have any 
  * bugs, update the version Number, write dowm your name and the date. The most
  * important thing is make sure the users will have clear and definite under-
  * standing through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
#ifndef  _UPPER_MONITOR_H_
#define  _UPPER_MONITOR_H_
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
union type_change               //���ݴ��乲����ṹ
{
  uint8_t   change_u8[4];       // 8λ�޷������͡�1���ֽڡ�
  float     change_float;       //32λ���������ݡ�4���ֽڡ�
  int       change_int;         //32λ�з������͡�4���ֽڡ�
  short int change_u16;         //16λ�з������͡�2���ֽڡ�
};

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function declarations --------------------------------------------*/
void Sent_Contorl(UART_HandleTypeDef* huart_x);
uint32_t RecHandle(uint8_t *data_buf,uint16_t length);

#ifdef __cplusplus
}
#endif
#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

