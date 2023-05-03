#include "UpperMonitor.h"
/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    UpperMonitor.c
  * @author  LiangHong Lin(������) 
  * @brief   Code for Upper monitor supported by Mr.Lin in STM32F4.
  * @date    Unkown.
  * @version 1.0
  * @par Change Log��
  * <table
  * <tr><th>Date        <th>Version  <th>Author    		  <th>Description
  * <tr><td>2019-06-12  <td> 1.0     <td>LiangHong Lin  <td>Creator
  * </table>2019-11-06  <td> 1.1     <td>Mentos Seetoo  <td>Add return valie for 
  *                                                         `RecHandle()`
  *
  ==============================================================================
                      ##### How to use this driver #####
  ==============================================================================
    @note
      -# ������ָ������`extern`��Ҫ�۲�����޸ĵı����� \n
      -# ���Ҫ�۲����������ʽ��`UpperMonitor_Sent_Choose()`�� \n
         ���Ҫ�޸ı���������ʽ��`PARAMETER_MODIFICATION()`��
      -# ����`RecHandle()`������λ�����������������ݰ�(ͨ���ڴ����жϺ�����ֱ�ӵ���)
      -# ����`Sent_Contorl()`�������ݸ���λ����
    @warning
      -# �û���Ҫ����Ӳ�������޸�`Sent_Contorl()`�����ڴ��ڷ��͵ĺ�����

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
	
/***********************��λ������ʹ��***********************/
/* ������extern��Ҫʹ�õı�������Ҫ������ͷ�ļ� */


/***********************��λ������ʹ��***********************/

/* Includes ------------------------------------------------------------------*/ 
#include <UpperMonitor.h>
#include "usart.h"
#include "main.h"
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
//extern RC_Ctrl_t RC_CtrlData;
/** 
* @brief ǧ��Ҫ�����±������ݣ�������
*/
#define Sent_Data_Num 9
uint8_t On_Off_flag;
type_change Sent_data_type[Sent_Data_Num+2];              //�������ݹ�����
uint8_t USART0_Sent_Choose_Data[9]={0,1,2,3,4,5,6,7,8};   //����ѡ���͵����ݱ�־

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
void UpperMonitor_Sent_Set(float *data);
void UpperMonitor_Sent_Choose(float * data);
float PARAMETER_Change_float(uint8_t * PARAMETER);
void PARAMETER_MODIFICATION(uint8_t * PARAMETER);
void MODE_MODIFICATION(uint8_t * PARAMETER);
/* function prototypes -------------------------------------------------------*/
/**
* @brief  �������ݺ���
* @param  None
* @return None
*/
void Sent_Contorl(UART_HandleTypeDef* huart_x)
{
  float temp[Sent_Data_Num];
  UpperMonitor_Sent_Choose(temp);                            //ѡ��Ҫ���������
  UpperMonitor_Sent_Set(temp);                            //��������ת����ʽ
  HAL_UART_Transmit_DMA(huart_x,(uint8_t*)Sent_data_type+3,39);
}

/**
* @brief  ���ڷ��Ͳ���ѡ����(Ҫ�ۿ�������),����ѡ����Ҫ���������
* @param  data:��Ҫ���������ָ��
* @return None.
*/
void UpperMonitor_Sent_Choose(float * data)
{
  uint8_t i;
  for(i=0;i<Sent_Data_Num;i++)
  {
    switch(USART0_Sent_Choose_Data[i])
    {
      /* ���²������ڹ۲�������� */
			
			/*
      case 0: data[i]= RC_CtrlData.rc.ch0;  
          break;
      case 1: data[i]= RC_CtrlData.rc.ch1;  
          break;
			case 2: data[i]= RC_CtrlData.rc.ch2;  
			    break;
			case 3: data[i]= RC_CtrlData.rc.ch3;
			    break;
			case 4: data[i]= RC_CtrlData.rc.s1;
			    break;
			case 5: data[i]= RC_CtrlData.rc.s2;
			    break;
			*/
			    //break;

      default:break;
	  /* ���ϲ������ڹ۲�������� */
    }
  }
}

/**
* @brief  ��λ�������޸ĺ�����Ҫ���Ĳ�����
* @param  PARAMETER��ָ������ָ�룬���ڶ�ȡָ��
* @return None.
*/
void PARAMETER_MODIFICATION(uint8_t * PARAMETER)
{
  switch(PARAMETER[0])
  {
    /* ���²��������޸Ĳ������� */
		/*
     case 0x00: speed_pid.kp=PARAMETER_Change_float(PARAMETER+1);
          break;
		 case 0x01: speed_pid.ki=PARAMETER_Change_float(PARAMETER+2);
          break;
		 case 0x02: speed_pid.kd=PARAMETER_Change_float(PARAMETER+3);
		      break;
		 case 0x03: target_speed=PARAMETER_Change_float(PARAMETER+4);
          break;
*/
		default:break;
	/* ���ϲ��������޸Ĳ������� */
  }
}

/**
* @brief  ���ڷ������ú���,��������DMA���ڵ�����
* @param  data:��Ҫ���������ָ��
* @return None.
*/
void UpperMonitor_Sent_Set(float *data)
{
  uint8_t j;
  Sent_data_type[0].change_u8[3]=0xfd;                          //��������ͷ
  for(j=1;j<Sent_Data_Num+1;j++)                                //������
  {
    Sent_data_type[j].change_float=data[j-1];
  }
  Sent_data_type[Sent_Data_Num+1].change_u8[0]=Sent_Data_Num;   //����β
  Sent_data_type[Sent_Data_Num+1].change_u8[1]=0xfe;            //У��λ
}

/**
* @brief  ��λ������ת��ɸ���������
* @param  PARAMETER��ָ������ָ�룬���ڶ�ȡָ��
* @return None.
*/
float PARAMETER_Change_float(uint8_t * PARAMETER)
{
  uint8_t i=0;
  union type_change Sent_data_temp;                       //�������ݹ�����
  for(i=0;i<4;i++)
  {
    Sent_data_temp.change_u8[i]=PARAMETER[3-i];           //ת���ɹ�������������
  }
  return Sent_data_temp.change_float;                     //���ع�����ת���������
}



/**
* @brief  ��λ�������޸ĺ���
* @param  PARAMETER�� ָ������ָ�룬���ڶ�ȡָ��
* @return None.
*/
void MODE_MODIFICATION(uint8_t * PARAMETER)
{
  switch(PARAMETER[0])
  {
    case 0x00: USART0_Sent_Choose_Data[0]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x01: USART0_Sent_Choose_Data[1]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x02: USART0_Sent_Choose_Data[2]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x03: USART0_Sent_Choose_Data[3]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x04: USART0_Sent_Choose_Data[4]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x05: USART0_Sent_Choose_Data[5]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x06: USART0_Sent_Choose_Data[6]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x07: USART0_Sent_Choose_Data[7]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x08: USART0_Sent_Choose_Data[8]=PARAMETER_Change_float(PARAMETER+1);
          break;
    default:break;
  }
}

uint8_t  USART_Interrupt_flag=0xff;           //�����жϱ�־λ 
uint8_t  USART_Get_Num_Flag=0;                //�������ݻ�ȡ��־
uint8_t  USART_receive[5]={0};                //���ڽ��ջ�������
int len=0;
/**
* @brief  ���ڽ��ս�������
* @param  data_buf�����յ�������ָ��
          length  �����ݳ���
* @return No meaning.
*/
uint32_t RecHandle(uint8_t *data_buf,uint16_t length)
{
  uint8_t Temp=0;
    len=length;
  for(int i=0;i<length;i++)
  {
    Temp=data_buf[i]; 
    switch(USART_Interrupt_flag)
    {
      case 0xff:  //USART0_Interrupt_flag==0xffʱΪ�ȴ�ģʽ���ȴ�ָ��ͷ����
            if(Temp==0xf0)                  //ָ��ͷ��ʶ����λ���������޸�ָ��
              USART_Interrupt_flag=0xf0;    //��һ��ָ�����ģʽѡ��ģʽ
            break;
      case 0xf0:                            //����ģʽѡ��
            if(Temp==0x00)                  //�޸Ĳ���
            {
              USART_Interrupt_flag=0x00;    //��������޸�ģʽ
              USART_Get_Num_Flag=0;
            }
            else if(Temp==0x01)             //�޸�ģʽ
            {
              USART_Interrupt_flag=0x01;    //����ģʽ�޸�ģʽ
              USART_Get_Num_Flag=0;
            }
            else if(Temp==0x02)
            {
              USART_Interrupt_flag=0x02;    //����ģʽ�޸�ģʽ
              USART_Get_Num_Flag=0;
            }
            break;
      case 0x00:
            USART_receive[USART_Get_Num_Flag]=Temp;
            USART_Get_Num_Flag++;
            if(USART_Get_Num_Flag>4)        //��������
            {
              PARAMETER_MODIFICATION(USART_receive);
              USART_Interrupt_flag=0xff;    //�ص��ȴ�ģʽ
            }
            break;
      case 0x01:
            USART_receive[USART_Get_Num_Flag]=Temp;
            USART_Get_Num_Flag++;
            if(USART_Get_Num_Flag>4)        //��������
            {
              MODE_MODIFICATION(USART_receive);
              USART_Interrupt_flag=0xff;    //�ص��ȴ�ģʽ
            }
            break;
      case 0x02:  USART_receive[USART_Get_Num_Flag]=Temp;
            USART_Get_Num_Flag++;
            if(USART_Get_Num_Flag>4)        //��������
            {
              if(USART_receive[0]==0x0a)
              {
                for(int j=1;j<5;j++)
                {
                  if(USART_receive[j]!=0x0a)
                    USART_Interrupt_flag=0xff;    //�ص��ȴ�ģʽ
                }
                if(USART_Interrupt_flag==0x02)
                {
                  On_Off_flag =1;
                  USART_Interrupt_flag=0xff;    //�ص��ȴ�ģʽ
                }
              }
              else if(USART_receive[0]==0xb0)
              {
                for(int j=1;j<5;j++)
                {
                  if(USART_receive[j]!=0xb0)
                    USART_Interrupt_flag=0xff;   //�ص��ȴ�ģʽ
                }
                if(USART_Interrupt_flag==0x02)
                {
                  On_Off_flag =0;
                  USART_Interrupt_flag=0xff;    //�ص��ȴ�ģʽ
                }
              }
              else 
                USART_Interrupt_flag=0xff;     //�ص��ȴ�ģʽ
            }
            break;
            
      default:  USART_Interrupt_flag=0xff;    //�ص��ȴ�ģʽ
            break;
    }
  }
  return 0;
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

