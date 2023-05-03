#include "myiic.h"

//�����Լ�д������ʱ����
void delay_us(uint32_t nus)
{
// uint32_t temp;
// SysTick->LOAD = 21*nus;
// SysTick->VAL=0X00;//��ռ�����
// SysTick->CTRL=0X01;//ʹ�ܣ����������޶����������ⲿʱ��Դ
// do
// {
//  temp=SysTick->CTRL;//��ȡ��ǰ������ֵ
// }while((temp&0x01)&&(!(temp&(1<<16))));//�ȴ�ʱ�䵽��
//     SysTick->CTRL=0x00; //�رռ�����
//    SysTick->VAL =0X00; //��ռ�����
 
   uint16_t i = 0;  
   while(nus--)
   {
      i=12;  //�Լ�����
      while(i--) ;    
   }
}

void delay_ms(uint16_t nms)
{
// uint32_t temp;
// SysTick->LOAD = 21000*nms;
// SysTick->VAL=0X00;//��ռ�����
// SysTick->CTRL=0X01;//ʹ�ܣ����������޶����������ⲿʱ��Դ
// do
// {
//  temp=SysTick->CTRL;//��ȡ��ǰ������ֵ
// }while((temp&0x01)&&(!(temp&(1<<16))));//�ȴ�ʱ�䵽��
//    SysTick->CTRL=0x00; //�رռ�����
//    SysTick->VAL =0X00; //��ռ�����
	
	uint16_t i=0;  
   while(nms--)
   {
      i=12000;  //�Լ�����
      while(i--) ;    
   }
}


#if 1

void IIC_Init(void)
{
  GPIO_InitTypeDef GPIO_Initure;
    
  IIC_RCC_ENABLE  //ʹ��GPIOBʱ��
    
    //PB6,7��ʼ������
  GPIO_Initure.Pin=IIC_SCL_PIN|IIC_SDA_PIN;
  GPIO_Initure.Mode=GPIO_MODE_OUTPUT_OD;  //�������
  //GPIO_Initure.Pull=GPIO_PULLUP;          //����
  GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;     //����
  HAL_GPIO_Init(IIC_GPIO_PORT,&GPIO_Initure);
    
  IIC_SCL_H
  IIC_SDA_H
  //IIC_Stop();  
}

//#include "myiic.h"
//#include "delay.h"

//����IIC��ʼ�ź�
void IIC_Start(void)
{
	IIC_SDA_OUT();     //sda�����
	IIC_SDA_H	  	  
	IIC_SCL_H
	delay_us(4);
 	IIC_SDA_L//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL_L//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	IIC_SDA_OUT();//sda�����
	IIC_SCL_L
	IIC_SDA_L//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL_H 
	IIC_SDA_H//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	IIC_SDA_IN();      //SDA����Ϊ����  
	IIC_SDA_H
	delay_us(1);	   
	IIC_SCL_H
	delay_us(1);	 
	while(IIC_SDA_READ)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_L//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL_L
	IIC_SDA_OUT();
	IIC_SDA_L
	delay_us(2);
	IIC_SCL_H
	delay_us(2);
	IIC_SCL_L
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL_L
	IIC_SDA_OUT();
	IIC_SDA_H
	delay_us(2);
	IIC_SCL_H
	delay_us(2);
	IIC_SCL_L
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	IIC_SDA_OUT(); 	    
    IIC_SCL_L//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        //IIC_SDA=(txd&0x80)>>7; //ֻ�е�txd=0x80ʱ������ȫ1������7λ��
      if((txd&0x80)>>7) 
			{
				IIC_SDA_H
			}else{
			IIC_SDA_L
			}				
			txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL_H
		delay_us(2); 
		IIC_SCL_L	
		delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	IIC_SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL_L 
        delay_us(2);
		IIC_SCL_H
        receive<<=1;
        if(IIC_SDA_READ)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}


#endif

