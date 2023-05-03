#include "myiic.h"

//以下自己写两个延时函数
void delay_us(uint32_t nus)
{
// uint32_t temp;
// SysTick->LOAD = 21*nus;
// SysTick->VAL=0X00;//清空计数器
// SysTick->CTRL=0X01;//使能，减到零是无动作，采用外部时钟源
// do
// {
//  temp=SysTick->CTRL;//读取当前倒计数值
// }while((temp&0x01)&&(!(temp&(1<<16))));//等待时间到达
//     SysTick->CTRL=0x00; //关闭计数器
//    SysTick->VAL =0X00; //清空计数器
 
   uint16_t i = 0;  
   while(nus--)
   {
      i=12;  //自己定义
      while(i--) ;    
   }
}

void delay_ms(uint16_t nms)
{
// uint32_t temp;
// SysTick->LOAD = 21000*nms;
// SysTick->VAL=0X00;//清空计数器
// SysTick->CTRL=0X01;//使能，减到零是无动作，采用外部时钟源
// do
// {
//  temp=SysTick->CTRL;//读取当前倒计数值
// }while((temp&0x01)&&(!(temp&(1<<16))));//等待时间到达
//    SysTick->CTRL=0x00; //关闭计数器
//    SysTick->VAL =0X00; //清空计数器
	
	uint16_t i=0;  
   while(nms--)
   {
      i=12000;  //自己定义
      while(i--) ;    
   }
}


#if 1

void IIC_Init(void)
{
  GPIO_InitTypeDef GPIO_Initure;
    
  IIC_RCC_ENABLE  //使能GPIOB时钟
    
    //PB6,7初始化设置
  GPIO_Initure.Pin=IIC_SCL_PIN|IIC_SDA_PIN;
  GPIO_Initure.Mode=GPIO_MODE_OUTPUT_OD;  //推挽输出
  //GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
  GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;     //快速
  HAL_GPIO_Init(IIC_GPIO_PORT,&GPIO_Initure);
    
  IIC_SCL_H
  IIC_SDA_H
  //IIC_Stop();  
}

//#include "myiic.h"
//#include "delay.h"

//产生IIC起始信号
void IIC_Start(void)
{
	IIC_SDA_OUT();     //sda线输出
	IIC_SDA_H	  	  
	IIC_SCL_H
	delay_us(4);
 	IIC_SDA_L//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL_L//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	IIC_SDA_OUT();//sda线输出
	IIC_SCL_L
	IIC_SDA_L//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL_H 
	IIC_SDA_H//发送I2C总线结束信号
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	IIC_SDA_IN();      //SDA设置为输入  
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
	IIC_SCL_L//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
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
//不产生ACK应答		    
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
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	IIC_SDA_OUT(); 	    
    IIC_SCL_L//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        //IIC_SDA=(txd&0x80)>>7; //只有当txd=0x80时，才是全1，右移7位后
      if((txd&0x80)>>7) 
			{
				IIC_SDA_H
			}else{
			IIC_SDA_L
			}				
			txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL_H
		delay_us(2); 
		IIC_SCL_L	
		delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	IIC_SDA_IN();//SDA设置为输入
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
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}


#endif

