/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include <stdlib.h>

#include "stdio.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "UpperMonitor.h"

#define KEY0 		PFin(6)   
#define KEY1 		PFin(7)		
#define KEY2 		PFin(8)		
#define KEY3 	  PFin(9)		


//按键值定义
#define KEY0_DATA	  1
#define KEY1_DATA	  2
#define KEY2_DATA	  3
#define KEY3_DATA   4



//定义一些常用的数据类型短关键字 
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  
typedef const int16_t sc16;  
typedef const int8_t sc8;  

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  
typedef __I int16_t vsc16; 
typedef __I int8_t vsc8;   

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  
typedef const uint16_t uc16;  
typedef const uint8_t uc8; 

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  
typedef __I uint16_t vuc16; 
typedef __I uint8_t vuc8;  

typedef struct
{
	u16 data0:1;
	u16 data1:1;
	u16 data2:1;
	u16 data3:1;
	u16 data4:1;
	u16 data5:1;
	u16 data6:1;
	u16 data7:1;
	u16 data8:1;
	u16 data9:1;
	u16 data10:1;
	u16 data11:1;
	u16 data12:1;	
	u16 data13:1;
	u16 data14:1;
	u16 data15:1;	
}_gpio_group;

//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).M4同M3类似,只是寄存器地址变了.
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014     

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //输出 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //输入

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //输出 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //输入

#define SYSCLK 168    //系统时钟

//以下为汇编函数
void WFI_SET(void);		   //执行WFI指令
void INTX_DISABLE(void); //关闭所有中断
void INTX_ENABLE(void);	 //开启所有中断
void MSR_MSP(u32 addr);	 //设置堆栈地址 

void GPIO_group_OUT(_gpio_group *group,u16 outdata);
void GPIO_bits_OUT(GPIO_TypeDef* GPIOx, u8 start_bit, u8 bit_size,u16 outdata);

void Stm32_Clock_Init(u32 plln,u32 pllm,u32 pllp,u32 pllq);//时钟系统配置
	
//void delay_init(void);
//void delay_ms(u16 nms);
//void delay_us(u32 nus);


#define USART3_REC_NUM  			100  	//定义最大接收字节数 200
extern u8 uart_byte_count;          //uart_byte_count要小于USART_REC_LEN
extern u8 receive_str[USART3_REC_NUM];  

extern UART_HandleTypeDef UART3_Handler; //UART句柄
#define RXBUFFERSIZE   1 //缓存大小
extern u8 aRxBuffer[RXBUFFERSIZE];//HAL库USART接收Buffer

void uart3_init(u32 bound);
void uart3SendChars(u8 *str, u16 strlen);




#include "string.h"
#include "stdlib.h"  


//LED端口定义
#define LED0 PGout(13)	 
#define LED1 PGout(14)	 
#define LED2 PGout(15)	  

//函数声明
void LED_Init(void);//初始化

void LED_Init(void)
{    	 
    GPIO_InitTypeDef GPIO_Initure;
	  __HAL_RCC_GPIOG_CLK_ENABLE();           //开启GPIOG时钟
	
    GPIO_Initure.Pin=GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15; //PG13,14，15
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOG,&GPIO_Initure);
	
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_SET);	//PG13置1，默认初始化后灯灭
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_14,GPIO_PIN_SET);	//PG14置1，默认初始化后灯灭
	  HAL_GPIO_WritePin(GPIOG,GPIO_PIN_15,GPIO_PIN_SET);	//PG15置1，默认初始化后灯灭
}

/*********************************************************************************
************************启明欣欣 STM32F407核心开发板******************************
**********************************************************************************
* 文件名称: usart1.c                                                             *
* 文件简述：USART1使用                                                           *
* 创建日期：2015.03.06                                                           *
* 版    本：V1.0                                                                 *
* 作    者：Clever                                                               *
* 说    明：利用串口调试助手经过USART1控制LED亮灭与蜂鸣器响闭                    * 
**********************************************************************************
*********************************************************************************/	

u8 receive_str[USART3_REC_NUM];     //接收缓存数组,最大USART_REC_LEN个字节 
u8 uart_byte_count=0;

u8 aRxBuffer[RXBUFFERSIZE]; //HAL库使用的串口接收缓冲
UART_HandleTypeDef UART3_Handler; //UART句柄

/****************************************************************************
* 名    称: void uart1_init(u32 bound)
* 功    能：USART1初始化
* 入口参数：bound：波特率
* 返回参数：无
* 说    明： 
****************************************************************************/
void uart3_init(u32 bound)
{   
  //UART 初始化设置
	UART3_Handler.Instance=USART3;					    //USART3
	UART3_Handler.Init.BaudRate=bound;				    //波特率
	UART3_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART3_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART3_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART3_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART3_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&UART3_Handler);					    //HAL_UART_Init()会使能UART1
	
	HAL_UART_Receive_IT(&UART3_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
}


/*因printf()之类的函数，使用了半主机模式。使用标准库会导致程序无法
	运行,以下是解决方法:使用微库,因为使用微库的话,不会使用半主机模式. 
	请在工程属性的“Target“-》”Code Generation“中勾选”Use MicroLIB“这
	样以后就可以使用printf，sprintf函数了*/ 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART3->SR&0X40)==0);//循环发送,直到发送完毕   
	USART3->DR = (u8) ch;      
	return ch;
}


//串口1发送一个字符
void uart3SendChar(u8 ch)
{      
	while((USART3->SR&0x40)==0);  
    USART3->DR = (u8) ch;      
}


/****************************************************************************
* 名    称: void uart1SendChars(u8 *str, u16 strlen)
* 功    能：串口1发送一字符串
* 入口参数：*str：发送的字符串
            strlen：字符串长度
* 返回参数：无
* 说    明： 
****************************************************************************/
void uart3SendChars(u8 *str, u16 strlen)
{ 
	  u16 k= 0 ; 
   do { uart3SendChar(*(str + k)); k++; }   //循环发送,直到发送完毕   
    while (k < strlen); 
} 


//串口1中断服务程序
void USART3_IRQHandler(void)  
{
	u32 timeout=0;
	
	HAL_UART_IRQHandler(&UART3_Handler);	//调用HAL库中断处理公用函数
	
	timeout=0;
    while (HAL_UART_GetState(&UART3_Handler) != HAL_UART_STATE_READY)//等待就绪
	{
	 timeout++;////超时处理
     if(timeout>HAL_MAX_DELAY) break;		
	
	}
     
	timeout=0;
	while(HAL_UART_Receive_IT(&UART3_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)//一次处理完成之后，重新开启中断并设置RxXferCount为1
	{
	 timeout++; //超时处理
	 if(timeout>HAL_MAX_DELAY) break;	
  } 
}


//变量声明
extern u8   keydown_data;    //按键按下后就返回的值
extern u8   keyup_data;      //按键抬起返回值
extern u16  key_time;
extern u8   key_tem; 

//函数声明
void KEY_Init(void);	      //IO初始化
void key_scan(u8 mode);  		//按键扫描函数	

float pitch,roll,yaw; 		//欧拉角
short aacx,aacy,aacz;		//加速度传感器原始数据
short gyrox,gyroy,gyroz;	//陀螺仪原始数据
short temp;				//温度

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void servo_angle(int num,int angle);	

float Motorn1;  // 电机转速
float Motorv1;  // 电机转速
int pwm_balance;		  // 电机输出
float Motorn2;  // 电机转速
float Motorv2;  // 电机转速
int v;
int turn_pwm;
int left_speed;
int right_speed;
int Speed;
int Speed1;
int Speed2;
int lspeed;
int hspeed;
unsigned char i = 0;
int TIM2_Count = 0;
int Encoder_left = 0;
int Encoder_right = 0;
float Velocity=0,Encoder_Last=0,Encoder=0,Movement=0;
float Encoder_Integral=0;
float bias;
int target_speed = 0;
u8  keydown_data=0x00;    //按键按下后就返回的值
u8  keyup_data=0x00;      //按键抬起返回值
u16  key_time=0x00;       //按键按下之后的时间计数，该值乘以扫描一次按键函数的时间就等于按键按下的时间
u8  key_tem=0x00;         //长按的按键值与按键扫描程序过渡变量
u8  key_bak=0x00;
int run = 0;
int up_down = 0;
int run_ud = 0;
int delt = 0;

//int fputc(int ch, FILE *p)
//{
//	while(!(USART1->SR & (1<<7)));
//	USART1->DR = ch;
//	
//	return ch;
//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
	KEY_Init();
	uart3_init(9600);	    //串口初始化波特率为9600
	LED_Init();		  		  //初始化与LED 
	printf("%d\r\n",MPU_Init());
	mpu_dmp_init();
	HAL_Delay(500);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);	    // TIM1_CH1(pwm)  PE9
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);	    // TIM1_CH2(pwm)  PE11
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);	    // TIM1_CH3(pwm)  PE13
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);	    // TIM1_CH4(pwm)  PE14
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1); // 开启编码器A    PD12
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2); // 开启编码器B	  PD13
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_1); // 开启编码器A    PC6
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_2); // 开启编码器B	  PC7
  HAL_TIM_Base_Start_IT(&htim2);                // 使能定时器2中断
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);


  while (1)
  {
		 
		//按键模式
//		key_scan(0);
//		if(keyup_data==KEY0_DATA)     //key按下抬起之后执行相应代码
//		{
//			run = 0;
//			up_down = 0;
//			run_ud = 1;
//		}
//		if(keyup_data==KEY1_DATA)     //key按下抬起之后执行相应代码
//		{
//			run = 0;
//			up_down = 1;
//			run_ud = 0;
//		}
//		if(keyup_data==KEY0_DATA)     //key按下抬起之后执行相应代码
//		{
//			run = 1;
//			up_down = 0;
//			run_ud = 0;
//		}
//		
//		if(!run && !up_down && !run_ud)
//		{	
//			servo_angle(1,5);   //CH1 PA6
//			servo_angle(2,5);   //CH2 PA7		
//			servo_angle(3,5);   //CH3 PB0
//			servo_angle(4,5);   //CH4 PB1
//		}

//			servo_angle(1,5);   //CH1 PA6
//			servo_angle(2,5);   //CH2 PA7		
//			servo_angle(3,5);   //CH3 PB0
//			servo_angle(4,5);   //CH4 PB1
//		
		//舵机角度 TIM3
		//0-20ms->2000,0.5-1.5ms->50-150(13)或150-50(24)

			//******** 伸缩腿平衡 ********
//		if(up_down == 1)
//		{
//			servo_angle(1,5);   //CH1 PA6
//			servo_angle(2,5);   //CH2 PA7		
//			servo_angle(3,5);   //CH3 PB0
//			servo_angle(4,5);   //CH4 PB1
//			HAL_Delay(3000);
//			servo_angle(1,10);   //CH1 PA6
//			servo_angle(2,10);   //CH2 PA7		
//			servo_angle(3,10);   //CH3 PB0
//			servo_angle(4,10);   //CH4 PB1
//			HAL_Delay(50);
//			servo_angle(1,15);   //CH1 PA6
//			servo_angle(2,15);   //CH2 PA7		
//			servo_angle(3,15);   //CH3 PB0
//			servo_angle(4,15);   //CH4 PB1
//			HAL_Delay(50);
//			servo_angle(1,20);   //CH1 PA6
//			servo_angle(2,20);   //CH2 PA7		
//			servo_angle(3,20);   //CH3 PB0
//			servo_angle(4,20);   //CH4 PB1
//			HAL_Delay(50);
//			servo_angle(1,25);   //CH1 PA6
//			servo_angle(2,25);   //CH2 PA7		
//			servo_angle(3,25);   //CH3 PB0
//			servo_angle(4,25);   //CH4 PB1
//			HAL_Delay(50);
//			servo_angle(1,30);   //CH1 PA6
//			servo_angle(2,30);   //CH2 PA7		
//			servo_angle(3,30);   //CH3 PB0
//			servo_angle(4,30);   //CH4 PB1
//			HAL_Delay(500);
//			servo_angle(1,25);   //CH1 PA6
//			servo_angle(2,25);   //CH2 PA7		
//			servo_angle(3,25);   //CH3 PB0
//			servo_angle(4,25);   //CH4 PB1
//			HAL_Delay(50);
//			servo_angle(1,20);   //CH1 PA6
//			servo_angle(2,20);   //CH2 PA7		
//			servo_angle(3,20);   //CH3 PB0
//			servo_angle(4,20);   //CH4 PB1
//			HAL_Delay(50);
//			servo_angle(1,15);   //CH1 PA6
//			servo_angle(2,15);   //CH2 PA7		
//			servo_angle(3,15);   //CH3 PB0
//			servo_angle(4,15);   //CH4 PB1
//			HAL_Delay(50);
//			servo_angle(1,10);   //CH1 PA6
//			servo_angle(2,10);   //CH2 PA7		
//			servo_angle(3,10);   //CH3 PB0
//			servo_angle(4,10);   //CH4 PB1
//			HAL_Delay(50);
//		}
		
//		if(run_ud == 1)
//		{
//			servo_angle(1,5);   //CH1 PA6
//			servo_angle(2,5);   //CH2 PA7		
//			servo_angle(3,5);   //CH3 PB0
//			servo_angle(4,5);   //CH4 PB1
//			HAL_Delay(3000);
//			servo_angle(1,10);   //CH1 PA6
//			servo_angle(2,10);   //CH2 PA7		
//			servo_angle(3,10);   //CH3 PB0
//			servo_angle(4,10);   //CH4 PB1
//			HAL_Delay(50);
//			servo_angle(1,17);   //CH1 PA6
//			servo_angle(2,15);   //CH2 PA7		
//			servo_angle(3,15);   //CH3 PB0
//			servo_angle(4,17);   //CH4 PB1
//			HAL_Delay(50);
//			servo_angle(1,23);   //CH1 PA6
//			servo_angle(2,20);   //CH2 PA7		
//			servo_angle(3,20);   //CH3 PB0
//			servo_angle(4,23);   //CH4 PB1
//			HAL_Delay(50);
//			servo_angle(1,29);   //CH1 PA6
//			servo_angle(2,25);   //CH2 PA7		
//			servo_angle(3,25);   //CH3 PB0
//			servo_angle(4,29);   //CH4 PB1
//			HAL_Delay(50);
//			servo_angle(1,23);   //CH1 PA6
//			servo_angle(2,20);   //CH2 PA7		
//			servo_angle(3,20);   //CH3 PB0
//			servo_angle(4,23);   //CH4 PB1
//			HAL_Delay(50);
//			servo_angle(1,17);   //CH1 PA6
//			servo_angle(2,15);   //CH2 PA7		
//			servo_angle(3,15);   //CH3 PB0
//			servo_angle(4,17);   //CH4 PB1
//			HAL_Delay(50);
//			servo_angle(1,10);   //CH1 PA6
//			servo_angle(2,10);   //CH2 PA7		
//			servo_angle(3,10);   //CH3 PB0
//			servo_angle(4,10);   //CH4 PB1
//			HAL_Delay(50);
//		}


  }
  /* USER CODE END 3 */
}



//按键IO口初始化函数
void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOF_CLK_ENABLE();           //开启GPIOF时钟
 
		GPIO_Initure.Pin=GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9; //PF6,7,8,9    KEY0 KEY1 KEY2 KEY3对应引脚
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOF,&GPIO_Initure);
} 
/****************************************************************************
* 名    称: void key_scan(u8 mode)
* 功    能：按键扫描函数
* 入口参数：mode：0：单按 
                  1: 连按
* 返回参数：无
* 说    明：响应优先级,KEY0>KEY1>KEY2>KEY3
****************************************************************************/
void key_scan(u8 mode)
{	   
	 keyup_data=0;         //键抬起后按键值一次有效
	
	if(KEY0==0||KEY1==0||KEY2==0||KEY3==0)   //有键正按下
	{
		if(KEY0==0)      key_tem=1;
		else if(KEY1==0) key_tem=2;
		else if(KEY2==0) key_tem=3;
		else if(KEY3==0) key_tem=4;
		
		  if (key_tem == key_bak)      //有键按下后第一次扫描不处理，与else配合第二次扫描有效，这样实现了去抖动
			  {
				   key_time++;             //有键按下后执行一次扫描函数，该变量加1
					 keydown_data=key_tem;   //按键值赋予keydown_data
					
					 if( (mode==0)&&(key_time>1) )//key_time>1按键值无效，这就是单按，如果mode为1就为连按
					    keydown_data=0;
       	}
	    else                             //去抖动      
	      {
		       key_time=0;
		       key_bak=key_tem;
	      }
		
	}
	else       //键抬起
		{
	     if(key_time>2)            //按键抬起后返回一次按键值
	        {
		        keyup_data=key_tem;  //键抬起后按键值赋予keydown_data            						
	       	}
				key_bak=0;               //要清零，不然下次执行扫描程序时按键的值跟上次按的值一样，就没有去抖动处理了
	      key_time=0;
				keydown_data=0;		
	  }    
}


void servo_angle(int num,int angle)
{
	int pwm =  angle*200/180;
	if (num==1) __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 55+pwm);
	if (num==2) __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 150-pwm);
	if (num==3) __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 50+pwm);
	if (num==4) __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 150-pwm);
}


int balance(int v,float Angle,float Gyro)
{  
	 float kp,kd;  //kp对应角度，kd对应角速度
	 //角速度=gyro/16.4；°/s
	 Gyro = Gyro/16.4;
	
   kp=80;kd=7;
	 float angle_balance = 0;
	 int balance;
	 bias=Angle-angle_balance-v;       //===求出平衡的角度中值 和机械相关
	 balance=kp*bias+Gyro*kd;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
		
	 return balance;
}


int velocity(int encoder_left,int encoder_right)
{  
	
		int left = encoder_left;
		int right = encoder_right;
		
    float Velocity_Kp=1,Velocity_Ki=Velocity_Kp/200.0;

    //=============速度PI控制器=======================// 
    Encoder_Last =(left + right)-target_speed;                 //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
    Encoder *= 0.8;                                                  //===一阶低通滤波器       
    Encoder += Encoder_Last*0.2;                                    //===一阶低通滤波器    
    Encoder_Integral +=Encoder;                                      //===积分出位移 积分时间：5ms
    Encoder_Integral=Encoder_Integral-Movement;                      //===接收遥控器数据，控制前进后退
    if(Encoder_Integral>10000)  Encoder_Integral=10000;              //===积分限幅
    if(Encoder_Integral<-10000) Encoder_Integral=-10000;             //===积分限幅  
    Velocity = Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;     //===速度控制  
    return Velocity;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    //static unsigned char i = 0;
		//5ms一次定TIM2中断
    if (htim == (&htim2))
    {
				TIM2_Count ++;
				if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
				{
					//roll往前倒为负
					temp = MPU_Get_Temperature();	//得到温度值
					MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据  角加速度=aac/2048
					MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据        角速度=gyro/16.4
					//printf("\r\npitch:%02f   yaw:%02f   roll:%02f\r\n",pitch,yaw,roll);	
		    }
			
			  //1.获取转速  时间10ms，减速比1:30，编码器ppr=13，采用4倍频，则 n=100*Pulses/(13*30*4)
        //MotorSpeed = (short)(__HAL_TIM_GET_COUNTER(&htim4)*100/(13*4*30));  
//				Motorn1 = (__HAL_TIM_GET_COUNTER(&htim4)*0.064);  //实际大概为一半？	
//				Motorv1 = Motorn1*3.14*0.065;
//				Motorn2 = (__HAL_TIM_GET_COUNTER(&htim8)*0.064);  //实际大概为一半？	
//				Motorv2 = Motorn2*3.14*0.065;
        // TIM4计数器获得电机脉冲，该电机在10ms采样的脉冲/18则为实际转速的rpm
				
				//向前为正，向后为负
				Encoder_left = (short)__HAL_TIM_GET_COUNTER(&htim4);
				Encoder_right = -(short)__HAL_TIM_GET_COUNTER(&htim8);
        __HAL_TIM_SET_COUNTER(&htim4,0);  // 计数器清零
			  __HAL_TIM_SET_COUNTER(&htim8,0);  // 计数器清零
	
				//*****串级PID*****
//				v = velocity(Encoder_left,Encoder_right);
//				pwm_balance=balance(v,roll,gyrox); 
//				Speed = pwm_balance;
				
				//******** 行走 ********
//				if(TIM2_Count<=500) target_speed=0;
//				else target_speed=-10;
//				else if(TIM2_Count<=1500 && target_speed>-15) target_speed--;
//				else if(TIM2_Count<=1500 && target_speed<=-15) target_speed=-15;
//				else if(TIM2_Count<=2700 && target_speed<15) target_speed++;
//				else if(TIM2_Count<=2700 && target_speed>=15) target_speed=15;
//				else TIM2_Count=0;

				int min_speed = 180;
				int max_speed = 400;

//				if(run == 1)
//				{
//					if(TIM2_Count<=500) target_speed=0;
//					else if(TIM2_Count<=1500 && target_speed>-20) target_speed--;
//					else if(TIM2_Count<=1500 && target_speed<=-20) target_speed=-20;
//					else if(TIM2_Count<=2800 && target_speed<20) target_speed++;
//					else if(TIM2_Count<=2800 && target_speed>=20) target_speed=20;
//					else TIM2_Count=0;
//				}
//				
//				if(run_ud == 1)
//				{
//					max_speed = 250;
//					if(TIM2_Count<=500) target_speed=0;
//					else if(TIM2_Count<=1500 && target_speed>-15) target_speed--;
//					else if(TIM2_Count<=1500 && target_speed<=-15) target_speed=-15;
//					else if(TIM2_Count<=2600 && target_speed<15) target_speed++;
//					else if(TIM2_Count<=2600 && target_speed>=15) target_speed=15;
//					else TIM2_Count=0;
//				}
				
				
				//*****并级PID*****
				pwm_balance=balance(0,roll,gyrox); 
				v = velocity(Encoder_left,Encoder_right);
				Speed = pwm_balance - 20*v;

				// 并串PID
//				v = velocity(Encoder_left,Encoder_right);
//				pwm_balance=balance(v,roll,gyrox); 
//				Speed = abs(pwm_balance - v);


					if (Speed>0)
					{
						if (Speed<min_speed) Speed=min_speed;
						else if(Speed>max_speed) Speed=max_speed;
						else Speed=Speed;
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Speed);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Speed-delt);
					} else
					{
						if (Speed>-min_speed) Speed=-min_speed;
						else if(Speed<-max_speed) Speed=-max_speed;
						else Speed=Speed;
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -Speed);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -Speed-delt);
					}
				

        i++;
        if(i>10)
        {
          // 通过观察小灯亮灭来判断是否正确进入定时器中断
          HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
          // 打印定时器4的计数值，short（-32768——32767）
					//printf("Encoder = %d moto = %d \r\n",MotorSpeed,MotorOutput);	
					//printf("转速 = %.2fr/s,  线速度=%.2fm/s \r\n",Motorn1,Motorv1);
					//printf("roll=%.2f,  gyrox=%hd\r\n\r\n",roll,gyrox/(short)16);
//					printf("pitch=%.2f,  gyroy=%hd\r\n\r\n",pitch,gyroy/(short)16);
					//printf("Encoder=%.2f, Integral=%.2f\r\n\r\n",Encoder,Encoder_Integral);
					//printf("left=%d,  right=%d\r\n\r\n",Encoder_left,Encoder_right);
					//printf("speed=%d\r\n\r\n",Speed);
					//printf("roll=%.2f, pitch=%.2f, yaw=%.2f, rx=%d\r\n\r\n",roll,pitch,yaw,gyrox);  //x:roll前后平衡,y:pitch左右平衡,z:yaw转动
					//printf("ax=%hd,ay=%hd,az=%hd,gx=%hd,gy=%hd,gz=%hd \r\n",aacx,aacy,aacz,gyrox,gyroy,gyroz);
          i=0;
        }
//			}
    }
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //中断执行任务函数  被  中断服务函数调用
{
	u8 rec_data;
	
	if(huart->Instance==USART3)//如果是串口1
	{
		rec_data=aRxBuffer[0] ;
		if(rec_data=='S')		  	 //起始位
				{
					uart_byte_count=0x01; 
				}

			else if(rec_data=='E')  //停止位
				{
					if(strcmp("forward",(char *)receive_str)==0)       
						{target_speed-=6;}  //前进
					else if(strcmp("back",(char *)receive_str)==0)   
						{target_speed+=6;}  //后退
					else if(strcmp("stop",(char *)receive_str)==0)   
						{target_speed=0;delt=0;}  //停止
					else if(strcmp("left",(char *)receive_str)==0)  
						{delt+=20;}	//左转
					else if(strcmp("right",(char *)receive_str)==0)   
						{delt-=20;}	  //右转
					
					
					for(uart_byte_count=0;uart_byte_count<32;uart_byte_count++)receive_str[uart_byte_count]=0x00;
					uart_byte_count=0;    
				}				  
			else if((uart_byte_count>0)&&(uart_byte_count<=USART3_REC_NUM))
				{
				   receive_str[uart_byte_count-1]=rec_data;
				   uart_byte_count++;
				}
	}
}





/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 168-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 500-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 840-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 839;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}






/* USER CODE BEGIN 4 */







/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
