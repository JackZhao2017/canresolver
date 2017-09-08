#include "stdio.h"
#include "stm32f10x.h"
#include "led.h"
#include "delay.h"
#include "crc8.h"
#include "usart.h"
#include "usart2.h"
#include "can.h"
#include "string.h"
#include "can_speed.h"
#include "timer.h"


FILTER filter[2]={{0x125,0x7ff,STD},{0x00fe6c00,0x00ffff00,EXT}};
volatile s8 g_can_protocol=0;

void set_system_status(s8 status)
{
			g_can_protocol=status;
}

int main(void)
{
		u16 scale=920;
		u16 period=1000;
		delay_init();
		LED_Init();		
		uart_init(9600);
		crcInit(LSB,POLY);
	  USART2_Init(115200);
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		CAN_GPIO_Config();					//CAN管脚初始化
	  CAN_NVIC_Configuration(); 	//CAN中断初始化   
	  CAN_INIT(CAN_250kbps);			//CAN初始化N模块	
		CAN_FILTERINIT(filter,sizeof(filter)/sizeof(FILTER));
		TIM3_Int_Init(499,7199);//10Khz的计数频率，计数到500为50ms  
		LED_ON();
	  while(1)
		{		 
				if(g_can_protocol>0){
						scale=500;
						period=1000;
				}else if(g_can_protocol<0){
						period=500;
						scale =period/2;
				}else if(g_can_protocol==0){
						scale=80;
						period=1000;
				}
				LED0=0;
				LED1=1;
				LED_ON();
				delay_ms(period-scale);
				LED0=1;
				LED1=0;
				LED_OFF();
				delay_ms(scale);
		}	
}
