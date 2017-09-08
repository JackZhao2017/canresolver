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

#define factor 1/256

unsigned char buf[]={0x55,0x30,0x6,0x8,0x55,0xc,0x0};
volatile u16 g_speed=0;
void can_resolve_speed(unsigned char *buf)
{
		u16 speed;
		speed=buf[6];
		g_speed=speed;
		printf(" g_speed =%d \r\n",g_speed);
}
void uart_send_speed(u16 speed)
{
		USART2->DR=speed&0xff;
		while((USART2->SR&0X40)==0);
		//can_tx(buf,sizeof(buf));
}



