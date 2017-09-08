/**********************************************************************************
 * 文件名  ：MCP2515.c
 * 描述    ：MCP2515驱动函数库         
 * 实验平台：NiRen_STC/IAP15核心板(或用户STC15单片机开发板) + NiRen_MCP2515 CAN模块    
**********************************************************************************/

#include "MCP2515.H"
#include "stm32f10x.h" 
#include "sys.h"



//MCP2515引脚定义
#define  MCP2515_SCK  		PAout(5)	 	//SPI时钟引脚 
#define  MCP2515_MOSI 		PAout(7)		//SPI主机输出从机输入引脚 
#define  MCP2515_MISO 		PAin(6)			//SPI主机输入从机输出引脚 
#define  MCP2515_CS   		PAout(4)		//SPI片选引脚 

//MCP2515波特率预分频
#define	CAN_10Kbps	0x31
#define CAN_25Kbps	0x13
#define CAN_50Kbps	0x09
#define CAN_100Kbps	0x04
#define CAN_125Kbps	0x03
#define CAN_250Kbps	0x01
#define	CAN_500Kbps	0x00

void MCP2515_GPIO_Config(void)
{
		GPIO_InitTypeDef GPIO_InitStructure; 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;			//SPI CS
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;			//SPI SCK
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;			//SPI_MISO
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	 
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;			//SPI_MOSI
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		

}
/*******************************************************************************
* 函数名  : Delay_Nms
* 描述    : 通过软件延时约nms(不准确)
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 此方式延时时间是不准确的,准确延时建议用定时器
*******************************************************************************/
void Delay_Nms(unsigned int x)
{
	unsigned int y;

	for (;x>0;x--)
		for (y=0;y<100;y++);
}

/*******************************************************************************
* 函数名  : SPI_ReadByte
* 描述    : 通过SPI读取一个字节数据
* 输入    : 无
* 输出    : 无
* 返回值  : rByte(读取到的一个字节数据)
* 说明    : 无
*******************************************************************************/
unsigned char SPI_ReadByte(void)
{
		unsigned char i,rByte=0;
	
		MCP2515_SCK=0;
		for(i=0;i<8;i++)
		{
			MCP2515_SCK=1;
			rByte<<=1;
			rByte|=MCP2515_MISO;
			MCP2515_SCK=0;	
		}
		return rByte;
}

/*******************************************************************************
* 函数名  : SPI_SendByte
* 描述    : SPI发送一个字节数据
* 输入    : dt:待发送的数据
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void SPI_SendByte(unsigned char dt)
{
	unsigned char i;
		
	for(i=0;i<8;i++)
	{	
		MCP2515_SCK=0;
		if((dt<<i)&0x80)
			MCP2515_MOSI=1;
		else
			MCP2515_MOSI=0;					
		MCP2515_SCK=1;
	}
	MCP2515_SCK=0;
}

/*******************************************************************************
* 函数名  : MCP2515_WriteByte
* 描述    : 通过SPI向MCP2515指定地址寄存器写1个字节数据
* 输入    : addr:MCP2515寄存器地址,dat:待写入的数据
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void MCP2515_WriteByte(unsigned char addr,unsigned char dat)
{
	MCP2515_CS=0;				//置MCP2515的CS为低电平
	SPI_SendByte(CAN_WRITE);	//发送写命令
	SPI_SendByte(addr);			//发送地址
	SPI_SendByte(dat);			//写入数据
	MCP2515_CS=1;				//置MCP2515的CS为高电平 
}

/*******************************************************************************
* 函数名  : MCP2515_ReadByte
* 描述    : 通过SPI从MCP2515指定地址寄器读1个字节数据
* 输入    : addr:MCP2515寄存器地址
* 输出    : 无
* 返回值  : rByte:读取到寄存器的1个字节数据
* 说明    : 无
*******************************************************************************/
unsigned char MCP2515_ReadByte(unsigned char addr)
{
	unsigned char rByte;
	
	MCP2515_CS=0;				//置MCP2515的CS为低电平
	SPI_SendByte(CAN_READ);		//发送读命令
	SPI_SendByte(addr);			//发送地址
	rByte=SPI_ReadByte();		//读取数据
	MCP2515_CS=1;				//置MCP2515的CS为高电平
	return rByte;				//返回读到的一个字节数据
}

/*******************************************************************************
* 函数名  : MCP2515_Reset
* 描述    : 发送复位指令软件复位MCP2515
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 将内部寄存器复位为缺省状态,并将器件设定为配置模式
*******************************************************************************/
void MCP2515_Reset(void)
{
	MCP2515_CS=0;				//置MCP2515的CS为低电平
	SPI_SendByte(CAN_RESET);	//发送寄存器复位命令
	MCP2515_CS=1;				//置MCP2515的CS为高电平
}

/*******************************************************************************
* 函数名  : MCP2515_Init
* 描述    : MCP2515初始化配置
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 初始化包括：软件复位、工作波特率设置、标识符相关配置等。
*******************************************************************************/
void MCP2515_Init(void)
{
	unsigned char temp=0;

	MCP2515_Reset();	//发送复位指令软件复位MCP2515
	Delay_Nms(1);		//通过软件延时约nms(不准确)

	//设置波特率为125Kbps
	//set CNF1,SJW=00,长度为1TQ,BRP=49,TQ=[2*(BRP+1)]/Fsoc=2*50/8M=12.5us
	MCP2515_WriteByte(CNF1,CAN_125Kbps);
	//set CNF2,SAM=0,在采样点对总线进行一次采样，PHSEG1=(2+1)TQ=3TQ,PRSEG=(0+1)TQ=1TQ
	MCP2515_WriteByte(CNF2,0x80|PHSEG1_3TQ|PRSEG_1TQ);
	//set CNF3,PHSEG2=(2+1)TQ=3TQ,同时当CANCTRL.CLKEN=1时设定CLKOUT引脚为时间输出使能位
	MCP2515_WriteByte(CNF3,PHSEG2_3TQ);
	
	MCP2515_WriteByte(TXB0SIDH,0xFF);//发送缓冲器0标准标识符高位
	MCP2515_WriteByte(TXB0SIDL,0xE0);//发送缓冲器0标准标识符低位
	
	MCP2515_WriteByte(RXB0SIDH,0x00);//清空接收缓冲器0的标准标识符高位
	MCP2515_WriteByte(RXB0SIDL,0x00);//清空接收缓冲器0的标准标识符低位
	MCP2515_WriteByte(RXB0CTRL,0x20);//仅仅接收标准标识符的有效信息
	MCP2515_WriteByte(RXB0DLC,DLC_8);//设置接收数据的长度为8个字节
	
	MCP2515_WriteByte(RXF0SIDH,0xFF);//配置验收滤波寄存器n标准标识符高位
	MCP2515_WriteByte(RXF0SIDL,0xE0);//配置验收滤波寄存器n标准标识符低位
	MCP2515_WriteByte(RXM0SIDH,0xFF);//配置验收屏蔽寄存器n标准标识符高位
	MCP2515_WriteByte(RXM0SIDL,0xE0);//配置验收屏蔽寄存器n标准标识符低位
	
	MCP2515_WriteByte(CANINTF,0x00);//清空CAN中断标志寄存器的所有位(必须由MCU清空)
	MCP2515_WriteByte(CANINTE,0x01);//配置CAN中断使能寄存器的接收缓冲器0满中断使能,其它位禁止中断
	
	MCP2515_WriteByte(CANCTRL,REQOP_NORMAL|CLKOUT_ENABLED);//将MCP2515设置,退出配置模式
	
	temp=MCP2515_ReadByte(CANSTAT);//读取CAN状态寄存器的值
	
	if(OPMODE_NORMAL!=(temp&&0xE0))//判断MCP2515是否已经进入环回模式
	{
		MCP2515_WriteByte(CANCTRL,REQOP_NORMAL|CLKOUT_ENABLED);//再次将MCP2515设置为正常,退出配置模式
	}
}

/*******************************************************************************
* 函数名  : CAN_Send_Buffer
* 描述    : CAN发送指定长度的数据
* 输入    : *CAN_TX_Buf(待发送数据缓冲区指针),len(待发送数据长度)
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void CAN_Send_Buffer(unsigned char *CAN_TX_Buf,unsigned char len)
{
	unsigned char j,dly,count;

	count=0;
	while(count<len)
	{
		dly=0;
		while((MCP2515_ReadByte(TXB0CTRL)&0x08) && (dly<50))//快速读某些状态指令,等待TXREQ标志清零
		{
			Delay_Nms(1);//通过软件延时约nms(不准确)
			dly++;
		}
				
		for(j=0;j<8;)
		{
			MCP2515_WriteByte(TXB0D0+j,CAN_TX_Buf[count++]);//将待发送的数据写入发送缓冲寄存器
			j++;
			if(count>=len) break;
		}
		MCP2515_WriteByte(TXB0DLC,j);//将本帧待发送的数据长度写入发送缓冲器0的发送长度寄存器
		MCP2515_CS=0;
		MCP2515_WriteByte(TXB0CTRL,0x08);//请求发送报文
		MCP2515_CS=1;
	}
}

/*******************************************************************************
* 函数名  : CAN_Receive_Buffer
* 描述    : CAN接收一帧数据
* 输入    : *CAN_TX_Buf(待接收数据缓冲区指针)
* 输出    : 无
* 返回值  : len(接收到数据的长度,0~8字节)
* 说明    : 无
*******************************************************************************/
unsigned char CAN_Receive_Buffer(unsigned char *CAN_RX_Buf)
{
	unsigned char i=0,len=0,temp=0;

	temp = MCP2515_ReadByte(CANINTF);
	if(temp & 0x01)
	{
		len=MCP2515_ReadByte(RXB0DLC);//读取接收缓冲器0接收到的数据长度(0~8个字节)
		while(i<len)
		{	
			CAN_RX_Buf[i]=MCP2515_ReadByte(RXB0D0+i);//把CAN接收到的数据放入指定缓冲区
			i++;
		}
	}
	MCP2515_WriteByte(CANINTF,0);//清除中断标志位(中断标志寄存器必须由MCU清零)
	return len;
}


