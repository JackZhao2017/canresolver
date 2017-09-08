//======================================================================
//  ???:  MCP2515.c
//  ????:	??SPI??,??MCP2515
//	????:	2009-08-14	v1.0    
//				2009-10-28  v1.1    
//======================================================================

#include "controller.h"
#include "stm32f10x.h" 
#include "sys.h"
#include "stdio.h"
#include "delay.h"
/****************************************************************************
MCP2515_CS		GPB7		output		( nSS0 )
MCP2515_SI		GPE12		output		( SPIMOSI0 )
MCP2515_SO		GPE11		input		( SPIMISO0 )
MCP2515_SCK		GPE13		output		( SPICLK0 )
MCP2515_INT		GPG0		input		( EINT8 )
****************************************************************************/
#define MCP2515_DEBUG    1
#define DELAY_TIME		800


#define MCP2515_CS_H   PAout(4) =1      
#define MCP2515_CS_L   PAout(4) =0    


#define MCP2515_SI_H		PAout(7)=1
#define MCP2515_SI_L		PAout(7)=0

#define MCP2515_SCK_H		PAout(5)=1
#define MCP2515_SCK_L		PAout(5)=0

#define MCP2515_SO_IN		 PAin(6)
#define MCP2515_SO_GET		 PAin(6)

	
/********************** MCP2515 Instruction *********************************/
#define MCP2515INSTR_RESET		0xc0		//???????,????????
#define MCP2515INSTR_READ		0x03		//?????????
#define MCP2515INSTR_WRITE		0x02		//????????
#define MCP2515INSTR_RTS		0x80		//?????????????????
#define MCP2515INSTR_RDSTAT		0xa0		//????
#define MCP2515INSTR_BITMDFY	0x05		//???
//***************************************************************************


/****************************************************************************   
??????SPI??IO?????   
****************************************************************************/   


void CONTROLLER_GPIO_Config(void)
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
static void MCP2515_IO_CS_Init( void )    
{
   U16 k;   
   MCP2515_SI_L ;       //SI put 0     
   MCP2515_SCK_L ;      //SCK put 0    
   for (k = 0; k <= DELAY_TIME; k++);  //????300ns    
   MCP2515_CS_H ;           // unselect the MCP2515    
   for (k = 0; k <= DELAY_TIME; k++);  //????300ns    
}   
   
/****************************************************************************  
??????SPI??????,????  
****************************************************************************/   
static void MCP2515_RW_Start( void )    
{
   U16 k;   
   MCP2515_SI_L ;       //SI put 0    
   MCP2515_SCK_L ;      //SCK put 0    
   for (k = 0; k <= DELAY_TIME; k++);  //????300ns    
   MCP2515_CS_L ;           // Select the MCP2515    
   for (k = 0; k <= DELAY_TIME; k++);  //????300ns    
}   
   
/****************************************************************************  
??????SPI??????  
****************************************************************************/   
static void Spi_Write( U8 Data )    
{   
    U8 m ;
    U16 k;   
   
    for( m = 0; m < 8; m++ )   
    {   
        if( (Data&0x80)==0x80 )
        {  
            MCP2515_SI_H;       //SI put 1 
        }   
        else
        {   
            MCP2515_SI_L;       //SI put 0  
        }  
   
		for (k = 0; k <= DELAY_TIME; k++);  //????300ns    
        MCP2515_SCK_H ;     //SCK put 1    
        Data = Data<<1 ;   
        MCP2515_SCK_L ;     //SCK put 0    
		for (k = 0; k <= DELAY_TIME; k++);  //????300ns    
    }   
}   
   
/****************************************************************************  
??????SPI??????  
****************************************************************************/   
static U8 Spi_Read( void)   
{   
    U8 m ;   
    U8 data = 0 ;
    U16 k;   
   
    for( m = 0; m < 8; m++ )   
    {   
        MCP2515_SCK_H ;     //SCK put 1    
		for (k = 0; k <= DELAY_TIME; k++);  //????300ns    
        data = data<<1;  
        if( MCP2515_SO_GET != 0 )   
            data |= 0x01 ;   
        else   
            data &= 0xfe;   
   
		for (k = 0; k <= DELAY_TIME; k++);  //????300ns    
        MCP2515_SCK_L ;     //SCK put 0    
		for (k = 0; k <= DELAY_TIME; k++);  //????300ns    
    }   
   
    return (data);   
}   

/****************************************************************************
?????? Send Command to MCP2515 via SPI 
****************************************************************************/
//static void SendCMDMCP2515( U8 CMD )
//{
//   MCP2515_RW_Start() ;		//Initial IO port and CS is select
//   Spi_Write( CMD );
//   MCP2515_CS_H ;			// Deselect the MCP2515
//}

/****************************************************************************
??????????MCP2515
****************************************************************************/
static void MCP2515_Reset()
{
	MCP2515_RW_Start() ;
	Spi_Write( MCP2515INSTR_RESET );
	MCP2515_CS_H ;
}

/****************************************************************************
???????MCP2515??????????
****************************************************************************/
static void MCP2515_Write( U8 address, U8 value)
{
	MCP2515_RW_Start() ;

	Spi_Write(MCP2515INSTR_WRITE);
	Spi_Write( address );
	Spi_Write( value );

	MCP2515_CS_H ;
}

/****************************************************************************
???????????????????
****************************************************************************/
static void MCP2515_WriteBits( U8 address, U8 data, U8 mask )
{
	MCP2515_RW_Start() ;

	Spi_Write( MCP2515INSTR_BITMDFY );
	Spi_Write( address);
	Spi_Write( mask);
	Spi_Write( data);

	MCP2515_CS_H ;
}

/****************************************************************************
??????              Read often used status
//Status 	 7    	6    	5    	4    	3    	2  	1	0
//		|	|	|	|	|	|	|	|									
//		|	|	|	|	|	|	|	|___CANINTF.RX0IF
//		|	|	|	|	|	|	|_______CANINTF.RX1IF
//		|	|	|	|	|	|___________TXB0CTRL.TXREQ
//		|	|	|	|	|_______________CANINTF.TX0IF
//		|	|	|	|___________________TXB1CTRL.TXREQ
//		|	|	|_______________________CANINTF.TX1IF
//		|	|___________________________TXB2CTRL.TXREQ
//		|_______________________________CANINTF.TX2IF
****************************************************************************/
static unsigned char MCP2515_ReadStatus()
{
	unsigned char result;

	MCP2515_RW_Start() ;

	Spi_Write(MCP2515INSTR_RDSTAT);
	result = Spi_Read() ;
	Spi_Write( 0 ) ;		//??????
	MCP2515_CS_H ;
	 // if( MCP2515_DEBUG )		Uart_Printf( "StatusREG = 0x%x\n", result ) ;
	return result;
}

/****************************************************************************
???????MCP2515???????????
****************************************************************************/
static unsigned char MCP2515_Read( U8 address )
{
	unsigned char result;
	MCP2515_RW_Start() ;

	Spi_Write(MCP2515INSTR_READ) ;		//0x03
	Spi_Write( address ) ;
	result = Spi_Read() ;

	MCP2515_CS_H ;
	return result ;
}

/****************************************************************************
??????????MCP2515??	
****************************************************************************/
static void MCP2515_SRead( U8 address, unsigned char* pdata, U8 nlength )
{
	int i;

	MCP2515_RW_Start() ;
	Spi_Write(MCP2515INSTR_READ);
	Spi_Write( address );

	for (i=0; i<nlength; i++)
	{
		*pdata=Spi_Read();
		   //if( MCP2515_DEBUG )    Uart_Printf( "  0x%x\n", (unsigned char)*pdata ) ;
		pdata++;
	}
	MCP2515_CS_H ;
}


/****************************************************************************
??????????MCP2515??	
****************************************************************************/
static void MCP2515_Swrite( U8 address, unsigned char* pdata, U8 nlength)
{
	int i;
	MCP2515_RW_Start() ;

	Spi_Write(MCP2515INSTR_WRITE);
	Spi_Write((unsigned char)address);

	for (i=0; i < nlength; i++) 
	{
		Spi_Write( (unsigned char)*pdata );
		//if( MCP2515_DEBUG )    Uart_Printf( "0x%x\n", (unsigned char)*pdata ) ;
		pdata++;
	}
	MCP2515_CS_H ;
}

/****************************************************************************
??????
****************************************************************************/
static void MCP2515_SetBandRate(CanBandRate bandrate, int IsBackNormal)
{
	U8 value=0;
	U8 ReadBackCNT = 0;

	// Bit rate calculations.
	//
	//Input clock fre=16MHz
	// In this case, we'll use a speed of 125 kbit/s, 250 kbit/s, 500 kbit/s.
	// If we set the length of the propagation segment to 7 bit time quanta,
	// and we set both the phase segments to 4 quanta each,
	// one bit will be 1+7+4+4 = 16 quanta in length.
	//
	// setting the prescaler (BRP) to 0 => 500 kbit/s.
	// setting the prescaler (BRP) to 1 => 250 kbit/s.
	// setting the prescaler (BRP) to 3 => 125 kbit/s.
	//
	// If we set the length of the propagation segment to 3 bit time quanta,
	// and we set both the phase segments to 1 quanta each,
	// one bit will be 1+3+2+2 = 8 quanta in length.
	// setting the prescaler (BRP) to 0 => 1 Mbit/s.

	// Go into configuration mode
	MCP2515_Write(MCP2515REG_CANCTRL, MODE_CONFIG);

	if( MCP2515_DEBUG )  printf( "MCP2515REG_CANCTRL =  0x%x\n", MCP2515_Read(MCP2515REG_CANCTRL) );

	while( ReadBackCNT<8 )
	{
		value = ( MCP2515_Read( MCP2515REG_CANSTAT ) & 0xe0 );
		if(value == MODE_CONFIG ){
			printf( "ReadBackCNT = 0x%x\n", ReadBackCNT );
			break;
		}
		ReadBackCNT++ ;
		printf( "ReadBackCNT = 0x%x,value=0x%x\n", ReadBackCNT,value );
	}
	
	if( ReadBackCNT == 8 ) 			//Set mcp2515's mode failed,redo it again
	{
		printf( "Set config mode is failed! CANCTRL = 0x%x\n", value );
		MCP2515_Reset();
		MCP2515_Write(MCP2515REG_CANCTRL, MODE_CONFIG);		//redo to set mcp2515 mode
		delay_ms(150);
		value = ( MCP2515_Read(MCP2515REG_CANCTRL) & 0xe0 );	//read back mode from CANSTAT Register
		printf( "Set is 0x%x , Read is 0x%x\n", MODE_CONFIG, value ) ;
	}
	switch(bandrate){
	case BandRate_10kbps:
		MCP2515_Write(CNF1, 0x31);	//10k	16TQ
		MCP2515_Write(CNF2, 0xb0);  //PS1=7 TQ  PSeg=1 TQ
		MCP2515_Write(CNF3, 0x06);  //PS2=7 TQ SYNC=1 TQ	
		break;
	case BandRate_125kbps:
		MCP2515_Write(CNF1, SJW1|BRP2);	//Synchronization Jump Width Length =1 TQ
		MCP2515_Write(CNF2, BTLMODE_CNF3|(SEG4<<3)|SEG7); // Phase Seg 1 = 4, Prop Seg = 7
		MCP2515_Write(CNF3, SEG4);// Phase Seg 2 = 4
		break;
	case BandRate_250kbps:
		MCP2515_Write(CNF1, SJW1|BRP1);	//Synchronization Jump Width Length =1 TQ
		MCP2515_Write(CNF2, BTLMODE_CNF3|(SEG4<<3)|SEG7); // Phase Seg 1 = 4, Prop Seg = 7
		MCP2515_Write(CNF3, SEG4);// Phase Seg 2 = 4
		break;
	case BandRate_500kbps:
		MCP2515_Write(CNF1, SJW1|BRP1);	//Synchronization Jump Width Length =1 TQ
		MCP2515_Write(CNF2, BTLMODE_CNF3|(SEG3<<3)|SEG2); // Phase Seg 1 = 3, Prop Seg = 2
		MCP2515_Write(CNF3, SEG2);// Phase Seg 2 = 2
		break;
	default:
		MCP2515_Write(CNF1, SJW1|BRP1);	//Synchronization Jump Width Length =1 TQ
		MCP2515_Write(CNF2, BTLMODE_CNF3|(SEG3<<3)|SEG2); // Phase Seg 1 = 2, Prop Seg = 3
		MCP2515_Write(CNF3, SEG2);// Phase Seg 2 = 2
		break;
	}

	if( IsBackNormal == TRUE  )
	{
		//Enable clock output
		MCP2515_Write(CLKCTRL, MODE_NORMAL | CLKEN | CLK8);
	}

}

/****************************************************************************
????????MCP2515 CAN??ID
??: address?MCP2515?????
	can_id????ID?
???
TRUE,?????ID(29?)
FALSE,?????ID(11?)
****************************************************************************/
static int MCP2515_Read_Can_ID( U8 address, U32* can_id)
{
	U32 tbufdata;
	unsigned char* p=(unsigned char*)&tbufdata;

	MCP2515_SRead(address, p, 4);
	*can_id = (tbufdata<<3)|((tbufdata>>13)&0x7);
	*can_id &= 0x7ff;

	if ( (p[MCP2515LREG_SIDL] & TXB_EXIDE_M) ==  TXB_EXIDE_M ) {
		*can_id = (*can_id<<2) | (p[MCP2515LREG_SIDL] & 0x03);
		*can_id <<= 16;
		*can_id |= tbufdata>>16;
		return TRUE;
	}
	return FALSE;
}

/***********************************************************\
*	??MCP2515 ?????							*
*	??: nbuffer??????????3??4	*
*			can_id????ID?							*
*			rxRTR?????RXRTR						*
*			data???????						*
*			dlc??data length code							*
*	???												*
*		TRUE,???????						*
*		FALSE,???????						*
\***********************************************************/
static int MCP2515_Read_Can(U8 nbuffer, int* rxRTR, U32* can_id, U8* data , U8* dlc)
{

	U8 mcp_addr = (nbuffer<<4) + 0x31, ctrl;
	int IsExt;

	IsExt=MCP2515_Read_Can_ID( mcp_addr, can_id);
	ctrl=MCP2515_Read(mcp_addr-1);
	*dlc=MCP2515_Read( mcp_addr+4);
	if ((ctrl & 0x08)) {
		*rxRTR = TRUE;
	}
	else{
		*rxRTR = FALSE;
	}
	*dlc &= DLC_MASK;	
	//MCP2515_SRead(mcp_addr+5, data, *dlc);
	MCP2515_SRead(mcp_addr+5, data, 8); 
	return IsExt;
}


/***********************************************************\
*	??MCP2515 ?????							*
*	??: nbuffer??????????0?1?2	*
*			ext?????????					*
*			can_id????ID?							*
*			rxRTR?????RXRTR						*
*			data???????						*
*			dlc??data length code							*
*		FALSE,???????						*
\***********************************************************/
static void MCP2515_Write_Can( U8 nbuffer, int ext, U32 can_id, int rxRTR, U8* data,U8 dlc )
{
	U8 mcp_addr = (nbuffer<<4) + 0x31;
	MCP2515_Swrite(mcp_addr+5, data, dlc );  // write data bytes
	MCP2515_Write_Can_ID( mcp_addr, can_id,ext);  // write CAN id
	if (rxRTR)
		dlc |= RTR_MASK;  // if RTR set bit in byte
	MCP2515_Write((mcp_addr+4), dlc);            // write the RTR and DLC
}

/*******************************************\
*	??MCP2515 CAN??ID				*
*	??: address?MCP2515?????*
*			can_id????ID?			*
*			IsExt???????ID	*
\*******************************************/
static void MCP2515_Write_Can_ID(U8 address, U32 can_id, int IsExt)
{
	U32 tbufdata;

	if (IsExt) {
		can_id&=0x1fffffff;	//29?
		tbufdata=can_id &0xffff;
		tbufdata<<=16;
		tbufdata|=(can_id>>(18-5)&(~0x1f));
		tbufdata |= TXB_EXIDE_M;
	}
	else{
		can_id&=0x7ff;	//11?
		tbufdata= (can_id>>3)|((can_id&0x7)<<13);
	}
	MCP2515_Swrite(address, (unsigned char*)&tbufdata, 4);
}

/***********************************************************************************\
								????
	??:
		data,????
	Note: ???????????,??????????
\***********************************************************************************/

static void Can_Write(U32 id, U8 *pdata, unsigned char dlc, int IsExt, int rxRTR)
{
	unsigned char err ;
	static int ntxbuffer=0;
	MCP2515_Write_Can(ntxbuffer, IsExt, id, rxRTR, pdata, dlc);

	switch(ntxbuffer){
	case 0:
		MCP2515_WriteBits(TXB0CTRL, (TXB_TXREQ_M|TXB_TXP10_M), 0xff) ;
		//do { err = MCP2515_Read(TXB0CTRL) ; }
		//while( (err &0x08)==0x08 )  ;
		if( (err &0x70) != 0 )  printf( "  Can Send Err = 0x%x\n", err  );
		ntxbuffer=1;
		break;
	case 1:
		MCP2515_WriteBits(TXB1CTRL, (TXB_TXREQ_M|TXB_TXP10_M), 0xff) ;
		//do { err = MCP2515_Read(TXB1CTRL) ; }
		//while( (err &0x08)==0x08 )  ;
		if( (err &0x70) != 0 )  printf( "  Can Send Err = 0x%x\n", err  );
		ntxbuffer=2;
		break;
	case 2:
		MCP2515_WriteBits(TXB2CTRL, (TXB_TXREQ_M|TXB_TXP10_M), 0xff) ;
		//do { err = MCP2515_Read(TXB2CTRL) ; }
		//while( (err &0x08)==0x08 )  ;
		if( (err &0x70) != 0 )  printf( "  Can Send Err = 0x%x\n", err  );
		ntxbuffer=0;
		break;
	}

}


/***********************************************************************************\
								????????
	???:??????,???-1,
			??,???????????
	Note: ????????????,?????????
\***********************************************************************************/
static int Can2515_Poll()
{
	if( MCP2515_ReadStatus()&RX0INT )
		return 0;
	
	if( MCP2515_ReadStatus()&RX1INT )
		return 1;

	return -1;
}

/****************************************************************************
??????
****************************************************************************/
static int Can_Read(int n, U32* id, U8 *pdata,  U8*dlc, int* rxRTR, int *isExt)
{
	U8 byte;
	byte = MCP2515_Read(CANINTF);

	if(n==0)
	{
		if(byte & RX0INT)
		{
			*isExt=MCP2515_Read_Can(n+3, rxRTR, id, pdata, dlc);
			MCP2515_WriteBits(CANINTF, (U8)(~(RX0INT)), RX0INT); // Clear interrupt
			return TRUE ;
		}
		printf( "Error! 0 bytes is Read!!! CANINTF=0x%x\n", byte ) ;
		return FALSE;
	}
	else if(n ==1 )
	{
		if(byte & RX1INT)
		{
			*isExt=MCP2515_Read_Can(n+3, rxRTR, id, pdata, dlc);
			MCP2515_WriteBits(CANINTF, (U8)(~(RX1INT)), RX1INT); // Clear interrupt
			return TRUE ;
		}
		printf( "0 bytes is Read!!! CANINTF=0x%x\n", byte ) ;
		return FALSE;
	}

	printf( "Error! Receive channel=0x%x\n", n ) ;
	return FALSE;
}

/****************************************************************************
??????
****************************************************************************/
// Setup the CAN buffers used by the application.
// We currently use only one for reception and one for transmission.
// It is possible to use several to get a simple form of queue.
//
// We setup the unit to receive all CAN messages.
// As we only have at most 4 different messages to receive, we could use the
// filters to select them for us.
//
// Init_MCP2515() should already have been called.
void Can_2515Setup(void)
{
    // As no filters are active, all messages will be stored in RXB0 only if
    // no roll-over is active. We want to recieve all CAN messages (standard and extended)
    // (RXM<1:0> = 11).
    // SPI_mcp_write_bits(RXB0CTRL, RXB_RX_ANY, 0xFF);
    // SPI_mcp_write_bits(RXB1CTRL, RXB_RX_ANY, 0xFF);
    // But there is a bug in the chip, so we have to activate roll-over.
	MCP2515_WriteBits(RXB0CTRL, RXB_RX_STD|RXB_BUKT, 0xFF);		//????????,??????,???? 
	MCP2515_WriteBits(RXB1CTRL, RXB_RX_STD, 0xFF);		//????????,??????
}

/****************************************************************************
??????
****************************************************************************/
void Init_MCP2515(CanBandRate bandrate)
{
	unsigned char i,j,a;

	MCP2515_IO_CS_Init() ;
	MCP2515_Reset();

	MCP2515_SetBandRate(bandrate,FALSE);		//?????

	// Disable interrups.
	MCP2515_Write(CANINTE, NO_IE);  		//??????

	// Mark all filter bits as don't care:
	MCP2515_Write_Can_ID(RXM0SIDH, 0x7ff,0);
	MCP2515_Write_Can_ID(RXM1SIDH, 0x7ff,0);
	// Anyway, set all filters to 0:
	MCP2515_Write_Can_ID(RXF0SIDH, 0x125, 0);
	MCP2515_Write_Can_ID(RXF1SIDH, 0x123, 0);
	MCP2515_Write_Can_ID(RXF2SIDH, 0, 0);
	MCP2515_Write_Can_ID(RXF3SIDH, 0, 0);
	MCP2515_Write_Can_ID(RXF4SIDH, 0, 0);
	MCP2515_Write_Can_ID(RXF5SIDH, 0, 0);


    MCP2515_Write(CLKCTRL, MODE_NORMAL| CLKEN | CLK8);//????
  
	// Clear, deactivate the three transmit buffers
	a = TXB0CTRL;
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 14; j++) {
			MCP2515_Write(a, 0);
			a++;
	        }
       	a += 2; // We did not clear CANSTAT or CANCTRL
	}
	// and the two receive buffers.
	MCP2515_Write(RXB0CTRL, 0);
	MCP2515_Write(RXB1CTRL, 0);

	// The two pins RX0BF and RX1BF are used to control two LEDs; set them as outputs and set them as 00.
	MCP2515_Write(BFPCTRL, 0x3C);	
	//Open Interrupt
	MCP2515_Write(CANINTE, RX0IE|RX1IE);
}

/****************************************************************************
??????MCP2515????
****************************************************************************/
int  CAN_2515_RX(unsigned int  *id ,unsigned char *data_read)   
{   
    int i;   
    unsigned char dlc=0;   
    int rxRTR, isExt;      
    if( (i=Can2515_Poll())!=-1 )  
    {   
        Can_Read(i, id, data_read, &dlc, &rxRTR, &isExt); 
				printf("id =%x dlc =%d \r\n",*id,dlc);
				for(i=0;i<dlc;i++)
				{
						printf("0x%x ",data_read[i]);
				}
				printf("\r\n");
				return 1;
    }
		return 0;
}


int CAN_2515_TX(U32 id,unsigned char *data ,int len)
{
		Can_Write(id, data, len, 0,0);	
		return 0;
}



