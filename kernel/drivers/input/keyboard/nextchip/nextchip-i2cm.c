#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <asm/gpio.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/earlysuspend.h>
#include <asm/io.h>

#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#include <mach/board-sec-u8500.h>

#define ACK_DELAY	10
#define TX_DELAY	40
#define STOP_DELAY	10
#define RX_DELAY	10
#define START_DELAY	30


unsigned char I2Cm_Get(void); //get MSB first
unsigned char I2Cm_Put(unsigned char TxTemp);
void I2Cm_Start(void);
void I2Cm_Stop(void);
void I2Cm_delay(unsigned char I2Cm_DlyCnt);
void I2Cm_NAKSend(void);
void I2Cm_ACKSend(void);

void I2Cm_SetSCLHiZ(void);
void I2Cm_SetSDAHiZ(void);
void I2Cm_SCL_Low(void);
void I2Cm_SCL_High(void);
void I2Cm_SDA_Low();
void I2Cm_SDA_High();
unsigned char I2Cm_GetSCL(void);
unsigned char I2Cm_GetSDA(void);

void I2Cm_Pin_Init(void);

// ============================================================================
// I2Cm_SetSCLKHiZ()
// Set SCL pin to High-Z drive mode.
// ============================================================================
void I2Cm_SetSCLHiZ(void)
{
	gpio_direction_input(TOUCHKEY_SCL_JANICE_R0_0);
}

// ============================================================================
// I2Cm_SetSDAKHiZ()
// Set SDA pin to High-Z drive mode.
// ============================================================================
void I2Cm_SetSDAHiZ(void)
{
	gpio_direction_input(TOUCHKEY_SDA_JANICE_R0_0);
}

// ============================================================================
// I2Cm_SCL_Low()
// Set SCL pin to Low.
// ============================================================================
void I2Cm_SCL_Low(void)
{
	gpio_direction_output(TOUCHKEY_SCL_JANICE_R0_0, 0);
}

// ============================================================================
// I2Cm_SCL_High()
// Set SCL pin to High.
// ============================================================================
void I2Cm_SCL_High(void)
{
	gpio_direction_output(TOUCHKEY_SCL_JANICE_R0_0, 1);
}

// ============================================================================
// I2Cm_SDA_Low()
// Set SCL pin to Low.
// ============================================================================
void	I2Cm_SDA_Low()
{	
	gpio_direction_output(TOUCHKEY_SDA_JANICE_R0_0, 0);
}

// ============================================================================
// I2Cm_SDA_High()
// Set SCL pin to High.
// ============================================================================
void I2Cm_SDA_High()
{	
	gpio_direction_input(TOUCHKEY_SDA_JANICE_R0_0);
}

// ============================================================================
// I2Cm_GetSCL()
// read SCL pin whether it is high or low.
// ============================================================================
unsigned char I2Cm_GetSCL(void)
{
	return gpio_get_value(TOUCHKEY_SCL_JANICE_R0_0);
}

// ============================================================================
// I2Cm_GetSDA()
// read SDA pin whether it is high or low.
// ============================================================================
unsigned char I2Cm_GetSDA(void)
{
	return gpio_get_value(TOUCHKEY_SDA_JANICE_R0_0);
}

// ============================================================================
// I2Cm_Pin_Init()
// Set SCL/SDA pin to High-Z drive mode.
// ============================================================================
void I2Cm_Pin_Init(void)
{
	I2Cm_SetSCLHiZ();
	I2Cm_SetSDAHiZ();
}

//-----------------------------------------------------------------------------
//  FUNCTION NAME: I2Cm_Readunsigned chars
//
//  DESCRIPTION:
//
//-----------------------------------------------------------------------------
//
//  ARGUMENTS:
//  SlaveAdr   => Address of slave
//  RxArray    => Array address to get data in.
//  SubAdr     => Sub address 
//  /*Rxunsigned*/ charCount=> Count of unsigned chars to read.

//  RETURNS:
//   	  0 if a slave responds. 
//	  1 if first slave address sending error. 
//	  2 if sub address sending error. 
//	  4 if 2nd slave address sending error. 

unsigned char I2Cm_ReadBytes(unsigned char SlaveAdr, unsigned char *RxArray, unsigned char SubAdr0, unsigned char SubAdr1, unsigned char RxByteCount)
{
	unsigned char	i=0;

	SlaveAdr &= (~0x01);	//for write 	// and the address with the Write bit.       

	I2Cm_Start();
	if(I2Cm_Put(SlaveAdr))
	{
		I2Cm_Stop();
		return 1; //exit with SlaveAddress error bit on  
	} 

	if(I2Cm_Put(SubAdr0))
	{
		I2Cm_Stop();
		return 2; //exit with SubAddress0 error bit on
	} 

	if(I2Cm_Put(SubAdr1))
	{
		I2Cm_Stop();
		return 3; //exit with SubAddress1 error bit on
	} 
	
	I2Cm_Stop(); 

	udelay(50);	// 50us
	
	I2Cm_Start();
	
	SlaveAdr |= 0x01;	//for read		//OR the address with the Read bit.

	if(I2Cm_Put(SlaveAdr)) ;//return 4; //exit with SlaveAddress error bit on    

	udelay(50);	// 50us
	
	for(i=0; i<RxByteCount; i++)
	{
		RxArray[i] = I2Cm_Get();

		if(i!=(RxByteCount-1)) I2Cm_ACKSend(); 
		else I2Cm_NAKSend(); 
	}

	I2Cm_Stop();

    return 0;
}

//-----------------------------------------------------------------------------
//  FUNCTION NAME: I2Cm_Writeunsigned chars
//
//  DESCRIPTION:
//
//-----------------------------------------------------------------------------
//
//  ARGUMENTS:
//  SlaveAdr   => Address of slave
//  TxArray    => Array address to put data out.
//  SubAdr     => Sub address 
//  /*Txunsigned*/ charCount=> Count of unsigned chars to read.

//  RETURNS:
//    0 if a slave responds. 
//	  1 if first slave address sending error. 
//	  8 if data sending error. 
 
unsigned char I2Cm_WriteBytes(unsigned char SlaveAdr, unsigned char *TxArray, unsigned char SubAdr0, unsigned char SubAdr1, unsigned char TxByteCount)
{    
	unsigned char	i=0;
	
	SlaveAdr &= (~0x01);	//for write   //and the address with the Write bit.
	I2Cm_Start();
	if(I2Cm_Put(SlaveAdr)) 
	{	
		I2Cm_Stop();
		return 1; 	//exit with SlaveAddress error bit on   
	}  

	if(I2Cm_Put(SubAdr0)) 
	{
		I2Cm_Stop();
		return 2; //exit with SubAddress0 error bit on
	}    

	if(I2Cm_Put(SubAdr1)) 
	{
		I2Cm_Stop();
		return 3; //exit with SubAddress1 error bit on
	}    

	udelay(50);	// 50us
	
	for(i=0; i<TxByteCount; i++)
	{
		if(I2Cm_Put(TxArray[i]))    
		{
			I2Cm_Stop();
			return 8;  //exit with write data error bit on
		}   	
	}
	I2Cm_Stop();
	return 0;
}

//-----------------------------------------------------------------------------
//Function:	I2Cm_Put
//-----------------------------------------------------------------------------
//
//Description:
//	writes 1unsigned char data and checks ACK
//
//  ARGUMENTS:
//      TxData - Contains data to be transmitted.
//
//Returns:
//	0: Send and get ACK informs Success
//      1: Send and get NAK informs fail
//
unsigned char I2Cm_Put(unsigned char TxTemp) //put MSB first
{
	unsigned char	i=0;
	unsigned char	TxData;
	
	TxData = TxTemp;

	//for unsigned char Sending
	for(i=0; i<8; i++) // __|-- times 8
	{
		I2Cm_SCL_Low();		
		I2Cm_delay(TX_DELAY/2);
		if(TxData & 0x80) 
		{
			I2Cm_SDA_High();
		}
		else 
		{
			 I2Cm_SDA_Low(); //I2Cm_I2CPRTDR= 0x00;//		
		}
		TxData = TxData<<1;
		I2Cm_delay(TX_DELAY/2);
		I2Cm_SCL_High();
//		while(!I2Cm_GetSCL()); //Clock Stretch	
		I2Cm_delay(TX_DELAY);
	}	
	
	//for ACK Checking
	I2Cm_SCL_Low();
	I2Cm_delay(TX_DELAY/2);
	I2Cm_SDA_High(); //release SDA pin open for reading ACK
	I2Cm_delay(ACK_DELAY);
	I2Cm_SCL_High();
//	while(!I2Cm_GetSCL()); //Clock Stretch	
	I2Cm_delay(ACK_DELAY);
	if(I2Cm_GetSDA())
	{
		return 1;
	}
	else 
		return 0;
}
    
//-----------------------------------------------------------------------------
//Function:	I2Cm_get
//Description:
//	Reads 1unsigned char data
//
//  ARGUMENTS:
//      TxData - Contains data to be transmitted.
//
//
//Returns:
//	read 1unsigned char data
//
unsigned char I2Cm_Get(void) //get MSB first
{
	unsigned char	i=0;
	unsigned char	RxTemp;
	//for unsigned char Reading	
	
	RxTemp = 0;
	for(i=0; i<8; i++) // __|-- times 8
	{		
		I2Cm_SCL_Low();		
		I2Cm_delay(RX_DELAY/2);
		RxTemp = RxTemp<<1;	
		I2Cm_SDA_High(); 	//release SDA pin open for reading one bit		
		I2Cm_delay(RX_DELAY/2);		
		I2Cm_SCL_High();
//		while(!I2Cm_GetSCL()); //Clock Stretch	
		if(I2Cm_GetSDA())
		{
			RxTemp |= 0x01;			
		}
		I2Cm_delay(RX_DELAY);
	}	
	return RxTemp;
}

//-----------------------------------------------------------------------------
//  FUNCTION NAME: I2Cm_ACKSend
//
//  DESCRIPTION:
//    Send acknowledge to slave. 
//
//-----------------------------------------------------------------------------
//  ARGUMENTS:
//    void
//
//  RETURNS:
//    void
//
void I2Cm_ACKSend(void)
{
	I2Cm_SCL_Low();	
	I2Cm_SDA_Low();	
	I2Cm_delay(ACK_DELAY);	
	I2Cm_SCL_High();
//	while(!I2Cm_GetSCL()); //Clock Stretch	
	I2Cm_delay(ACK_DELAY);		
}

//-----------------------------------------------------------------------------
//  FUNCTION NAME: I2Cm_ACKSend
//
//  DESCRIPTION:
//    Send acknowledge to slave. 
//
//-----------------------------------------------------------------------------
//  ARGUMENTS:
//    void
//
//  RETURNS:
//    void
//
void I2Cm_NAKSend(void)
{
	I2Cm_SCL_Low();	
	I2Cm_SDA_High();	
	I2Cm_delay(ACK_DELAY);	
	I2Cm_SCL_High();
//	while(!I2Cm_GetSCL()); //Clock Stretch	
	I2Cm_delay(ACK_DELAY);		
}

//-----------------------------------------------------------------------------
//  FUNCTION NAME: I2Cm_Start
//
//  DESCRIPTION:
//
//-----------------------------------------------------------------------------
//
//  ARGUMENTS: none
//
//  RETURNS: none

//-----------------------------------------------------------------------------
 void I2Cm_Start(void)
 {    
    // Set pins to drive mode OPEN DRAIN LOW
    I2Cm_Pin_Init();   
   // I2Cm_I2CPRTDM2|=(I2Cm_SDA_Pin|I2Cm_SCL_Pin);
    //I2Cm_I2CPRTDM1|=(I2Cm_SDA_Pin|I2Cm_SCL_Pin);
    //I2Cm_I2CPRTDM0|=(I2Cm_SDA_Pin|I2Cm_SCL_Pin); 
    
    
    // Setup port for normal operation    
    I2Cm_delay(START_DELAY);
    I2Cm_SDA_Low();
    I2Cm_delay(START_DELAY);
    I2Cm_SCL_Low();    
    I2Cm_delay(START_DELAY);
}

//-----------------------------------------------------------------------------
//  FUNCTION NAME: I2Cm_Stop
//
//  DESCRIPTION:
//    This function performs no operation and is used for future
//    module compatibility.
//
//-----------------------------------------------------------------------------
//
//  ARGUMENTS: none
//
//  RETURNS: none
//
void I2Cm_Stop(void)
 {      
    // Setup port for normal operation   
    I2Cm_SCL_Low();
    I2Cm_delay(STOP_DELAY); 
    I2Cm_SDA_Low();
    I2Cm_delay(STOP_DELAY);
    I2Cm_SCL_High();     
    //while(!I2Cm_GetSCL()); //Clock Stretch	
    I2Cm_delay(STOP_DELAY);
    I2Cm_SDA_High();  
    I2Cm_delay(STOP_DELAY); 
}    

//-----------------------------------------------------------------------------
//  FUNCTION NAME: I2Cm_delay
//
//  DESCRIPTION:
//    Create delays for I2Cm routines.
//
//-----------------------------------------------------------------------------
//
//  ARGUMENTS: 
//	Delay_Counter
//
//  RETURNS: none
//      
void I2Cm_delay(unsigned char I2Cm_DlyCnt)
{
	while(I2Cm_DlyCnt--);
}



