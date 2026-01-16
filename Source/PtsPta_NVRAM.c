/****************************************************************************
 * @file     main.c
 * @version  V3.0
 * $Revision: 5 $
 * $Date: 15/05/22 2:05p $
 * @brief    Perform A/D Conversion with ADC single mode.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "M051Series.h"
#include "PtsPta_TypeDef.h"
#include "PtsPta_Global.h"


#if 0

unsigned char CheckEepromType(void)
{
	unsigned char L_EepromType = eEEPROM_Type_NONE;		
	unsigned int	L_Count = 0;	

	SystemCoreClock = CLK_GetHCLKFreq(); 	

		I2C_Open(I2C0, 100000);			
		I2C_Trigger(I2C0, 1, 0, 1, 0);      //set start	
	//	Delayms(5);
		//printf("11=%X\n",(I2C0->I2CSTATUS)); //Normal : 0x08 Fail : 0.F8
		while (((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0)             //poll si flag
		{
			
				++L_Count;
				if( L_Count >= 10000 )
				{				
						break;
				}
		}
		//printf("22=%d \n",L_Count);		
		L_Count = 0;		
////////////////////////////////////////////////////////////
		I2C0->I2CDAT = 0XA0;
		I2C_Trigger(I2C0, 0, 0, 1, 0);	   
		
		while(((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )			
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
						break;
				}
		}
//		printf("33=%d \n",L_Count);			
		L_Count = 0;
CLK_SysTickLongDelay(300);
//		printf("44=%X\n",(I2C0->I2CSTATUS)); 

		if(I2C0->I2CSTATUS == 0x20)
		{
			CLK_SysTickLongDelay(600);
			I2C_Trigger(I2C0, 0, 1, 1, 0);      //set start	
			L_EepromType = eEEPROM_Type_NONE;
			return L_EepromType;
		}
/////////////////////////////////////////////////////////



		I2C0->I2CDAT = 0x00;	
		I2C_Trigger(I2C0, 0, 0, 1, 1); //clr si and set ack	
		while(((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )			//poll si flag
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
					//	L_back = 1;
						break;
				}		
		}		
		L_Count = 0;
		
CLK_SysTickLongDelay(300);
//		printf("55=%X\n",(I2C0->I2CSTATUS)); 		



		I2C0->I2CDAT = 0x01;	
		I2C_Trigger(I2C0, 0, 0, 1, 1); //clr si and set ack	
		while(((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )			//poll si flag
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
					//	L_back = 1;
						break;
				}		
		}		
		L_Count = 0;
		
CLK_SysTickLongDelay(300);
	//	printf("66=%X\n",(I2C0->I2CSTATUS)); 		
/////////////////////////////////////////////////////////////////////////


		I2C_Trigger(I2C0, 1, 0, 1, 0);    //clr si and send start	
		while(((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )			   //poll si flag
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
					//	L_back = 1;
						break;
				}
		}
		L_Count = 0;		


CLK_SysTickLongDelay(300);
	//	printf("77=%X\n",(I2C0->I2CSTATUS)); 	
/////////////////////////////////////////////////////////////////////////
		I2C0->I2CDAT = 0XA1;
		I2C_Trigger(I2C0, 0, 0, 1, 1);    //clr si
		while(((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )			   //poll si flag
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
					//	L_back = 1;
						break;
				}
		}
		L_Count = 0;	


CLK_SysTickLongDelay(3000);
//		printf("88=%X\n",(I2C0->I2CSTATUS)); 






		
		//resive data
		I2C0->I2CDAT = 0XFF;
		I2C_Trigger(I2C0, 0, 0, 1, 0);    //clr si	
		while(((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )			   //poll si flag
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
	//					L_back = 1;
						break;
				}
		}
		L_Count = 0;
		
	//	TEMP= I2C0->I2CDAT;
CLK_SysTickLongDelay(3000);
//		printf("99=%X\n",(I2C0->I2CSTATUS)); 

//printf("AA=%X\n",(I2C0->I2CDAT)); 

		if( I2C0->I2CDAT == 0x51 || I2C0->I2CDAT == 0x61)
			L_EepromType = eEEPROM_Type_24LC64;
		else if(I2C0->I2CDAT == 0xC1|| I2C0->I2CDAT == 0xC0)
			L_EepromType = eEEPROM_Type_24LC02;
		else 
			L_EepromType = eEEPROM_Type_NONE;






//////////////////////////////////////////////////////////////////
CLK_SysTickLongDelay(10);
		I2C_Trigger(I2C0, 0, 1, 1, 0);      //set start	




//printf(" \n\n\n");

 return L_EepromType;
}

void Write_24LC64( unsigned int address, unsigned char data, unsigned char *EEPROM_STATUS )
{
	unsigned int ii =0;
	unsigned char L_back = 0;
	unsigned int	L_Count = 0;
	
	while(1)
	{
	
		SystemCoreClock =   CLK_GetHCLKFreq(); 
		//Open I2C1 and set clock = 50Kbps 
	//	DrvI2C_Open(I2C_PORT0, 50000);
		I2C_Open(I2C0, 100000);
		
		//send i2c start
		I2C_Trigger(I2C0, 1, 0, 0, 0);	//set start
		while (((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0)			//poll si flag
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
						L_back = 1;
						break;
				}		
		}		
		L_Count = 0;
		
		//send writer command
		I2C0->I2CDAT = 0XA0;					//send writer command
		I2C_Trigger(I2C0, 0, 0, 1, 0); //clr si flag
		while(((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )		    //poll si flag
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
						L_back = 1;
						break;
				}		
		}		
		L_Count = 0;
		
		//send address high
		I2C0->I2CDAT = (address>>8)&0XFF;	
		I2C_Trigger(I2C0, 0, 0, 1, 1); //clr si and set ack	
		while(((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )			//poll si flag
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
						L_back = 1;
						break;
				}		
		}		
		L_Count = 0;
		
		//send address low
		I2C0->I2CDAT = address&0XFF;	
		I2C_Trigger(I2C0, 0, 0, 1, 1); //clr si and set ack	
		while(((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )			//poll si flag
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
						L_back = 1;
						break;
				}		
		}		
		L_Count = 0;
		
		//I2C slave device send Nack
		if(I2C0->I2CSTATUS==0x20)
			*EEPROM_STATUS = 0x20;
		else
			*EEPROM_STATUS = 0xff;
		
		//send data
		I2C0->I2CDAT = data;					//write data to 
		I2C_Trigger(I2C0, 0, 0, 1, 1); //clr si and set ack	
		while(((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )			//poll si flag
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
						L_back = 1;
						break;
				}		
		}
		L_Count = 0;
		
		//I2C slave device send Nack
		if(I2C1->I2CSTATUS==0x30)
			*EEPROM_STATUS = 0x30;
		else
			*EEPROM_STATUS = 0xff;
		
		//send i2c stop
		I2C_Trigger(I2C0, 0, 1, 1, 0); //send stop	
		while( I2C0->I2CON &  I2C_I2CON_STO_Msk)
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
						L_back = 1;
						break;
				}
		}
		L_Count = 0;
		
		//while( I2C1->CON.SI == 0 );
		for(ii=0;ii<60;ii++);
		I2C_Close(I2C0);
			
		for(ii=0;ii<6000;ii++);
		for(ii=0;ii<6000;ii++);
		
	//	L_back = 1;
		
		if( L_back )	
		{
				L_back = 0;
				continue;
		}
		else
				break;
	}
}


/* I2C_PORT0 */
unsigned char  Read_24LC64( unsigned int address, unsigned char *EEPROM_STATUS )
{
	unsigned char TEMP;
	unsigned char L_back = 0;
	unsigned int	L_Count = 0;	

	while(1)
	{
		
		//Open I2C1 and set clock = 50Kbps
		SystemCoreClock = CLK_GetHCLKFreq(); 
	//	DrvI2C_Open(I2C_PORT0, 50000);
		I2C_Open(I2C0, 100000);
		
		//send i2c start
		I2C_Trigger(I2C0, 1, 0, 1, 0);      //set start
		while (((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0)             //poll si flag
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
						L_back = 1;
						break;
				}
		}
		L_Count = 0;		

		//send writer command
		I2C0->I2CDAT = 0XA0;
		I2C_Trigger(I2C0, 0, 0, 1, 0);	   //clr si
		while(((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )			   //poll si flag
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
						L_back = 1;
						break;
				}
		}
		L_Count = 0;		
		
		//send address high
		I2C0->I2CDAT = (address>>8)&0XFF;	
		I2C_Trigger(I2C0, 0, 0, 1, 1);    //clr si and set ack
		while(((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )			   //poll si flag
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
						L_back = 1;
						break;
				}
		}
		L_Count = 0;		
			
		//send address low
		I2C0->I2CDAT = address&0XFF;	
		I2C_Trigger(I2C0, 0, 0, 1, 0);    //clr si	and set ack
		while(((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )			   //poll si flag
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
						L_back = 1;
						break;
				}
		}
		L_Count = 0;		
		
		//send start flag
		I2C_Trigger(I2C0, 1, 0, 1, 0);    //clr si and send start	
		while(((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )			   //poll si flag
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
						L_back = 1;
						break;
				}
		}
		L_Count = 0;		
		
		//send read command
		I2C0->I2CDAT = 0XA1;
		I2C_Trigger(I2C0, 0, 0, 1, 1);    //clr si
		while(((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )			   //poll si flag
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
						L_back = 1;
						break;
				}
		}
		L_Count = 0;		

		//I2C slave device send Nack
		if(I2C1->I2CSTATUS==0x48)
			*EEPROM_STATUS = 0x48;
		else
			*EEPROM_STATUS = 0xff;
		
		//resive data
		I2C0->I2CDAT = 0XFF;
		I2C_Trigger(I2C0, 0, 0, 1, 0);    //clr si	
		while(((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )			   //poll si flag
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
						L_back = 1;
						break;
				}
		}
		L_Count = 0;
		
		TEMP= I2C0->I2CDAT;
		
		//send i2c stop
		I2C_Trigger(I2C0, 0, 1, 1, 0);    //clr si and set stop
		while( I2C0->I2CON &  I2C_I2CON_STO_Msk )
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
						L_back = 1;
						break;
				}
		}
		L_Count = 0;
		
		I2C_Close(I2C0);
		
		if( L_back )	
		{
				L_back = 0;
				continue;
		}
		else
				break;
	}	

	return TEMP; 
}


unsigned char CheckVoltage_CurrentAndNVRAM(void)
{
		unsigned int L_Cell1V, L_Cell2V, L_Cell3V;
#if (FourCell_14V4)
		unsigned int L_Cell4V;
#endif		
		unsigned char   L_Cell1VGap,	L_Cell2VGap,	L_Cell3VGap;
#if (FourCell_14V4)
		unsigned char   L_Cell4VGap;
#endif			
		Delayms(100);


	#if(ThreeCell_10V8)
		printf("Current Battery 1=%d, 2=%d, 3=%d\n",Battery_Voltage_Cell_1,Battery_Voltage_Cell_2,Battery_Voltage_Cell_3);
	#elif (FourCell_14V4)
		printf("Current Battery 1=%d, 2=%d, 3=%d, 4=%d\n",Battery_Voltage_Cell_1,Battery_Voltage_Cell_2,Battery_Voltage_Cell_3,Battery_Voltage_Cell_4);
	#endif
		
		

	if(G_EEPROM_TYPE == eEEPROM_Type_24LC64)
	{	

		L_Cell1V = Read_24LC64(NVRM_Addr_Cell1Voltage+0,&G_Error_EEPROM) << 8;
		L_Cell1V = L_Cell1V + Read_24LC64(NVRM_Addr_Cell1Voltage+1,&G_Error_EEPROM);

		L_Cell2V = Read_24LC64(NVRM_Addr_Cell2Voltage+0,&G_Error_EEPROM) << 8;
		L_Cell2V = L_Cell2V + Read_24LC64(NVRM_Addr_Cell2Voltage+1,&G_Error_EEPROM);

		L_Cell3V = Read_24LC64(NVRM_Addr_Cell3Voltage+0,&G_Error_EEPROM) << 8;
		L_Cell3V = L_Cell3V + Read_24LC64(NVRM_Addr_Cell3Voltage+1,&G_Error_EEPROM);
	}
	else
	{

		L_Cell1V = Read_24LC02(NVRM_Addr_Cell1Voltage+0,&G_Error_EEPROM) << 8;
		L_Cell1V = L_Cell1V + Read_24LC02(NVRM_Addr_Cell1Voltage+1,&G_Error_EEPROM);

		L_Cell2V = Read_24LC02(NVRM_Addr_Cell2Voltage+0,&G_Error_EEPROM) << 8;
		L_Cell2V = L_Cell2V + Read_24LC02(NVRM_Addr_Cell2Voltage+1,&G_Error_EEPROM);

		L_Cell3V = Read_24LC02(NVRM_Addr_Cell3Voltage+0,&G_Error_EEPROM) << 8;
		L_Cell3V = L_Cell3V + Read_24LC02(NVRM_Addr_Cell3Voltage+1,&G_Error_EEPROM);


	}

		

#if (FourCell_14V4)
		L_Cell4V = Read_24LC64(NVRM_Addr_Cell4Voltage+0,&G_Error_EEPROM) << 8;
		L_Cell4V = L_Cell4V + Read_24LC64(NVRM_Addr_Cell4Voltage+1,&G_Error_EEPROM);
#endif



	//	printf("NVRAM 1=%d 2=%d 3=%d 4=%d\n",L_Cell1V , L_Cell2V , L_Cell3V , L_Cell4V);

		if(Battery_Voltage_Cell_1 > L_Cell1V)
		{
			L_Cell1VGap = Battery_Voltage_Cell_1 - L_Cell1V;
		}
		else
		{
			L_Cell1VGap = L_Cell1V - Battery_Voltage_Cell_1;
		}

		if(Battery_Voltage_Cell_2 > L_Cell2V)
		{
			L_Cell2VGap = Battery_Voltage_Cell_2 - L_Cell2V;
		}
		else
		{
			L_Cell2VGap = L_Cell2V - Battery_Voltage_Cell_2;
		}

		if(Battery_Voltage_Cell_3 > L_Cell3V)
		{
			L_Cell3VGap = Battery_Voltage_Cell_3 - L_Cell3V;
		}
		else
		{
			L_Cell3VGap = L_Cell3V - Battery_Voltage_Cell_3;
		}

#if (FourCell_14V4)
		if(Battery_Voltage_Cell_4 > L_Cell4V)
		{
			L_Cell4VGap = Battery_Voltage_Cell_4 - L_Cell4V;
		}
		else
		{
			L_Cell4VGap = L_Cell4V - Battery_Voltage_Cell_4;
		}
#endif



		

#if(ThreeCell_10V8)
		printf("VGap  1=%d 2=%d 3=%d \n",L_Cell1VGap, L_Cell2VGap , L_Cell3VGap );
#elif (FourCell_14V4)
		printf("VGap  1=%d 2=%d 3=%d 4=%d\n",L_Cell1VGap, L_Cell2VGap , L_Cell3VGap , L_Cell4VGap);
#endif	



#if(ThreeCell_10V8)
		if((L_Cell1VGap < 7) && (L_Cell2VGap < 7) && (L_Cell3VGap < 7) )
#elif (FourCell_14V4)
		if((L_Cell1VGap < 7) && (L_Cell2VGap < 7) && (L_Cell3VGap < 7) && (L_Cell4VGap < 7))
#endif			
			return TRUE;
		else
			return FALSE; 






}


#endif

/* I2C_PORT0 */
void Write_24LC02( unsigned char address, unsigned char data, unsigned char *EEPROM_STATUS )
{
	unsigned int ii =0;
	unsigned char L_back = 0;
	unsigned int	L_Count = 0;
	
	while(1)
	{
	
		SystemCoreClock = CLK_GetHCLKFreq(); 
		//Open I2C1 and set clock = 50Kbps 
	//	DrvI2C_Open(I2C_PORT0, 50000);
		I2C_Open(I2C0, 100000);
		
		//send i2c start
		I2C_Trigger(I2C0, 1, 0, 0, 0);	//set start
		while ( ((I2C0->I2CON) & I2C_I2CON_SI_Msk)  == 0)			//poll si flag
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
						L_back = 1;
						break;
				}		
		}	
		L_Count = 0;

		//send writer command
		I2C0->I2CDAT = 0XA0;					//send writer command
		I2C_Trigger(I2C0, 0, 0, 1, 0); //clr si flag
		while( ((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )		    //poll si flag
		{
				++L_Count;
				if( L_Count >= 10000 )
				{						
						L_back = 1;
						break;
				}		
		}		

		L_Count = 0;


	//	printf("1STUS =%X\n",(I2C0->I2CSTATUS));
		//I2C slave device send Nack
		if((I2C0->I2CSTATUS)==0x20)
			*EEPROM_STATUS = 0x20; //Master Transmit Address NACK
		else
			*EEPROM_STATUS = 0xff;







		
	
		//send address data
		I2C0->I2CDAT = address&0XFF;	
		I2C_Trigger(I2C0, 0, 0, 1, 1); //clr si and set ack	
		while( ((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )			//poll si flag
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
						L_back = 1;
						break;
				}		
		}		
		L_Count = 0;


	//	printf("2STUS =%X\n",(I2C0->I2CSTATUS));
		if((I2C0->I2CSTATUS)==0x30)
			*EEPROM_STATUS = 0x30; // Master Transmit Data NACK
		else
			*EEPROM_STATUS = 0xff;





	
		
		
		//send data
		I2C0->I2CDAT = data;					//write data to 
		I2C_Trigger(I2C0, 0, 0, 1, 1); //clr si and set ack	
		while( ((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )			//poll si flag
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
						L_back = 1;
						break;
				}		
		}
		L_Count = 0;


	//	printf("3STUS =%X\n",(I2C0->I2CSTATUS));
		if((I2C0->I2CSTATUS)==0x30)
			*EEPROM_STATUS = 0x30; // Master Transmit Data NACK
		else
			*EEPROM_STATUS = 0xff;








		
		//send i2c stop
		I2C_Trigger(I2C0, 0, 1, 1, 0); //send stop	
		while( I2C0->I2CON &  I2C_I2CON_STO_Msk )
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
						L_back = 1;
						break;
				}
		}
		L_Count = 0;

		//while( I2C1->CON.SI == 0 );
		for(ii=0;ii<60;ii++);
		I2C_Close(I2C0);
			
		for(ii=0;ii<6000;ii++);
		for(ii=0;ii<6000;ii++);
		
	//	L_back = 1;
		
		if( L_back )	
		{
				L_back = 0;
				continue;
		}
		else
				break;
	}
}

unsigned char Read_24LC02( unsigned char  address, unsigned char *EEPROM_STATUS )
{
	unsigned char TEMP;
	unsigned char L_back = 0;
	unsigned int	L_Count = 0;	

	while(1)
	{
			
			//Open I2C1 and set clock = 50Kbps
			SystemCoreClock = CLK_GetHCLKFreq(); 

			I2C_Open(I2C0, 100000);
			
			//send i2c start
			I2C_Trigger(I2C0, 1, 0, 1, 0);      //set start
			while (((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0)             //poll si flag
			{
					++L_Count;
					if( L_Count >= 10000 )
					{
							L_back = 1;
							break;
					}
			}
			L_Count = 0;		


			//send writer command
			I2C0->I2CDAT = 0XA0;
			I2C_Trigger(I2C0, 0, 0, 1, 0);	   //clr si
			while( ((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )			   //poll si flag
			{
					++L_Count;
					if( L_Count >= 10000 )
					{
							L_back = 1;
							break;
					}
			}
			L_Count = 0;		




//		printf("1STUS =%X\n",(I2C0->I2CSTATUS));

		if((I2C0->I2CSTATUS)==0x20)
			*EEPROM_STATUS = 0x20; //Master Transmit Address NACK
		else
			*EEPROM_STATUS = 0xff;




#if 1



			//send address low
			I2C0->I2CDAT = address&0XFF;	
			I2C_Trigger(I2C0, 0, 0, 1, 0);    //clr si	and set ack
			while( ((I2C0->I2CON) & I2C_I2CON_SI_Msk)== 0 )			   //poll si flag
			{
					++L_Count;
					if( L_Count >= 10000 )
					{
							L_back = 1;
							break;
					}
			}
			L_Count = 0;		



//		printf("2STUS =%X\n",(I2C0->I2CSTATUS));

		if((I2C0->I2CSTATUS)==0x30)
			*EEPROM_STATUS = 0x30; //Master Transmit Address NACK
		else
			*EEPROM_STATUS = 0xff;



			
			//send start flag
			I2C_Trigger(I2C0, 1, 0, 1, 0);    //clr si and send start	
			while(((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )			   //poll si flag
			{
					++L_Count;
					if( L_Count >= 10000 )
					{
							L_back = 1;
							break;
					}
			}
			L_Count = 0;		




//		printf("3STUS =%X\n",(I2C0->I2CSTATUS));



			
			//send read command
			I2C0->I2CDAT = 0XA1;
			I2C_Trigger(I2C0, 0, 0, 1, 1);    //clr si
			while( ((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )			   //poll si flag
			{
					++L_Count;
					if( L_Count >= 10000 )
					{
							L_back = 1;
							break;
					}
			}
			L_Count = 0;		




//		printf("4STUS =%X\n",(I2C0->I2CSTATUS));
		
		if((I2C0->I2CSTATUS)==0x48)
			*EEPROM_STATUS = 0x48; //Master Receive Address NACK
		else
			*EEPROM_STATUS = 0xff;

			
			//resive data
			I2C0->I2CDAT = 0XFF;
			I2C_Trigger(I2C0, 0, 0, 1, 0);    //clr si	
			while( ((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )			   //poll si flag
			{
					++L_Count;
					if( L_Count >= 10000 )
					{
							L_back = 1;
							break;
					}
			}
			L_Count = 0;




	
			TEMP= I2C0->I2CDAT;


//		printf("5STUS =%X\n",(I2C0->I2CSTATUS));


/*
		if((I2C0->I2CSTATUS)==0x58)
			*EEPROM_STATUS = 0x58; //Master Receive Data NACK
		else
			*EEPROM_STATUS = 0xff;

*/


			
			
			//send i2c stop
			I2C_Trigger(I2C0, 0, 1, 1, 0);    //clr si and set stop
			while(( I2C0->I2CON )& I2C_I2CON_STO_Msk)
			{
					++L_Count;
					if( L_Count >= 10000 )
					{
							L_back = 1;
							break;
					}
			}
			L_Count = 0;
#endif			
			I2C_Close(I2C0);
			
			if( L_back )	
			{
					L_back = 0;
					continue;
			}
			else
					break;
		}
  return TEMP;
}












unsigned char CHECK_ID(void)
{
		                                                                                /* BS */	    

//		printf("RICO=%X, ",Read_24LC02(NVRM_Addr_RICO, &G_Error_EEPROM));
//		printf("Type=%X, ",Read_24LC02(NVRM_Addr_TYPE, &G_Error_EEPROM));
//		printf("Layer=%X, ",Read_24LC02(NVRM_Addr_Layer, &G_Error_EEPROM));
//		printf("Cell=%X, ",Read_24LC02(NVRM_Addr_Cell, &G_Error_EEPROM));
//		printf("Check=%X \n",Read_24LC02(NVRM_Addr_CHECK, &G_Error_EEPROM));


#if(ScrewType == _BS_1Nm2)
		if( (Read_24LC02(NVRM_Addr_RICO, &G_Error_EEPROM) == 0x41)   &&  (Read_24LC02(NVRM_Addr_TYPE, &G_Error_EEPROM) == 0x20) 
			&&  (Read_24LC02(NVRM_Addr_Layer, &G_Error_EEPROM) == 0x01)&&  (Read_24LC02(NVRM_Addr_Cell, &G_Error_EEPROM) == 0x02)  )
#else																						/* BE */
		if( (Read_24LC02(NVRM_Addr_RICO, &G_Error_EEPROM) == 0xC1)   &&  (Read_24LC02(NVRM_Addr_TYPE, &G_Error_EEPROM) == 0x40)  )
#endif

		
		{
			
				return _OK;
		}		
		else
		{
	
				return _NG;
		}




		
}


uint8_t CheckBatteryType(uint8_t DevAddr)
{
	uint8_t L_EepromType = eEEPROM_Type_NONE;
	unsigned int	L_Count = 0;	

	SystemCoreClock = CLK_GetHCLKFreq(); 	

		I2C_Open(I2C0, 100000);			
		I2C_Trigger(I2C0, 1, 0, 1, 0);      //set start	
	//	Delayms(5);
		//printf("11=%X\n",(I2C0->I2CSTATUS)); //Normal : 0x08 Fail : 0.F8
		while (((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0)             //poll si flag
		{
			
				++L_Count;
				if( L_Count >= 10000 )
				{				
						break;
				}
		}
		//printf("22=%d \n",L_Count);		
		L_Count = 0;		
////////////////////////////////////////////////////////////

I2C0->I2CDAT = DevAddr;
		I2C_Trigger(I2C0, 0, 0, 1, 0);	   
		
		while(((I2C0->I2CON) & I2C_I2CON_SI_Msk) == 0 )			
		{
				++L_Count;
				if( L_Count >= 10000 )
				{
						break;
				}
		}
//		printf("33=%d \n",L_Count);			
		L_Count = 0;
CLK_SysTickLongDelay(300);
		
//		printf("44=%X\n",(I2C0->I2CSTATUS)); 
		if( I2C0->I2CSTATUS == 0xF8 )			
		{
			printf("IIC error Bus Release\n");
			CLK_SysTickLongDelay(600);
			
			SYS_ResetModule(I2C0_RST);
			
			L_EepromType = eEEPROM_Type_IICBusError;
			return L_EepromType;

		}
		else if(I2C0->I2CSTATUS == 0x20)
		{
//			printf("IIC DevAddr  NACK \n");
			CLK_SysTickLongDelay(600);
			I2C_Trigger(I2C0, 0, 1, 1, 0);      //set start	
			L_EepromType = eEEPROM_Type_NONE;
			//G_IICAckErrorTimes++;
			return L_EepromType;
		}
		else if(I2C0->I2CSTATUS == 0x18 )	
		{
			printf("Nvram tyep OK \n");
			
			CLK_SysTickLongDelay(600);
			I2C_Trigger(I2C0, 0, 1, 1, 0);      //set start	

			if(DevAddr == eIICType_Nvram)
			{				
				L_EepromType = eEEPROM_Type_24LC64;
				return L_EepromType;
			}
			else if(DevAddr == eIICType_MCU)
			{
				L_EepromType = eEEPROM_Type_MCU;
				return L_EepromType;

			}

		}
	L_EepromType = eEEPROM_Type_NONE;
	return L_EepromType;


}




