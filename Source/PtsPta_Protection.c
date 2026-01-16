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
#include "M051Series.h"
#include "string.h"
#include "PtsPta_TypeDef.h"
#include "PtsPta_Global.h"
#include "PtsPta_Motor.h"


#if 1
void PCB_Temperature_Check(void)
{
	static unsigned int S_TempOver_80_Cnt = 0;
	static unsigned int S_TempUnder_70_Cnt = 0;

	if(G_PCBStatus == ePCBStatus_Normal) // 0
	{
		if(G_NTC_PCB > PCB_TEMP_80)
		{				
			S_TempOver_80_Cnt++;
			
			if(S_TempOver_80_Cnt >= _2S_By5ms)
			{
				S_TempOver_80_Cnt = 0;	S_TempUnder_70_Cnt =0;
				printf("*PCB_T>80'C*\n");	
				G_ScrewRPM = 1;
				C_NG_RED_OFF;
				G_BuzzerLED = eBehaving_TempOverHeat;
				Motor(DISABLE, G_Direction);
				G_PCBStatus = ePCBStatus_Over80;

			}
		}
		else S_TempOver_80_Cnt =0;
	}
	else if(G_PCBStatus == ePCBStatus_Over80 )
	{
		if(G_NTC_PCB < PCB_TEMP_70)
		{
			S_TempUnder_70_Cnt++;
			if(S_TempUnder_70_Cnt > _2S_By5ms)
			{
				S_TempOver_80_Cnt = 0;	S_TempUnder_70_Cnt =0;
				printf("*PCB_T<70'C*\n");	
				G_BuzzerLED = eBehaving_OFF;
				G_PCBStatus = ePCBStatus_Normal;	
			}						
		}
		else S_TempUnder_70_Cnt = 0;
	}

}
//5ms
void Motor_Temperature_Check(void)
{

	static unsigned int S_TempOver_100_Cnt = 0;
//	static unsigned int S_TempBetween90_100_Cnt = 0;
	static unsigned int S_TempUnder_90_Cnt = 0;

	if(G_MotorStatus == eMotorStatus_Normal) // 0
	{
		if(G_NTC_Motor >= MOTOR_TEMP_100)
		{			
			printf("Motor Temp=%d,\n",S_TempOver_100_Cnt);
			S_TempOver_100_Cnt++;

			if(S_TempOver_100_Cnt >= _2S_By5ms)
			{		
				printf("*Motor_T>100'C*\n");	
			
				G_ScrewRPM = 1;
				C_NG_RED_OFF;
				G_BuzzerLED = eBehaving_TempOverHeat;
				Motor(DISABLE, G_Direction);
				G_MotorStatus = eMotorStatus_Over100;

			}
		}
	}
	else if(G_MotorStatus == eMotorStatus_Over100)// 1
	{

		if(G_NTC_Motor > MOTOR_TEMP_90 && G_NTC_Motor < MOTOR_TEMP_100 && G_BuzzerLED == eBehaving_OFF)
		{
	//		S_TempOver_100_Cnt = 0;
	//		S_TempBetween90_100_Cnt ++;

	//		if( S_TempBetween90_100_Cnt > _1S_By5ms)
			{

				G_BuzzerLED = eBehaving_MotorTemp_90_100;
//				G_MotorStatus = eMotorStatus_Over90_100;
			}
		}
		else if(G_NTC_Motor < MOTOR_TEMP_90)
		{
			G_BuzzerLED = eBehaving_OFF;
			S_TempOver_100_Cnt = 0;
		//	S_TempBetween90_100_Cnt = 0;
			S_TempUnder_90_Cnt = 0;			
			G_MotorStatus = eMotorStatus_Normal;
		}
	}
	else if(G_MotorStatus == eMotorStatus_Over90_100) // 2
	{		
		if(G_NTC_Motor < MOTOR_TEMP_90)
			S_TempUnder_90_Cnt ++;
	
		if(S_TempUnder_90_Cnt >= _3S_By5ms)			
		{
			G_MotorStatus = eMotorStatus_Normal;
			G_BuzzerLED = eBehaving_OFF;

			S_TempOver_100_Cnt = 0;
		//	S_TempBetween90_100_Cnt = 0;
			S_TempUnder_90_Cnt = 0;
		}
	}	

}

//5ms
void OverCurrent_Check(void)
{
	static unsigned int S_OverCurrentShortCnt = 0;
	static unsigned int S_OverCurrentOverLoadCnt = 0;
	static unsigned int S_OverCurrentProtectCnt = 0;
	
	S_OverCurrentShortCnt++;
	S_OverCurrentOverLoadCnt++;
	S_OverCurrentProtectCnt++;
	
	//if(G_ClutchBrakeStatus == ON)
	//	return;
	
	if(G_Current >= OVER_CURRENT_SHORT_MAX) //1500
	{
		if(S_OverCurrentShortCnt > OVER_CURRENT_SHORT_MAX_DELAY) //30 ms
		{
			Motor(DISABLE, G_Direction);
			printf("I1\n");
			G_BuzzerLED = eBehaving_CurrentOver;   
			G_MotorStatus = eMotorStatus_OverCurrent;
			S_OverCurrentShortCnt = OVER_CURRENT_SHORT_MAX_DELAY;
		}
	}
	else
	{
		S_OverCurrentShortCnt = 0;
	}	
		
	if(G_Current >= OVER_CURRENT_OVERLOAD) //910
	{
		if(S_OverCurrentOverLoadCnt > OVER_CURRENT_OVERLOAD_DELAY) //100 ms
		{
			Motor(DISABLE, G_Direction);
			printf("I2\n");
			G_BuzzerLED = eBehaving_CurrentOver;  
			G_MotorStatus = eMotorStatus_OverCurrent;
			S_OverCurrentOverLoadCnt = OVER_CURRENT_OVERLOAD_DELAY;
			
		}
	}
	else
	{
		S_OverCurrentOverLoadCnt = 0;
	}
	
	if(G_Current >= OVER_CURRENT_PROTECT) // 180
	{
		if(S_OverCurrentProtectCnt > OVER_CURRENT_PROTECT_DELAY) // 600 ms
		{
			Motor(DISABLE, G_Direction);
			printf("I3\n");
			G_BuzzerLED = eBehaving_CurrentOver;  	
			G_MotorStatus = eMotorStatus_OverCurrent;
			S_OverCurrentProtectCnt = OVER_CURRENT_PROTECT_DELAY;
		}
	}
	else
	{
		S_OverCurrentProtectCnt = 0;
	}
}




void BatteryNTC_Check(void)
{
	static unsigned int S_BatteryNormalCnt = 0;
	static unsigned int S_BatteryErrorCnt = 0;
	

	if(G_BatteryStatus == eBatteryStatus_Normal)
	{
		
		if(G_NTC_Battery <  BatteryNTCMin || G_NTC_Battery >  BatteryNTCMax) 
		{
			S_BatteryErrorCnt ++;
			
			if(S_BatteryErrorCnt > OVER_BatteryNTC_MAX_DELAY) //500 ms
			{
				G_BuzzerLED = eBehaving_BatteryNTCErr;
				G_BatteryStatus = eBatteryStatus_Error;
				Motor(DISABLE, G_Direction);
				printf("B-NTC Err = %d\n",G_NTC_Battery);
				S_BatteryErrorCnt = OVER_BatteryNTC_MAX_DELAY;
			}
		}
		else
		{
			S_BatteryErrorCnt = 0;
		}	
	}
	else if(G_BatteryStatus == eBatteryStatus_Error)
	{//Error Status
		if(G_NTC_Battery > BatteryNTCMin && G_NTC_Battery < BatteryNTCMax)
		{
			S_BatteryNormalCnt ++;	
			if(S_BatteryNormalCnt > OVER_BatteryNTC_MAX_DELAY)
			{
				G_BatteryStatus = eBatteryStatus_Normal;

				printf("B-NTC OK\n");
				S_BatteryNormalCnt = OVER_BatteryNTC_MAX_DELAY;
			}


		}
		else
		{

			S_BatteryNormalCnt = 0;
		
		}



	}


}

#endif



