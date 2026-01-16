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
#include "PtsPta_FMC.h"
#include "PtsPta_Init.h"


#if 0
//20ms
/*                                           
---            --------            --------         -------- 
	|         | 500ms |         | 500ms |         | 500ms |
	|         |	          |         |	      |         |   	  |
	|_____|	          |_____|	      |_____|	         |
	 500ms                 500ms            500ms         


*/
void LowBatteryBehaving_10V5_11V(unsigned char  Value )
{
	static unsigned int S_Count = 0;
	S_Count++;

	if(Value == ON)
	{
		//printf("a%d\n",S_Count);
		if(S_Count < 25)
		{
			C_D2_Red_On;
			C_Buzzer_ON;
		}
		else if(S_Count >25 &&  S_Count < 50)
		{
			C_D2_Red_Off;	
			C_Buzzer_OFF;

		}
		else if(S_Count > 50)
		{
			S_Count = 0;
		}
	}
	else
	{
	//	printf("b%d\n",S_Count);
		S_Count = 0;
		C_D2_Red_Off;	
		C_Buzzer_OFF;


	}


}
#endif
//5ms

static u8 		mu8BuzCycle;
void Buzzer_LED_Behaving( unsigned char  * Mode )
{
	static unsigned int S_Count = 0;
	static unsigned char S_BeforeMode=0;
	static unsigned char S_KeepFlg=0;


	if(S_BeforeMode != G_BuzzerLED)
		S_Count= 0;

	switch( *Mode)
	{

		case eBehaving_Idle_0s5:
					S_Count++;
				//printf("0.5=%d\n",S_Count);
					if(S_Count < 100) // 0.5s					
					{



					}
					else
					{
						S_Count = 0;
						* Mode  = eBehaving_LowBattery;
					}


			break;
	
		case eBehaving_Idle_1S:
					S_Count++;
				//	printf("1S=%d\n",S_Count);	
					C_BAT_RED_OFF;
					C_PtsPta_Buzzer_OFF;
					if(S_Count > 200)
					{
						Start_TMR_PWROff_3S();
						Start_TMR_PWROff_24S();
						if( G_Voltage_Battery > 1800)
						{
							C_PtsPta_Buzzer_ON;								
							* Mode  = eBehaving_BeOnce;
							S_Count = 0;
						}	
						else if(1730 <= G_Voltage_Battery && G_Voltage_Battery < 1800)
						{
							
							* Mode  = eBehaving_LowBattery;
						}
						else
						{
							C_PtsPta_Buzzer_ON;						
							* Mode  = eBehaving_BeOnce_OutofBattery;
							S_Count = 0;

						}	
					}	
					
			
			break;

		case eBehaving_BeOnce_OutofBattery:
					S_Count++;
			//		printf("Once & out=%d\n",S_Count);
				if( S_Count < 40)
				{
				
				}
				else if( 40 <= S_Count  && S_Count <= 80)
				{
					C_PtsPta_Buzzer_OFF;
				}
				else if(S_Count > 80)
				{
					C_BAT_RED_ON;
					C_PtsPta_Buzzer_ON;	
					
				}


			break;
		case eBehaving_BeOnce:
				S_Count++;
			//		printf("Once=%d\n",S_Count);
				if(S_Count > 40)
				{
					C_PtsPta_Buzzer_OFF;
					
					* Mode  = eBehaving_OFF;
				}
					

			
			break;
			
		case eBehaving_ConnectButIDNotPass:
		//	printf("IDx%d\n",S_Count);

			
				S_Count++;
				
				// if( mu16BuzCount < 184)	C_Buzzer_OFF; 
				//else 					C_Buzzer_ON;	
					
			/*
				if(mu16BuzCount > 206 )
				{
					mu16BuzCount = 1;
					mu8BuzCycle++;
					printf("		mu8BuzCycle = %d\n",mu8BuzCycle);

					if(mu8BuzCycle >= 60)	
					{
						printf("                                ** 60s  ID Error Power off **\n");
						C_PWRKeep_OFF;
					}					
				}	

*/
				

				

				if(S_Count < (260/5))
				{
					C_OK_GREEN_OFF;	C_BAT_RED_ON;		C_Buzzer_ON;							
				}
				else if(S_Count >= (260/5) && S_Count < (620/5))
				{
					C_OK_GREEN_ON;		C_BAT_RED_OFF;		C_Buzzer_OFF;
				}
				else if(S_Count >= (620/5) && S_Count < (880/5))
				{
					C_OK_GREEN_OFF;	C_BAT_RED_ON;		C_Buzzer_OFF;
				}
				else if(S_Count >= (880/5) && S_Count < (1230/5))
				{
					C_OK_GREEN_ON;		C_BAT_RED_OFF;		C_Buzzer_OFF;
				}
				
				if(S_Count >= (1230/5))	S_Count = 1;

				if(S_Count == (1000/5))
				{
					mu8BuzCycle++;		printf("		mu8BuzCycle = %d\n",mu8BuzCycle);	

					if(mu8BuzCycle >= 60)	
					{
						printf("                                ** 60s  ID Error Power off **\n");
						C_PWRKeep_OFF;
					}
				}

				
			
			break;	
		case eBehaving_MotorStall:
		case eBehaving_CurrentOver:	

			/*                                                                                         
                               ---	  -----	  -----   	    -----
					|     |80ms|     |80ms|     |
					|     |	   |     |	     |     |
					|___|        |___|       |___|
 					 80ms         80ms        80ms             */                                                                                         
			
						S_Count++;
						if(S_Count < 16)
						{
						//	C_D1_Red_On;
							C_Buzzer_ON;
						}
						else if(S_Count >= 16 && S_Count < 32)
						{
									C_D1_Red_Off;
									C_Buzzer_OFF;
						}
						else if(S_Count >= 32 && S_Count < 48)
						{
						//	C_D1_Red_On;
							C_Buzzer_ON;
						}
						else if(S_Count >= 48 && S_Count < 64)
						{
									C_D1_Red_Off;
									C_Buzzer_OFF;
						}
						else if(S_Count >= 64 && S_Count < 80)
						{
						//	C_D1_Red_On;
							C_Buzzer_ON;
						}
						
						if(S_Count >= 80)
							* Mode  = eBehaving_OFF;

			
			break;

		case eBehaving_PCBTemp_70_80:
		case eBehaving_MotorTemp_90_100:		

				S_Count++;

				
				if(S_Count < 100 )
				{
						
					#if ( ScrewType == _BS_PType_4Nm5 || ScrewType == _BS_PType_3Nm)	
						if(StartButton > Down  || S_KeepFlg == TRUE)			
					#else
						if(StartButton == Down || S_KeepFlg == TRUE)			
					#endif 
						{
							S_KeepFlg = TRUE;
							if((Get_TMR_PWROff_24S()*20) <5000000)
							{
								C_D1_Red_On;
								C_OK_GREEN_ON;
							}	
						}
				}
				else if(S_Count >= 100 && S_Count < 200)
				{
							C_D1_Red_Off;
							C_OK_GREEN_OFF;							
					
				}
		
				if(S_Count > 200)
					S_Count = 0;				
			
			break;
	
	//	case eBehaving_ClutchOK_300ms_G_LED_OFF:
	//			S_Count++;
	//			if(S_Count == 60)
	//			{
	//				C_D1_Green_Off;					
	//				* Mode  = eBehaving_OFF;
	//			}

	//			break;

#if 0
		case eBehaving_CurrentOver:						
		/*    ¹q ¬y «O Å@ 6  Án ÅT                                                                                                                                                             
                               ---	  -----	  -----   	    -----  	 -----  	  -----  	    -------
					|     |80ms|     |80ms|     |80ms|     |80ms|     |80ms|     |
					|     |	   |     |	     |     |	       |     |	  |     |	    |     |
					|___|        |___|       |___|        |___|       |___|       |___|
					 80ms         80ms        80ms         80ms        80ms        80ms
		*/                                                                                         
						;
						S_Count++;
						if(S_Count < 16)
						{
							C_D1_Red_On;
							C_Buzzer_ON;
						}
						else if(S_Count >= 16 && S_Count < 32)
						{
									C_D1_Red_Off;
									C_Buzzer_OFF;
						}
						else if(S_Count >= 32 && S_Count < 48)
						{
							C_D1_Red_On;
							C_Buzzer_ON;
						}
						else if(S_Count >= 48 && S_Count < 64)
						{
									C_D1_Red_Off;
									C_Buzzer_OFF;
						}
						else if(S_Count >= 64 && S_Count < 80)
						{
							C_D1_Red_On;
							C_Buzzer_ON;
						}
						else if(S_Count >= 80 && S_Count < 96)
						{
									C_D1_Red_Off;
									C_Buzzer_OFF;
						}
						else if(S_Count >= 96 && S_Count < 112)
						{
							C_D1_Red_On;
							C_Buzzer_ON;
						}
						else if(S_Count >= 112 && S_Count < 128)
						{
									C_D1_Red_Off;
									C_Buzzer_OFF;
						}
						else if(S_Count >= 128 && S_Count < 144)
						{
							C_D1_Red_On;
							C_Buzzer_ON;
						}
						else if(S_Count >= 144 && S_Count < 160)
						{
									C_D1_Red_Off;
									C_Buzzer_OFF;
						}
						else if(S_Count >= 160 && S_Count < 176)
						{
							C_D1_Red_On;
							C_Buzzer_ON;
						}
	
						
						if(S_Count >= 176)
							* Mode  = eBehaving_OFF;

					break;
	#endif
		case eBehaving_BatteryNTCErr:
		case eBehaving_TempOverHeat:
		
				/*                                                                                         
                               ---	  -----	  -----   	    -----  	 -----  	  -----  	    -----  	  -----  	 -----  	   -----
					|     |80ms|     |80ms|     |80ms|     |80ms|     |80ms|     |80ms |     |80ms|   |80ms|     |
					|     |	   |     |	     |     |	       |     |	  |     |	    |     |	        |     |       |	|        |	   | 
					|___|        |___|       |___|        |___|       |___|       |___|         |___|       |__|        |___|     
					 80ms         80ms        80ms         80ms        80ms        80ms           80ms        80ms      80ms     
				*/                                                                                         
			
						S_Count++;
						if(S_Count < (50/5))
						{
							//C_D1_Red_On;
							C_Buzzer_ON;
						}
						else if(S_Count >= (50/5) && S_Count < (100/5))
						{
									C_D1_Red_Off;
									C_Buzzer_OFF;
						}
						else if(S_Count >= (100/5)&& S_Count < (150/5))
						{
							//C_D1_Red_On;
							C_Buzzer_ON;
						}
						else if(S_Count >= (150/5) && S_Count < (200/5))
						{
									C_D1_Red_Off;
									C_Buzzer_OFF;
						}
						else if(S_Count >= (200/5) && S_Count < (250/5))
						{
							//C_D1_Red_On;
							C_Buzzer_ON;
						}
						else if(S_Count >= (250/5) && S_Count <(400/5))
						{
									C_D1_Red_Off;
									C_Buzzer_OFF;
						}
						
						if(S_Count >= (400/5)) S_Count = 0;
							
						//	* Mode  = eBehaving_OFF;
						

						
				break;
		
		case eBehaving_LowBattery:
				/*                                           
				---            --------            --------         -------- 
					|         | 500ms |         | 500ms |         | 500ms |
					|         |	          |         |	      |         |   	  |
					|_____|	          |_____|	      |_____|	         |
					 500ms                 500ms            500ms         
				*/

		//			printf("low=%d\n",S_Count);	

						S_Count++;
						if(S_Count < 54 )
						{	

							C_BAT_RED_ON;
							C_PtsPta_Buzzer_ON;
						}						
						else	 // 5		
						{
							C_BAT_RED_OFF;
							C_PtsPta_Buzzer_OFF;					
						}
						
						if(S_Count > 103)
							S_Count = 0;
				break;	
				
		case eBehaving_OutOfBattery:

			#if 1	
				/*
				-----	---------	 ---------	               ------
					|     | 410ms   |     | 410ms  |                    |410ms
					|     |	        |	 |	        |	  		|     
					|___|              |___|            |___________|     
					100ms             100ms                620ms          
				*/			
						S_Count++;

					if(S_Count < 20)//100ms
					{
						C_BAT_RED_ON;
						C_Buzzer_ON;
					}
					else if(S_Count >20 &&  S_Count < 102)//400ms
					{
						C_BAT_RED_ON;
						C_Buzzer_OFF;
					}
					else if(S_Count >102 &&  S_Count < 122)//100ms
					{
						C_BAT_RED_ON;
						C_Buzzer_ON;
					}
					else if(S_Count >122 &&  S_Count < 204)//400ms
					{
						C_BAT_RED_ON;
						C_Buzzer_OFF;
					}
					else if(S_Count >204 &&  S_Count < 328)//700ms
					{
						C_BAT_RED_ON;
						C_Buzzer_ON;
					}
					else if(S_Count >328 &&  S_Count < 410)//200ms
					{
						C_BAT_RED_ON;
						C_Buzzer_OFF;
					}
					
					if(S_Count > 410)
						S_Count = 0;
			#endif
			break;
			
		case eBehaving_OFF:
		//			printf("off\n");
		default:
		//			printf("def\n");
				C_D1_Red_Off;	
				C_NG_RED_OFF;
				C_OK_GREEN_OFF;
				//C_D2_Blue_Off;
				C_Buzzer_OFF;	
				C_PtsPta_Buzzer_OFF;	
				C_BAT_RED_OFF;	
				S_Count = 0;
				break;
			
	
	}

  
	S_BeforeMode = G_BuzzerLED;
	
}

#if 0

//20ms
/*
-----	---------	 ---------	               ------
	|     | 400ms   |     | 400ms  |                   |200ms
	|     |	        |	 |	        |	  		 |     
	|___|             |___|            |___________|     
	100ms             100ms                700ms          


*/
void LowBatteryBehavingUnder10V5(void)
{
	static unsigned int S_Count = 0;
	S_Count++;

	if(S_Count < 5)//100ms
	{
		C_D2_Red_On;
		C_Buzzer_ON;
	}
	else if(S_Count >5 &&  S_Count < 25)//400ms
	{
		C_D2_Red_Off;
		C_Buzzer_OFF;
	}
	else if(S_Count >25 &&  S_Count < 30)//100ms
	{
		C_D2_Red_On;
		C_Buzzer_ON;
	}
	else if(S_Count >30 &&  S_Count < 50)//400ms
	{
		C_D2_Red_Off;
		C_Buzzer_OFF;
	}
	else if(S_Count >50 &&  S_Count < 85)//700ms
	{
		C_D2_Red_On;
		C_Buzzer_ON;
	}
	else if(S_Count >85 &&  S_Count < 95)//200ms
	{
		C_D2_Red_Off;
		C_Buzzer_OFF;
	}
//////////////////////////////////////////////////
	if(S_Count >95 &&  S_Count < 100)//100ms
	{
		C_D2_Red_On;
		C_Buzzer_ON;
	}
	else if(S_Count >100 &&  S_Count < 120)//400ms
	{
		C_D2_Red_Off;
		C_Buzzer_OFF;
	}
	else if(S_Count >120 &&  S_Count < 125)//100ms
	{
		C_D2_Red_On;
		C_Buzzer_ON;
	}
	else if(S_Count >125 &&  S_Count < 145)//400ms
	{
		C_D2_Red_Off;
		C_Buzzer_OFF;
	}
	else if(S_Count >145 &&  S_Count < 180)//700ms
	{
		C_D2_Red_On;
		C_Buzzer_ON;
	}
	else if(S_Count >180 &&  S_Count < 190)//200ms
	{
		C_D2_Red_Off;
		C_Buzzer_OFF;
	}
//////////////////////////////////////////////////////////
	if(S_Count >195 &&  S_Count < 200)//100ms
	{
		C_D2_Red_On;
		C_Buzzer_ON;
	}
	else if(S_Count >200 &&  S_Count < 220)//400ms
	{
		C_D2_Red_Off;
		C_Buzzer_OFF;
	}
	else if(S_Count >220 &&  S_Count < 225)//100ms
	{
		C_D2_Red_On;
		C_Buzzer_ON;
	}
	else if(S_Count >225 &&  S_Count < 245)//400ms
	{
		C_D2_Red_Off;
		C_Buzzer_OFF;
	}
	else if(S_Count >245 &&  S_Count < 280)//700ms
	{
		C_D2_Red_On;
		C_Buzzer_ON;
	}
	else if(S_Count >280 &&  S_Count < 290)//200ms
	{
		C_D2_Red_Off;
		C_Buzzer_OFF;
	}
////////////////////////////////////////////////////////
	else if(S_Count > 290)
	{
		S_Count = 0;

		if(StartButton == Down)
		{
			G_Status_Item  = eLowBatteryUnder10V5;					
		}
		else
		{
			if(ReadFlashData(eAddrSpeed_Page0) == _FastRPM)
				G_Status_Item  = eFastMode;					
			else if(ReadFlashData(eAddrSpeed_Page0) == _SlowRPM)
				G_Status_Item  = eSlowMode;	
		}		
		C_D2_Red_Off;
		C_Buzzer_OFF;
	
	}
		
	   


}
#endif



#if ( ScrewType == _BS_PType_4Nm5 || ScrewType == _BS_PType_3Nm)	
void CheckDirection(void)
{
	if(Revers == ON)
		G_Direction = _CCW;
	else
		G_Direction = _CW;

}
#elif( ScrewType == _PTA_B120 ||ScrewType == _PTA_X50F || ScrewType == _BT_3Nm || ScrewType == _BT_2NmFast || ScrewType == _BS_1Nm2)
static u8	mu8DirBeforeStus;
extern unsigned int  HI_CW_var[3][50] ;
extern unsigned int HI_CCW_var[3][50] ;
extern unsigned int LO_CW_var[3][50] ;
extern unsigned int LO_CCW_var[3][50] ;
void CheckDirection(void)
{

	if(G_ScrewRPM == 0)
	{

		if(Brake == OFF && Revers == ON  )
		{
		//	printf("Forward\n");
			G_Direction = _CW;
			if (mu8DirBeforeStus == _Brake)
			{
				LIGHT_OFF;
		
			}
			//	default_init();//這邊一定要放,CMK才會准
		}
		else if (Brake == ON && Revers == ON)
		{
			//	printf("Brake \n");
			G_Direction = _Brake;
		//	default_init();//這邊一定要放,CMK才會准
		}
		else if (Brake == OFF && Revers == OFF)
		{
			//	printf("Reverse\n");
			G_Direction = _CCW;
			if (mu8DirBeforeStus == _Brake)
			{
				LIGHT_OFF;
			
			}
			//default_init();//這邊一定要放,CMK才會准
		}
		mu8DirBeforeStus =  G_Direction;
	}
	else
	{
		if( G_Direction == _CW && (Brake != OFF || Revers != ON  ))
		{	
			gu8BrakeStop = TRUE;
			Motor(DISABLE,G_Direction);			
			WDT_Close();
			TIMER_Stop(TIMER1); 			
			C_Buzzer_ON;
			Delayms(53);				
			C_NG_RED_ON;
			Delayms(200);		
			Delayms(261);		
			C_Buzzer_OFF;
			Delayms(109);		
			C_Buzzer_ON;
			Delayms(50);
			C_NG_RED_OFF;
			Delayms(105);		
			C_Buzzer_OFF;
			TIMER_Start(TIMER1); 			
			WDT_Init();						
		}
		else if(G_Direction == _CCW && (Brake != OFF || Revers != OFF  ))
		{			
			gu8BrakeStop = TRUE;
			Motor(DISABLE,G_Direction);			
			WDT_Close();
			TIMER_Stop(TIMER1); 			
			C_Buzzer_ON;
			Delayms(53);				
			C_NG_RED_ON;
			Delayms(200);		
			Delayms(261);		
			C_Buzzer_OFF;
			Delayms(109);		
			C_Buzzer_ON;
			Delayms(50);	
			C_NG_RED_OFF;
			Delayms(105);		
			C_Buzzer_OFF;	
			TIMER_Start(TIMER1); 			
			WDT_Init();								
		}

	}
#if 0
		G_Direction = _CW;
#endif
}




#endif






//20ms
void LED_InterlaceFlicker(void)
{ /*D1 Green ,D2 Red  */
   static unsigned int S_Count = 0;

//printf("LED_InterlaceFlicker \n");

      S_Count++;
	
      	if(S_Count >=1 &&  S_Count <= 100)
      	{
		C_OK_GREEN_ON;
		C_NG_RED_OFF;
		C_Buzzer_ON;
	}
	else if(S_Count >=100 &&  S_Count <= 200)
	{
		C_OK_GREEN_OFF;
		C_NG_RED_ON;
		C_Buzzer_OFF;
		if(S_Count >= 200)
			S_Count = 0;
	}
		

	

}

void vOscillation_P30(void)
{
	if(P30 == 0) 	P30=1;
	else			P30=0;

}



