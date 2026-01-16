
#include <stdio.h>
#include "M051Series.h"
#include "string.h"
#include "PtsPta_Global.h"

void SetPWMCMR(unsigned int PwmCMR)
{
//	unsigned int L_CNR ,L_CMR;

	
/*
	if(Duty == 100)
	{
		PWMA->CMR0 =PWMA->CNR0;
		PWMA->CMR2 =PWMA->CNR0;
		PWMB->CMR0 =PWMA->CNR0;
		
	}
	else
	{
		L_CNR = PWMA->CNR0;
		L_CMR = (L_CNR * Duty ) / 100;
		PWMA->CMR0 =L_CMR;
		PWMA->CMR2 =L_CMR;
		PWMB->CMR0 =L_CMR;

	}
*/

		PWMA->CMR0 = PwmCMR;
		PWMA->CMR2 = PwmCMR;
		PWMB->CMR0 = PwmCMR;
	

}
void Motor(unsigned char Enable_Motor,unsigned char Direction)
{
  

	UL_OFF();
	VL_OFF();
	WL_OFF();

	UH_OFF();	
	VH_OFF();	
	WH_OFF();	
	
	if(Enable_Motor == DISABLE)
	{
		SetPWMCMR(0);
				NVIC_DisableIRQ(GPIO_P0P1_IRQn);
				G_ScrewRPM = 0;
				G_MotorRPM = 0;
//	P31 = 0;
			UH_OFF();	
			VH_OFF();	
			WH_OFF();

//P13 = 0;

			CLK_SysTickLongDelay(900);
//P13 = 1;



			if((SYS->P2_MFP) != 0)
			{
				printf("&_\n");			
					UH_OFF();	
					VH_OFF();	
					WH_OFF();
			}


		
			UL_ON();
			VL_ON();
			WL_ON();




			Start_TMR_ReStart300ms();
			G_ReStar = DISABLE;
			
			//printf("&%d\n",(Get_TMR_MotorRuning_1Min()*20)/1000);
		//	printf("&\n");

		//	if((SYS->P2_MFP) == 0)			printf("  Motort ==>  UP_Mos=off,");		else			printf("  Motort ==>  UP_Mos=on,");
		//			printf(" %d %d %d\n",P21,P23,P25);

	}





}

void MotorStart(unsigned char Enable_Motor,unsigned char Direction)
{

	UL_OFF();
	VL_OFF();
	WL_OFF();

	UH_OFF();	
	VH_OFF();	
	WH_OFF();	

#if ( ScrewType == _BS_PType_4Nm5 || ScrewType == _BS_PType_3Nm )	
	 if(Enable_Motor == ENABLE && Direction != _Brake && StartButton > Down && G_ReStar == ENABLE && Get_TMR_MotorRuning_1Min() < _1Min)
#elif( ScrewType == _PTA_B120 ||ScrewType == _PTA_X50F || ScrewType == _BT_3Nm ||  ScrewType == _BT_2NmFast || ScrewType == _BS_1Nm2)
	 if(Enable_Motor == ENABLE && Direction != _Brake && StartButton == Down && G_ReStar == ENABLE && Get_TMR_MotorRuning_1Min() < _1Min)
#endif	
	{
		switch(HALL_STATUS)
			{
				case 1:	
				//	printf("1\n");	
				
						if(Direction == _CW)
						{
									WH_ON();  						
									VL_ON();																							
						}
						else								
						{
									#if ( MotorPhase == ForWard_Phas )	
										VH_ON();	
										UL_ON();
									#elif( MotorPhase == Center_Phas )							
										VH_ON();		
										WL_ON();  //0					
									#elif( MotorPhase == BackWard_Phas )								
										UH_ON(); //-1
										WL_ON();
									#endif														
						}										
					break;
				case 3:			
				//	printf("3\n");

						 if(Direction == _CW)
						{
										UH_ON();						
										VL_ON();
								
						}
						else
						{	
									#if ( MotorPhase == ForWard_Phas )									
										WH_ON();	
										UL_ON();
									#elif( MotorPhase == Center_Phas )
										VH_ON();	
										UL_ON();	
									#elif( MotorPhase == BackWard_Phas )
										VH_ON();
										WL_ON();//  -1
									#endif										
						}	
					break;					
				case 2:		
				//	printf("2\n");


	  				 	 if(Direction == _CW)
						{
									UH_ON();
							 		WL_ON();																	
						}
						else
						{

									#if ( MotorPhase == ForWard_Phas )	
										WH_ON();	
										VL_ON();	
									#elif( MotorPhase == Center_Phas )
										WH_ON();	
										UL_ON();
									#elif( MotorPhase == BackWard_Phas )													
										VH_ON(); //-1
										UL_ON();													
									#endif	
						}				
					break;					
				case 6:
				//	printf("6\n");	

				
					 	if(Direction == _CW)
						{
										VH_ON();
										WL_ON();						
						}
						else
						{
									#if ( MotorPhase == ForWard_Phas )	
										UH_ON();	
										VL_ON(); 	
									#elif( MotorPhase == Center_Phas )	
										WH_ON();	
										VL_ON();																
									#elif( MotorPhase == BackWard_Phas )
										WH_ON(); //-1
										UL_ON();
									#endif		
		
						}			
					break;								
				case 4:	
				//	printf("4\n");
				
						if(Direction == _CW)
						{										
									VH_ON();
									UL_ON();																				

						}
						else
						{
								#if ( MotorPhase == ForWard_Phas )	
									UH_ON(); 	
									WL_ON();
								#elif( MotorPhase == Center_Phas )
									UH_ON();	
									VL_ON(); // 0								
								#elif( MotorPhase == BackWard_Phas )							
									WH_ON();// -1
									VL_ON();
								#endif				
						}
					break;					
				case 5:	
				//	printf("5\n");
				
						if(Direction == _CW)
						{
								WH_ON();	
								UL_ON();
						}
						else
						{
									#if ( MotorPhase == ForWard_Phas )	
										VH_ON();	
										WL_ON(); 
			
									#elif( MotorPhase == Center_Phas )
										UH_ON();	
										WL_ON();								
									#elif( MotorPhase == BackWard_Phas )
										UH_ON();
										VL_ON();  

									#endif													
						}
					break;
		
			}

	}







}


