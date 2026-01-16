
#include <stdio.h>
#include "M051Series.h"
#include "string.h"
#include "PtsPta_Global.h"
#include "PtsPta_ADC.h"
#include "PtsPta_InputOutput.h"
#include "PtsPta_TypeDef.h"
#include "PtsPta_NVRAM.h"
#include "PtsPta_Motor.h"
#include "PtsPta_InputOutput.h"
#include "PtsPta_FMC.h"
#include "PtsPta_Protection.h"
#include "PtsPta_Init.h"






void PowerDownFunction(void)
{
    /* To check if all the debug messages are finished */
   // UART_WAIT_TX_EMPTY(UART0);

    SCB->SCR = 4;


    CLK->PWRCON = (CLK->PWRCON & ~(CLK_PWRCON_PWR_DOWN_EN_Msk | CLK_PWRCON_PD_WAIT_CPU_Msk)) |
                  CLK_PWRCON_PD_WAIT_CPU_Msk | CLK_PWRCON_PD_WU_INT_EN_Msk;
    CLK->PWRCON |= CLK_PWRCON_PWR_DOWN_EN_Msk;



    __WFI();
}

void LED_Flicker_B(unsigned char Value)
{
	static unsigned int Count = 0;
	static unsigned int	Times = 0;
	static unsigned char 	TurnOffLed = ON;

	//printf("Flicker B =%d   Times =%d\n",Count , Times);

	
	if(Value == ON)
	{
			Count++;

			if(Count < 200 )
			{
					if(TurnOffLed == ON)
					{
						C_D2_Blue_On;
					}
				
					if(Count == 50)
						Times++;
				
					if(Times == 5)
					{
						Times = 10;
						TurnOffLed = OFF;

						C_Buzzer_ON;
						TIMER_Stop(TIMER1); 
						Delayms(300);
						TIMER_Start(TIMER1); 			
						Delayms(50);
						C_Buzzer_OFF;


						
						
						printf("Change Speed Mode\n");
						if(ReadFlashData(eAddrSpeed_Page0) == _FastRPM)
						{
							printf("* Slow Mode *\n");	
							SaveData2Flash(eAddrSpeed_Page0,_SlowRPM);
							G_Status_Item = eSlowMode;
							C_D2_Blue_On;	
							
						}
						else
						{
							printf("* Fast Mode *\n");							
							SaveData2Flash(eAddrSpeed_Page0,_FastRPM);
							G_Status_Item = eFastMode;
						}



					
					}
			}	
			else if(Count > 200 &&  Count < 400 )
			{
				//LED_Green_OFF();
				//P06=0;
				C_D2_Blue_Off;
			}
			
			if(Count > 400)
			{
				Count =0;
			}
	}
	else
	{
		Count = 0;
		Times = 0;
		TurnOffLed = ON;
	}	
}





/**
 * @brief       Port0/Port1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Port0/Port1 default IRQ, declared in startup_M051Series.s.
 */
unsigned char errorflag = 0;
unsigned int G_StallCnt;
extern unsigned int G_Voltage_Battery;
extern uint32_t varcount;
uint8_t stall_flag = 0;
extern volatile uint32_t t2;
void GPIOP0P1_IRQHandler(void)
{
	static unsigned char S_HallSensorCount  = 0  ;
	static  unsigned char  BefHallSensor;
//	printf("%d \n", HALL_STATUS);
#if 1

	/* To check if P1.3 interrupt occurred */
	if(GPIO_GET_INT_FLAG(P0, BIT0))
	{
	    GPIO_CLR_INT_FLAG(P0, BIT0);
	}
	else if(GPIO_GET_INT_FLAG(P0, BIT1))		
	{
	    GPIO_CLR_INT_FLAG(P0, BIT1);
	}
	else if(GPIO_GET_INT_FLAG(P0, BIT2))		
	{
	    GPIO_CLR_INT_FLAG(P0, BIT2);
	}
#endif
#if(Pin7 == _GPIOFunc)
	P30= 0;
	CLK_SysTickDelay(1);
	P30= 1;
#endif


#if 0
			//每次中斷都會清
			Stop_TMR_MotorStall();
			Reset_TMR_MotorStall();
	
#else

#if ( ScrewType == _BS_PType_4Nm5 || ScrewType == _BS_PType_3Nm  )	
/*	CW		1 -> 5 -> 4 -> 6 -> 2 -> 3 -> 1
	CCW	1 -> 3 -> 2 -> 6 -> 4 -> 5 -> 1  */
		  
	if(((((BefHallSensor == 3 && HALL_STATUS == 1) || 
		(BefHallSensor == 2 && HALL_STATUS == 3) || 
		(BefHallSensor == 6 && HALL_STATUS == 2) || 
		(BefHallSensor == 4 && HALL_STATUS == 6) || 
		(BefHallSensor == 5 && HALL_STATUS == 4) || 
		(BefHallSensor == 1 && HALL_STATUS == 5) ) && G_Direction == _CW )||
		(((BefHallSensor == 1&& HALL_STATUS == 3) || 
		(BefHallSensor == 3 && HALL_STATUS == 2) || 
		(BefHallSensor == 2 && HALL_STATUS == 6) || 
		(BefHallSensor == 6 && HALL_STATUS == 4) || 
		(BefHallSensor == 4 && HALL_STATUS == 5) || 
		(BefHallSensor == 5 && HALL_STATUS == 1) ) && G_Direction == _CCW )) &&   G_MotorStatus == eMotorStatus_Normal)		
//	if(((((BefHallSensor == 1&& HALL_STATUS == 3) || (BefHallSensor == 3 && HALL_STATUS == 2) || (BefHallSensor == 2 && HALL_STATUS == 6) || (BefHallSensor == 6 && HALL_STATUS == 4) || (BefHallSensor == 4 && HALL_STATUS == 5) || (BefHallSensor == 5 && HALL_STATUS == 1) ) && G_Direction == _CCW )|| G_Direction == _CW) &&   G_MotorStatus == eMotorStatus_Normal)		

#else //BT Type
/*	CW		1 -> 3 -> 2 -> 6 -> 4 -> 5 -> 1	
	CCW	1 -> 5 -> 4 -> 6 -> 2 -> 3 -> 1 */
	if(((((BefHallSensor == 3 && HALL_STATUS == 1) || 
		(BefHallSensor == 2 && HALL_STATUS == 3) || 
		(BefHallSensor == 6 && HALL_STATUS == 2) || 
		(BefHallSensor == 4 && HALL_STATUS == 6) || 
		(BefHallSensor == 5 && HALL_STATUS == 4) || 
		(BefHallSensor == 1 && HALL_STATUS == 5) ) && G_Direction == _CCW )||
		(((BefHallSensor == 1&& HALL_STATUS == 3) || 
		(BefHallSensor == 3 && HALL_STATUS == 2) || 
		(BefHallSensor == 2 && HALL_STATUS == 6) || 
		(BefHallSensor == 6 && HALL_STATUS == 4) || 
		(BefHallSensor == 4 && HALL_STATUS == 5) || 
		(BefHallSensor == 5 && HALL_STATUS == 1) ) && G_Direction == _CW )) &&   G_MotorStatus == eMotorStatus_Normal)		

#endif

	
		{
	
		//	printf(" R\n");
			Stop_TMR_MotorStall();
			Reset_TMR_MotorStall();

			S_HallSensorCount ++;
			Start_TMR_CalcRPM();




#if ( ScrewType == _BS_PType_4Nm5 || ScrewType == _BS_PType_3Nm)	
			if(StartButton > Down && (Get_TMR_MotorRuning_1Min()*20 > 100)  && (Get_TMR_MotorRuning_1Min() < (_1Min - 1000)))			
#elif( ScrewType == _PTA_B120 ||ScrewType == _PTA_X50F  || ScrewType == _BT_3Nm ||  ScrewType == _BT_2NmFast || ScrewType == _BS_1Nm2)
			if(StartButton == Down && (Get_TMR_MotorRuning_1Min()*20 > 100) && (Get_TMR_MotorRuning_1Min() < (_1Min - 1000)))			
#endif
			{
				Start_TMR_MotorStall();
			}



			
	
		}
		else
		{


#if ( ScrewType == _BS_PType_4Nm5 || ScrewType == _BS_PType_3Nm)				
			if(G_MotorStatus == eMotorStatus_Normal && StartButton > Down  && (Get_TMR_MotorRuning_1Min() < (_1Min - 1000)) &&( Get_TMR_MotorRuning_1Min() * 20  > 3000)  )
#else
	//		if(G_MotorStatus == eMotorStatus_Normal && StartButton == Down  && (Get_TMR_MotorRuning_1Min() < (_1Min - 1000)) && ((Get_TMR_MotorRuning_1Min() * 20  > 30000 && G_Direction == _CW) ||(Get_TMR_MotorRuning_1Min() * 20  > 50000 && G_Direction == _CCW)) )
			if(G_MotorStatus == eMotorStatus_Normal && StartButton == Down  && (Get_TMR_MotorRuning_1Min() < (_1Min - 1000)) &&( Get_TMR_MotorRuning_1Min() * 20  > 3000)  )
#endif			
			G_StallCnt ++;
			//printf("   %d\n",G_StallCnt);
			
	
#if ( ScrewType == _BS_PType_4Nm5 || ScrewType == _BS_PType_3Nm)				
			if( G_StallCnt > 5000 && G_MotorStatus == eMotorStatus_Normal && StartButton > Down )
#else
			if( G_StallCnt > 25 && G_MotorStatus == eMotorStatus_Normal && StartButton == Down)
#endif
			{
				if(Get_TMR_MotorRuning_1Min()*20 > 70000)
				{
						printf("Force x%d\n",(Get_TMR_MotorRuning_1Min()*20)/1000);
				
						G_MotorStatus = eMotorStatus_Stop;
						Motor(DISABLE, G_Direction);
						G_BuzzerLED = eBehaving_MotorStall;  
				}
				else
				{
				
					G_MotorStatus = eMotorStatus_Stall;
					Start_TMR_MotorStall();
					printf("x%d\n",(Get_TMR_MotorRuning_1Min()*20)/1000);
				}	
				
				

			}

		}

	#endif	
	


	

	



		




#if(POLES2_SLOTS6)
	if(S_HallSensorCount >5 )			
#elif (POLES4_SLOTS6)
	if(S_HallSensorCount >11) 	
#endif
	{
		//G_ScrewRPM = Get_TMR_CalcRPM();

//printf("%d,%d\n",G_PwmDuty,G_ScrewRPM);

#if 1
		G_MotorRPM= (60000000/(Get_TMR_CalcRPM() * 20));		




#if ( ScrewType == _BS_PType_4Nm5)	
/*  尺比為 24.42
		       1 / 24.42 = 0.0409 =  (X *409)/ 10000		*/		

		G_ScrewRPM = (G_MotorRPM * 409) /10000 ;		

#elif( ScrewType == _BS_PType_3Nm)	
/*  尺比為 3.75 * 3.75 = 14.0625  				
		       1 / 14.06 = 0.0711 =  (X *711)/ 10000		*/		

		G_ScrewRPM = (G_MotorRPM * 711) /10000 ;		

		
#elif( ScrewType == _PTA_B120)

/*  尺比為 17.25
		       1 / 17.25 = 0.0579 =  (X *579)/ 10000		*/	

	if(StartButton == Up)
	{
		G_ScrewRPM = 0;
		G_MotorRPM = 0;
		
	}
	else if(G_ReStar == 1 && StartButton == Down)
	{
		G_ScrewRPM = (G_MotorRPM * 580) /10000 ;
	}
	
#elif(ScrewType == _PTA_X50F )

/*  尺比為 6.8571428
		       1 / 6.85714 = 0.1458 =  (X *1458)/ 10000		*/	

	if(StartButton == Up)
	{
		G_ScrewRPM = 0;
		G_MotorRPM = 0;
		
	}
	else if(G_ReStar == 1 && StartButton == Down)
	{
		G_ScrewRPM = (G_MotorRPM * 1458) /10000 ;
	}
	


	


#elif( ScrewType == _BT_3Nm || ScrewType == _BT_2Nm ) 
/*  尺比為 3.75 * 3.75 = 14.0625  				
		       1 / 14.06 = 0.0711 =  (X *711)/ 10000		*/		

		G_ScrewRPM = (G_MotorRPM * 711) /10000 ;		


#elif( ScrewType == _BT_2NmFast  ) 

	G_ScrewRPM = (G_MotorRPM ) /7 ;		

#elif(ScrewType == _BS_1Nm2)
/*  尺比為 16.00
		       1 / 16.00 = 0.0625 =  (X *625)/ 10000		*/		

	if(StartButton == Up)
	{
		G_ScrewRPM = 0;
		G_MotorRPM = 0;
		
	}
	else if(G_ReStar == 1 && StartButton == Down)
	{
		G_ScrewRPM = (G_MotorRPM * 625) /10000 ;

	}	

#endif

	
#endif		
		Stop_TMR_ICalcRPM();
		Reset_TMR_CalcRPM();
		Start_TMR_CalcRPM();

		S_HallSensorCount =0;	
	}

	
		

	if( (G_ClutchBrakFlow == eClutchBrake_None ||  G_ClutchBrakFlow == eClutchBrake_Hi ) && G_MotorStatus == eMotorStatus_Normal && G_ReStar == ENABLE)
	{
	
	//	 if(G_ReStar == ENABLE)
		 {// printf("a");
			MotorStart(ENABLE, G_Direction);
		 }
	}



	BefHallSensor = HALL_STATUS;
	
}
extern unsigned char T_1ms;
extern unsigned char T_1ms222;

unsigned int L_TargetCount, StartCMR, S_MaxADC, S_MinADC;

//extern volatile uint32_t t;
//volatile uint32_t t2 = 0;
volatile uint32_t wait_5S = 0;
volatile uint8_t wait_5S_flag = 0;
// #define START_THRESHOLD 2000
#define START_HYST 10
/*    20 us      ���Y�w�g�ϥΤF 3.6us ���`�N */

#define DEBOUNCE_MS 50

static uint8_t btnStable = Up;
static uint8_t btnLast = Up;
static uint32_t debounceCnt = 0;
//uint32_t t3 = 0, t4 = 0;
extern volatile uint8_t g_FoolProof;
extern uint8_t copy_StartButton;
uint32_t gpio_debunce_10ms = 0;
extern uint8_t g_u8DebounceFlag;
void TMR0_IRQHandler(void)
{
	static unsigned int S_Count, S_1ms, S_StartBtnDnCnt, S_StartBtnUpCnt;
	static unsigned char U_Phase, V_Phase, W_Phase;
	uint8_t btnNow;

	btnNow = (G_StartInAdc > (START_THRESHOLD + START_HYST)) ? Down, copy_StartButton = Down : Up, copy_StartButton = Up;

	/*	static unsigned int S_BeforeCurrent,S_BeforeStartInAdc   S_MaxADC =261 , S_MinADC=260*/;
	//	unsigned int L_TargetRPM , L_MinTime, L_MaxTime, L_tmp;
	/* Clear Timer0 time-out interrupt flag */
	TIMER_ClearIntFlag(TIMER0);
#if (Pin7 == _GPIOFunc)
	if (P31 == 1)
		P31 = 0;
	else
		P31 = 1;
#endif

	S_1ms++;

	// wait_5S++;
	/*t++;

	if (t >= 500)
	{
		t = 0;
		t2++;
	}
	*/
	if (S_1ms >= 50) // 20 * 50 = 1000us = 1ms
	{
		//		vOscillation_P30();

	//	T_1ms = 1;
		T_1ms222++;
		S_1ms = 0;

		if (btnNow != btnLast)
		{
			debounceCnt = 0; // 狀態變了，開始計數
			btnLast = btnNow;
		}
		else
		{
			if (debounceCnt < DEBOUNCE_MS)
				debounceCnt++;
			if (debounceCnt >= DEBOUNCE_MS)
			{
				btnStable = btnNow; // 確認後才更新
				StartButton = btnStable;
			}
		}

		//	wait_5S++;
		//	if(wait_5S>5000)
		//	{
		//		wait_5S_flag=1;
		//	}
		// if(wait_5S_flag==1)
		//	{
		/*if (G_StartInAdc > START_THRESHOLD)
	{
		S_StartBtnDnCnt++;
		S_StartBtnUpCnt = 0;

		if (S_StartBtnDnCnt >= 40) // 20ms
		{
			S_StartBtnDnCnt = 21;
			S_StartBtnUpCnt = 0;
			StartButton = Down; // printf("		Down\n");
		}
	}
	else
	{

		S_StartBtnUpCnt++;
		S_StartBtnDnCnt = 0;
		if (S_StartBtnUpCnt >= 40) // 20ms
		{
			S_StartBtnUpCnt = 14;
			S_StartBtnDnCnt = 0;
			StartButton = Up; // printf("		Up\n");
		}
	}
	*/
		/*if (G_StartInAdc > (START_THRESHOLD + START_HYST)) {
	if (S_StartBtnDnCnt < 40) S_StartBtnDnCnt++;
	if (S_StartBtnUpCnt > 0)  S_StartBtnUpCnt--; // 遞減，而不是直接清零

	if (S_StartBtnDnCnt >= 40) {
		StartButton = Down;
	}
}
else if (G_StartInAdc < (START_THRESHOLD - START_HYST)) {
	if (S_StartBtnUpCnt < 40) S_StartBtnUpCnt++;
	if (S_StartBtnDnCnt > 0)  S_StartBtnDnCnt--; // 遞減，而不是直接清零

	if (S_StartBtnUpCnt >= 40) {
		StartButton = Up;
	}
		*/
		//}
		//}
	}
	S_Count++;


	if(TMR_Double_Enalbe.u8_OnOFF == ON)
		TMR_Double_Enalbe.u32_Count++;
	else
		TMR_Double_Enalbe.u32_Count= 0;



	if(TMR_ID_OK.u8_OnOFF == ON)
		TMR_ID_OK.u32_Count++;
	else
		TMR_ID_OK.u32_Count= 0;


	if(TMR_CalcRPM.u8_OnOFF == ON)
		TMR_CalcRPM.u32_Count++;
	else
		TMR_CalcRPM.u32_Count= 0;

	if(TMR_PWROff_24S.u8_OnOFF == ON)
		TMR_PWROff_24S.u32_Count++;
	else
		TMR_PWROff_24S.u32_Count= 0;

	if(TMR_Stop_10S.u8_OnOFF == ON)
		TMR_Stop_10S.u32_Count++;
	else
		TMR_Stop_10S.u32_Count= 0;

	if(TMR_PWROff_3S.u8_OnOFF == ON)
		TMR_PWROff_3S.u32_Count++;
	else
		TMR_PWROff_3S.u32_Count= 0;

	if(TMR_MotorRuning_1Min.u8_OnOFF == ON)
		TMR_MotorRuning_1Min.u32_Count++;
	else
		TMR_MotorRuning_1Min.u32_Count= 0;


	if(TMR_MotorTempOverHeat_3S.u8_OnOFF == ON)
		TMR_MotorTempOverHeat_3S.u32_Count++;
	else
		TMR_MotorTempOverHeat_3S.u32_Count= 0;

	if(TMR_PCBTempOverHeat_3S.u8_OnOFF == ON)
		TMR_PCBTempOverHeat_3S.u32_Count++;
	else
		TMR_PCBTempOverHeat_3S.u32_Count= 0;

	if(TMR_MotorStall.u8_OnOFF == ON)
	{
		TMR_MotorStall.u32_Count++;
//		if(TMR_MotorStall.u32_Count == 20000)//10us    /*200ms 賭轉*/
#if ( ScrewType == _BS_PType_4Nm5 || ScrewType == _BS_PType_3Nm)
		if((Get_TMR_MotorStall()*20) >= 100000 && G_ClutchBrakFlow == eClutchBrake_None && StartButton > Down && G_MotorStatus == eMotorStatus_Normal&& G_PCBStatus == ePCBStatus_Normal)//20us    /*200ms 賭轉*/
#else
		if((Get_TMR_MotorStall()*20) >= 600000 && G_ClutchBrakFlow == eClutchBrake_None && StartButton == Down && G_MotorStatus != eMotorStatus_Stop && G_ReStar == ENABLE)//20us    /*200ms 賭轉*/

#endif
//		if(TMR_MotorStall.u32_Count == 7500 && G_ClutchBrakFlow == eClutchBrake_None)//20us    /*150ms 賭轉*/
		//if(TMR_MotorStall.u32_Count == 5000 && G_ClutchBrakFlow == eClutchBrake_None)//20us    /*100ms 賭轉*/
		{
		//	G_MotorStatus = eMotorStatus_Stop;
		//	NVIC_DisableIRQ(GPIO_P0P1_IRQn);
/*
			printf("\n\n");
			printf("                                                 ****************************************\n");
			printf("                                                 ****************************************\n");
			printf("                                                 **                                                               **\n");
			
			printf("                                                 **                                                               **\n");
			printf("                                                 ****************************************\n");
			printf("                                                 ****************************************\n");
			printf("\n\n");
*/

			//	P31 = 1;
		G_MotorStatus = eMotorStatus_Stop;
		Motor(DISABLE, G_Direction);
		G_BuzzerLED = eBehaving_MotorStall;  
		printf("Stl\n" );			


		}
		
	}	
	else
		TMR_MotorStall.u32_Count= 0;


	if(TMR_ReStart300ms.u8_OnOFF == ON)
		TMR_ReStart300ms.u32_Count++;
	else
		TMR_ReStart300ms.u32_Count= 0;

	
/* 到這裡約使用了1.73 us */	


//S_BeforeStartInAdc = G_StartInAdc;
//S_BeforeCurrent = G_Current;
//S_BeforeStall = G_StallCnt;


#if 0
		G_PwmCMR = PWMMaxCMR; //MOS PWM  100% 全開
#endif
//G_PwmCMR=	 990;//((PWMMaxCMR * 10)/100);

		if(G_PwmCMR > gu32MAXPWM ){	G_PwmCMR  = gu32MAXPWM ;	}
		
		SetPWMCMR(G_PwmCMR);


//StartButton = 4096;
/* 到這裡約使用了2.3 us */	
/////////////////////////////////////////////////////
/* 如果同 相位 立即中斷馬達 */
#if 1
	if(P23 == 1 && ((SYS->P2_MFP & SYS_MFP_P22_PWM2 ) == SYS_MFP_P22_PWM2) && G_Direction != _Brake && StartButton == Down && G_ReStar == ENABLE)
	{
		W_Phase ++;

		if(W_Phase >= 2)
		{
			Motor(DISABLE, G_Direction);	
			printf("~%d\n",G_Direction);
		}
	}
	else
		W_Phase =0; 
	
	if(P21 == 1 && ((SYS->P2_MFP & SYS_MFP_P20_PWM0 ) == SYS_MFP_P20_PWM0) && G_Direction != _Brake && StartButton == Down && G_ReStar == ENABLE)
	{
		V_Phase++;
		if(V_Phase >= 2)
		{
			Motor(DISABLE, G_Direction);		
			printf("!%d\n",G_Direction);
		}
	}
	else
		V_Phase = 0;
	if(P25 == 1 && ((SYS->P2_MFP & SYS_MFP_P24_PWM4 ) == SYS_MFP_P24_PWM4) && G_Direction != _Brake && StartButton == Down && G_ReStar == ENABLE)
	{
		U_Phase ++;
		if(U_Phase >= 2)
		{
			Motor(DISABLE, G_Direction);		
			printf("@%d\n",G_Direction);
		}	
	}
	else
		U_Phase = 0;
#endif	

/* 到這裡約使用了3.6 us */	


}

static u8 mu8DoubleCount;
unsigned char S_10ms = 0;
extern unsigned int Temp_var[3][200];
/* 5ms  */
void TMR1_IRQHandler(void)
{
	int i;
	static unsigned char		S_1s = 0 ;//,S_5Vflg= 0; //, S_StallCount=0;	
	static unsigned int		S_Debounce10V5 = 0 ;
	static unsigned char		S_Debounce10V5_11V = False;
//	static unsigned char 		S_Flg_ButtonDown = 0;
//	static unsigned char 		S_Flg_ButtonUp = 0;
	static unsigned char 		S_Flg_LowBattery = 0;
	static unsigned char 		S_Flg_DirIsBrakeOf10V5_11V = 0;
//	static unsigned char 		S_BeforeDirection = 0;	
	static unsigned char		S_Flag;
#if( ScrewType == _PTA_B120 ||ScrewType == _PTA_X50F  || ScrewType == _BT_3Nm || ScrewType == _BT_2NmFast || ScrewType == _BS_1Nm2)	
	static unsigned int 		S_MiddleBrakeCnt = 0; 
#endif
	TIMER_ClearIntFlag(TIMER1);  	

#if(WatchDog == ENABLE)
//	if((Get_TMR_PWROff_24S()/100000)  == 5) /* 驗證10 秒後 系統 reset  */
 // 	if(G_Direction != _Brake) /*切中段 來驗證 WatchDog */
		WDT_RESET_COUNTER(); /* 餵狗 程式 */
#endif



	/******************************************************/
	/********   Printf Message  *******************************/
	/******************************************************/
	S_1s++;
	S_10ms++;
	GetADCValue();

	if(G_ID_Status == _NG	)	
		Buzzer_LED_Behaving(&G_BuzzerLED);

	if(G_ID_Status == _NG	)	
	{

#ifdef Scott_Debug
 		if(1)
#else
		if(1)//id idef remove jacob 20250516
		//if(CheckBatteryType(eIICType_MCU) == eEEPROM_Type_MCU )
#endif			
		{
			G_ID_Status = _OK;	
			Reset_TMR_PWROff_3S();		Stop_TMR_PWROff_3S();
			Reset_TMR_PWROff_24S();	Stop_TMR_PWROff_24S();
			

			if(G_Voltage_Battery < LowVoltage_10V5)
			{
#ifdef jacob == 1
				printf("		##  %d > V ##\n\n", LowVoltage_10V5);
#endif

				C_BAT_RED_OFF;
				C_PtsPta_Buzzer_OFF;

			//	G_BuzzerLED = eBehaving_Idle_1S;  
				Buzzer_LED_Behaving(&G_BuzzerLED);				



				
	
			}
			else if(LowVoltage_10V5 <= G_Voltage_Battery && G_Voltage_Battery < LowVoltage_11V)
			{	
				printf("		## %d< V <%d\n\n", LowVoltage_10V5, LowVoltage_11V);


			//	G_BuzzerLED = eBehaving_Idle_1S;  		  
				Buzzer_LED_Behaving(&G_BuzzerLED);		

			}
			else
			{
				printf("		## %d < V\n\n",LowVoltage_11V);
				Start_TMR_PWROff_3S();
				Start_TMR_PWROff_24S();
//				G_BuzzerLED = eBehaving_OFF;  
		//		G_BuzzerLED = eBehaving_Idle_1S;  
				Buzzer_LED_Behaving(&G_BuzzerLED);

			}	
		}					
	}		


#if 0
{
	static unsigned int	i;
	
//	if(S_1s == 4 )//  5ms * 4 = 20ms


#if ( ScrewType == _BS_PType_4Nm5)	
//		if(StartButton > Down  && S_StartDownCnt >= 4 )			
#elif( ScrewType == _PTA_B120 ||ScrewType == _PTA_X50F || ScrewType == _BT_3Nm || ScrewType == _BT_2NmFast)
		if(StartButton == Up)
			i = 0;








		if(StartButton == Down  )			
#endif
	{
		i++;
		S_1s = 0;

	//printf("%3d %3d|%4d=%3d%%| ",G_StartInAdc ,G_ScrewRPM  ,G_PwmCMR  ,(PWMA->CMR0*100)/ PWMA->CNR0);
	//printf("%3d %3d|%2d%%| ",G_StartInAdc ,G_ScrewRPM  ,(PWMA->CMR0*100)/ PWMA->CNR0);
//	printf("%3d, %3d:%2d%%\n",G_ScrewRPM ,G_PwmCMR ,(PWMA->CMR0*100)/ PWMA->CNR0);


/*

	printf("%2d/%2d, %3d, %4d, %3d, %4d, %d\n"     		,S_StartBtnDnCnt
											,S_StartBtnUpCnt		
											,G_ScrewRPM 
											,G_PwmCMR  											
												
											,G_Current											
											,(Get_TMR_MotorRuning_1Min()*20)/1000 
											,G_StartInAdc);


*/

#if 1
	printf(" %4d, %4d, %3d, %3d, %4d, %d\n"  	,			G_ScrewRPM 
												,G_PwmCMR  																							
												,G_Current	
												
												,L_TargetCount
											
												,(Get_TMR_MotorRuning_1Min()*20)/1000 
												,(Get_TMR_MotorStall()*20)/1000 

											
																							
											//	,HALL_STATUS
											//	,G_StallCnt
											
											);


#endif

	}
}
#else	
	if(S_1s == 200  )//  5ms *200 = 1000ms =  1s 
	{
		S_1s = 0;

		if(G_ID_Status == _NG)
		{			
			printf("ID_NG\n");
			G_BuzzerLED = eBehaving_ConnectButIDNotPass;

		}
		else
		{



#if 1
	//	printf("%d, %d, %d=%d%%, I=%d :%d, V=%d, %d, %d/%d/%d %d/%d/%d , %d/%d, %d/%d, \n\n",G_Direction , G_ScrewRPM , G_PwmCMR , (PWMA->CMR0*100)/ PWMA->CNR0 , G_Current , ADC2Voltage(G_Current), G_Voltage_Battery , StartButton ,G_NTC_PCB , G_NTC_Motor , G_NTC_Battery,G_PCBStatus,G_MotorStatus,G_BatteryStatus,((Get_TMR_PWROff_3S()*20)/1000000),((Get_TMR_PWROff_24S()*20)/1000000),G_StallCnt , G_ReStar);
	//	printf("%d, %d, %d=%d%%, I=%d, V=%d, %d, %d/%d/%d %d/%d/%d , %d/%d, %d,%d, \n\n",G_Direction , G_ScrewRPM , G_PwmCMR , (PWMA->CMR0*100)/ PWMA->CNR0 , G_Current ,/* ADC2Voltage(G_Current),*/ G_Voltage_Battery , StartButton ,G_NTC_PCB , G_NTC_Motor , G_NTC_Battery,G_PCBStatus,G_MotorStatus,G_BatteryStatus,((Get_TMR_PWROff_3S()*20)/1000000),((Get_TMR_PWROff_24S()*20)/1000000),G_StallCnt , G_ReStar);
	/*	printf("%d, %d, %d, [%d/%d, %d, ], {%d/%d, %d}, %d, %d, [%d, %d"			,G_Direction 
																		,G_ScrewRPM 
																		,G_PwmCMR  
																					
																			,G_Current, gu32MaxCurrent
																			,G_Voltage_Battery 
																			 
																						
														,G_NTC_PCB  ,G_NTC_Motor 			,G_PCBStatus	
						
																	,((Get_TMR_PWROff_24S()*20)/1000000)
																	,G_StallCnt 


																	,G_Flg_ButtonUp
																,	G_StartInAdc
																			);

  



	//	printf("   StrInAdc=%d, %d",G_StartInAdc , tmp_RPM[G_StartInAdc - 260 ]);

		printf("\n\n");
*/

#else


			//	printf("       Stus=%d, Duty=%d, V=%d, I=%d, RPM=%d, Clutch=%d",G_Status_Item,G_PwmDuty,G_Voltage_Battery,G_Current ,G_ScrewRPM ,ClutchBrake);
		//	printf(" FirstBtnDwn=%d Dbuns10v5=%d, 5s=%d, 24s=%d, 1M=%d 3S=%d\n", G_IsFirstBtnDwn,S_Debounce10V5 ,((Get_TMR_PWROff_3S()*5)/1000000),((Get_TMR_PWROff_24S()*5)/1000000),((Get_TMR_MotorRuning_1Min()*5)/1000000),((Get_TMR_MotorTempOverHeat_3S()*5)/1000000),G_StallCnt , G_ReStar);
		//	printf("                                                                                                                           NTC Pcb=%d, Motor=%d, Start=%d\n",G_NTC_PCB , G_NTC_Motor,StartButton);
		//	printf("P33=%d P36=%d P37=%d Hall=%d, Dir=%d",P33,P36 , P37,HALL_STATUS,G_Direction);

			printf("%d,",G_Direction);
	//		printf("BuzLed=%d, ",G_BuzzerLED);

	//		printf("Stus=%d, ",G_Status_Item);
			

		   
		//	printf("Hall=%d, ",HALL_STATUS);		
		//		printf("D=%d,%d,  CNR=%d/%d =%d%%",G_PwmCMR, G_ScrewRPM , PWMA->CNR0 ,  PWMA->CMR0 ,(PWMA->CMR0*100)/ PWMA->CNR0);
	//		printf("%d:%d, CMR=%d=%d%%,", G_MotorRPM, G_ScrewRPM  ,G_PwmCMR  ,(PWMA->CMR0*100)/ PWMA->CNR0);
			printf("%d, %d=%d%%,",  G_ScrewRPM  ,G_PwmCMR  ,(PWMA->CMR0*100)/ PWMA->CNR0);
		//	printf("Duty=%d, ",G_PwmDuty);
			printf("I=%d :%d, ",G_Current,  ADC2Voltage(G_Current));
		//	printf("I=%d, ",G_Current);
			printf("V=%d, ",G_Voltage_Battery);
		//	printf("RPM=%d, ",G_ScrewRPM);



	//		printf("Clutch=%d, ",ClutchBrake);
	//		printf("Dir=%d, ",G_Direction); 	//0:CW	1:Brake 	2:CCW
		//	printf("FistBtn=%d, ",G_IsFirstBtnDwn);
		//	printf("Dbuns10v5=%d, ",S_Debounce10V5);

	//		printf("1Min=%d, ",((Get_TMR_MotorRuning_1Min()*10)/1000000));
	//		printf("3s=%d, ",((Get_TMR_MotorTempOverHeat_3S()*10)/1000000));


	//	  printf("TMR_Stall=%d, ",((Get_TMR_MotorStall()*10)/1000 ));


	//	  printf("G_ScrewLock=%d, ",G_ScrewLock);

		  printf("%d, ",StartButton);

	//	  printf("Cluch=%d, BC=%d, ",G_ClutchBrakFlow , G_BrakeCurrent);
		  

			
		printf("%d/%d/%d ",G_NTC_PCB ,G_NTC_Motor  ,G_NTC_Battery);
		printf("%d/%d/%d ",G_PCBStatus,G_MotorStatus,G_BatteryStatus);
		

	//	printf(" Count:%d ID=%d ,%d",G_Count, G_ID_Status , Get_TMR_MotorStall()*20);
	//	printf("      5s=%d 24s=%d",Get_TMR_PWROff_3S()/100000,Get_TMR_PWROff_24S()/100000);
	//	printf("   ReStar=%d, ",((Get_TMR_ReStart300ms()*10)/1000 ));
	//	printf("%X",((SYS->P2_MFP) & SYS_MFP_P20_PWM0)); 		printf(" %X",((SYS->P2_MFP) & SYS_MFP_P22_PWM2));	printf(" %X",((SYS->P2_MFP) & SYS_MFP_P24_PWM4));

		//	if((SYS->P2_MFP) == 0)			printf("       UP_Mos=off,");		else			printf("UP_Mos=on,");
		//	printf(" %d %d %d",P21,P23,P25);
			


	//		printf(" Clutch:%d",	G_ClutchBrakFlow);




		//	printf("      5s=%d, ",((Get_TMR_PWROff_3S()*20)/1000000));
		//	printf("24s=%d, ",((Get_TMR_PWROff_24S()*20)/1000000));
			printf("%d/%d, ",((Get_TMR_PWROff_3S()*20)/1000000),((Get_TMR_PWROff_24S()*20)/1000000));
	//		printf("Dn=%X/Up=%X",G_Flg_ButtonDown , G_Flg_ButtonUp );

			printf("%d, %d\n",G_StallCnt , G_ReStar);

			



	//		if(G_Voltage_Battery > LowVoltage_10V5 && G_Voltage_Battery < LowVoltage_11V && G_Direction != _Brake && S_Debounce10V5_11V > 5 )
	//		{				
		//		printf("LV_10V5=%d\n",G_Voltage_Battery);					
	//		}		
			
		//	printf("%d%d%d\n",((SYS->P2_MFP & SYS_MFP_P24_PWM4)	?(1):(0)), ((SYS->P2_MFP & SYS_MFP_P22_PWM2)	?(1):(0)) ,((SYS->P2_MFP & SYS_MFP_P20_PWM0)	?(1):(0)) );
		//	printf("%d%d%d\n",P25,P23,P21                );
			printf("\n");

	//		printf(" CNR0=%d, CMR0=%d | CNR2=%d, CMR2=%d | CNR0=%d, CMR0=%d\n\n" ,PWMA->CNR0,PWMA->CMR0 ,  PWMA->CNR2 , PWMA->CMR2  ,PWMB->CNR0,PWMB->CMR0);

#endif


			}
		
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif	


	if(G_ID_Status == _NG )
	{	
	
	//	LED_InterlaceFlicker();	

	}
	else
	{
	
	#if ( ScrewType != _BS_PType_4Nm5 && ScrewType != _BS_PType_3Nm )		
		CheckDirection();
	#endif	

#if ( ScrewType == _BS_PType_4Nm5 || ScrewType == _BS_PType_3Nm)	
		if(TMR_ReStart300ms.u8_OnOFF == ON && (((Get_TMR_ReStart300ms()*20)/1000 ) > 250) /*&&  (StartButton  < Up)*//* && (G_NTC_PCB < PCB_TEMP_70)*/)
#elif( ScrewType == _PTA_B120 ||ScrewType == _PTA_X50F  || ScrewType == _BT_3Nm || ScrewType == _BT_2NmFast || ScrewType == _BS_1Nm2)
		if(        G_ScrewRPM == 0            &&  TMR_ReStart300ms.u8_OnOFF == ON && (((Get_TMR_ReStart300ms()*20)/1000 ) > 400) &&  (StartButton == Up)/* && (G_NTC_PCB < PCB_TEMP_70) */)
#endif			
		{
//			printf("								500\n");
			G_ReStar = ENABLE; 
		}
		
		if(TMR_ReStart300ms.u8_OnOFF == ON && (((Get_TMR_ReStart300ms()*20)/1000 ) > 80)  )
		{
			UH_OFF();	VH_OFF();	WH_OFF();	
			
			UL_OFF();	VL_OFF();	WL_OFF();

		}


#if ( ScrewType == _BS_PType_4Nm5 || ScrewType == _BS_PType_3Nm )	
		if(StartButton > Down  )			
#elif( ScrewType == _PTA_B120 ||ScrewType == _PTA_X50F   || ScrewType == _BT_3Nm || ScrewType == _BT_2NmFast || ScrewType == _BS_1Nm2)
		if(StartButton == Down  )			
#endif
		{
			//	printf("Dn\n");		
				S_Flag = TRUE;
				Reset_TMR_PWROff_24S();
				Reset_TMR_PWROff_3S();
				if((G_Voltage_Battery < LowVoltage_10V5) && (G_ScrewRPM == 0))
				{
				       S_Debounce10V5++;				
				}
				else
					S_Debounce10V5=0;				
				

//				GetADCValue(); printf("v=%d\n",G_Voltage_Battery);
				if(G_Voltage_Battery < LowVoltage_11V && (Get_TMR_MotorRuning_1Min()*20 < 200))
				{
					S_Debounce10V5_11V = TRUE;	
				}


					
				if(G_Voltage_Battery < LowVoltage_11V && G_ScrewRPM == 0  && G_Direction == _Brake && S_Flg_DirIsBrakeOf10V5_11V == 0 && G_3STimeOut == False)
				{		printf("123\n");			
					S_Flg_DirIsBrakeOf10V5_11V = 1;
					C_Buzzer_ON;
					C_NG_RED_ON;
					TIMER_Stop(TIMER1); 
					Delayms(50);
					TIMER_Start(TIMER1); 			
					Delayms(50);
					C_Buzzer_OFF;
					C_NG_RED_OFF;					
				}		
			
			
				if((S_Debounce10V5 >= 60) /*&& (G_Direction != _Brake)*/ && (S_Flg_LowBattery == 0))
				{
					G_Flg_ButtonUp = 0;
					S_Flg_LowBattery = 1;
					printf("low battery\n");
							printf("%d%d%d\n",((SYS->P2_MFP & SYS_MFP_P24_PWM4)	?(1):(0)), ((SYS->P2_MFP & SYS_MFP_P22_PWM2)	?(1):(0)) ,((SYS->P2_MFP & SYS_MFP_P20_PWM0)	?(1):(0)) );
							printf("%d%d%d\n",P25,P23,P21);    
					G_BuzzerLED = eBehaving_OutOfBattery;
				}		
#if( ScrewType == _BS_PType_4Nm5 || ScrewType == _BS_PType_3Nm)									
				else if(G_Direction != _Brake && G_Voltage_Battery > LowVoltage_10V5 && S_Debounce10V5 == 0 && G_ReStar == ENABLE /* && G_ScrewRPM == 0 */)
#elif( ScrewType == _PTA_B120 ||ScrewType == _PTA_X50F  || ScrewType == _BS_1Nm2 )		
				else if(G_Direction != _Brake && G_Voltage_Battery > LowVoltage_10V5 && S_Debounce10V5 == 0 && G_ReStar == ENABLE  && G_ScrewRPM == 0 && G_StartInAdc > START_THRESHOLD )
#endif
				
				{
					if( (G_Flg_ButtonDown & 0x01) == 0)
					{
						if(G_PCBStatus == ePCBStatus_Normal && G_MotorStatus == eMotorStatus_Normal && G_BatteryStatus == eBatteryStatus_Normal ) //( G_NTC_Motor < MOTOR_TEMP_90 && G_NTC_PCB < PCB_TEMP_80 )
						{

							Stop_TMR_ReStart300ms(); Reset_TMR_ReStart300ms();
							G_Flg_ButtonDown = 1; 	
							G_Flg_ButtonUp = 0;


							
							//PWMMinCMR; //(PWMA->CNR0 / 2);	


											S_MaxADC = 261;
											S_MinADC = 260;

											
											G_3STimeOut = False;

											gu32MaxCurrent = G_Current = 0;


											
											printf("\n");

											printf("Dn=");	
											if(G_Status_Item == eFastMode)
												printf("F:%d\n",G_Direction);
											else if(G_Status_Item == eSlowMode)	
												printf("*S\n");
											else  if(G_Status_Item == eIDError)	
											printf("** ID Error **\n");

											printf("HS=%d\n",HALL_STATUS);

											printf("V=%d\n",G_Voltage_Battery);

											
											printf("\n");	

											G_BuzzerLED = eBehaving_OFF;
											G_StallCnt = 0;

											
											if(G_Status_Item == eSlowMode && G_Voltage_Battery > LowVoltage_11V )
												C_D2_Blue_On;
										//	S_StallCount = 0;

											#if ( ScrewType == _BS_PType_4Nm5 || ScrewType == _BS_PType_3Nm )	
												G_Direction = _Brake;
												CheckDirection();
											#endif	

										TIMER_Stop(TIMER0); 
										TIMER_Stop(TIMER1); 

											UH_OFF();	
											VH_OFF();	
											WH_OFF();
						
											UL_ON();
											VL_ON();
											WL_ON();
									
										Delayms(20);
										TIMER_Start(TIMER0); 			
										TIMER_Start(TIMER1); 	

											UL_OFF();
											VL_OFF();
											WL_OFF();
											
										Delayms(20);



#if ( ScrewType == _BS_PType_4Nm5)	
												G_PwmCMR=	 ((PWMMaxCMR *70)/100);

#elif( ScrewType == _BS_PType_3Nm)	


												G_PwmCMR=	 ((PWMMaxCMR *60)/100);

#elif( ScrewType == _PTA_B120 ||ScrewType == _PTA_X50F )

				//Delayms(2);

				//GetADCValue();
				//printf("SSS = %d \n",G_StartInAdc);

					if(SPEED == HI )
					{
						printf("Speed HI\n");
						gu8StartSpeed = HI;
						gu8TurboCnt = 0;
					/*	
						if(G_Voltage_Battery > 2000)		G_PwmCMR= (PWMMaxCMR * 25)/100;						
						else	if(G_Voltage_Battery > 1950)	G_PwmCMR= (PWMMaxCMR * 31)/100;						
						else	if(G_Voltage_Battery > 1900)	G_PwmCMR= (PWMMaxCMR * 37)/100;						
						else	if(G_Voltage_Battery > 1850)	G_PwmCMR= (PWMMaxCMR * 43)/100;
						else	if(G_Voltage_Battery > 1800)	G_PwmCMR= (PWMMaxCMR * 49)/100;						
						else	if(G_Voltage_Battery > 1750)	G_PwmCMR= (PWMMaxCMR * 55)/100;						
						else								G_PwmCMR= (PWMMaxCMR * 58)/100;
*/
						

						
					}
					else
					{
						printf("Speed LO\n");
						gu8TurboCnt = 0;

					/*	if(G_Voltage_Battery > 2000)		G_PwmCMR= (PWMMaxCMR * 15)/100;
						else	if(G_Voltage_Battery > 1950)	G_PwmCMR= (PWMMaxCMR * 17)/100;
						else	if(G_Voltage_Battery > 1900)	G_PwmCMR= (PWMMaxCMR * 19)/100;
						else	if(G_Voltage_Battery > 1850)	G_PwmCMR= (PWMMaxCMR * 21)/100;
						else	if(G_Voltage_Battery > 1800)	G_PwmCMR= (PWMMaxCMR * 23)/100;
						else	if(G_Voltage_Battery > 1750)	G_PwmCMR= (PWMMaxCMR * 25)/100;
						else								G_PwmCMR= (PWMMaxCMR * 26)/100;
*/
						
						gu8StartSpeed = LO;
					}



#elif( ScrewType == _BT_3Nm)								
												G_PwmCMR= (PWMMaxCMR * 30)/100;
#elif( ScrewType == _BT_2NmFast)



											if(G_Direction == _CCW)
											{
												
												G_PwmCMR = ((PWMMaxCMR *40)/100);
											}
											else
											{
												G_PwmCMR = ((PWMMaxCMR *15)/100);
											}


#elif( ScrewType == _BS_1Nm2)

											if(G_Status_Item == eFastMode)
												G_PwmCMR=	 ((PWMMaxCMR *70)/100);
											else
												G_PwmCMR=	 ((PWMMaxCMR *25)/100);
#else
												G_PwmCMR= PWMMinCMR;
#endif



											LIGHT_ON;
											MotorStart(ENABLE,G_Direction);
											Start_TMR_ID_OK();// C_Buzzer_ON;			
											G_StartupDirectin = G_Direction;	

											Reset_TMR_PWROff_3S();			Stop_TMR_PWROff_3S();
											Reset_TMR_PWROff_24S();		Stop_TMR_PWROff_24S();
											Reset_TMR_Stop_10S();			Stop_TMR_Stop_10S();
											Start_TMR_MotorRuning_1Min();

											NVIC_EnableIRQ(GPIO_P0P1_IRQn);	
											//	Delayms(100);
											 // 	NVIC_SetPriority(TMR0_IRQn, 		0);		
											//	NVIC_SetPriority(GPIO_P0P1_IRQn, 1);
											 // 	NVIC_SetPriority(TMR1_IRQn, 		2);	
										}
									//	else
									//	{
									//		printf("123321\n");
									//		G_BuzzerLED = eBehaving_TempOverHeat;
					//	}	



	
					}

				}

				else if(G_Direction == _Brake && ((G_Flg_ButtonUp == 1 && G_Flg_ButtonDown == 0)||(G_IsFirstBtnDwn == False&&G_Flg_ButtonUp == 0 && G_Flg_ButtonDown == 0)))
				{
					LIGHT_ON;
					printf("Brake\n");
					G_Flg_ButtonUp = 0;		G_Flg_ButtonDown = 1; 	
					
					WDT_Close(); 	TIMER_Stop(TIMER0); 	TIMER_Stop(TIMER1); 				

					C_NG_RED_ON; 	C_Buzzer_ON;		Delayms(500);					
					C_NG_RED_OFF;	C_Buzzer_OFF;	Delayms(100);
					C_Buzzer_ON;						Delayms(200);
					C_Buzzer_OFF;					
					
					WDT_Init(); 		TIMER_Start(TIMER0); 	TIMER_Start(TIMER1); 	
				}
				
			if(G_Flg_ButtonDown == 1 &&  S_Debounce10V5_11V == TRUE && G_3STimeOut == False && G_Direction != _Brake  && G_PCBStatus == ePCBStatus_Normal && G_MotorStatus == eMotorStatus_Normal && G_BatteryStatus == eBatteryStatus_Normal)
			{
		
				G_BuzzerLED = eBehaving_LowBattery;
			}
		}
#if ( ScrewType == _BS_PType_4Nm5 || ScrewType == _BS_PType_3Nm )	
		else if(StartButton < Up && S_Flag == TRUE )
#elif( ScrewType == _PTA_B120 ||ScrewType == _PTA_X50F  || ScrewType == _BT_3Nm || ScrewType == _BT_2NmFast || ScrewType == _BS_1Nm2)
		else if(StartButton == Up && S_Flag == TRUE )
#endif		
		{

			if((G_Flg_ButtonUp & 0x01) == 0)
			{		

				printf("\nUp\n");				
				StartCMR = 110;
				LIGHT_OFF;
				gu8TurboCnt = FALSE;
				

				if(G_Flg_ButtonDown == 1 &&  S_Debounce10V5_11V == TRUE && G_3STimeOut == False && G_Direction != _Brake  && G_PCBStatus == ePCBStatus_Normal && G_MotorStatus == eMotorStatus_Normal && G_BatteryStatus == eBatteryStatus_Normal)
				{
		
					G_BuzzerLED = eBehaving_Idle_0s5;
					Buzzer_LED_Behaving(&G_BuzzerLED);/* 讓 S_Count = 0 */

				}



				

				UL_OFF(); WL_OFF(); VL_OFF();
				G_BrakeCurrent = 0;
			//		if((SYS->P2_MFP) == 0)			printf(" keyUp ==>  UP_Mos=off,");		else			printf(" keyUp ==>  UP_Mos=on,");
			//		printf(" %d %d %d\n",P21,P23,P25);
		
			
				G_PwmCMR= PWMMinCMR;

	
				Start_TMR_ReStart300ms();
#if  ( ScrewType != _BS_PType_4Nm5 && ScrewType != _BS_PType_3Nm )					
				G_ReStar = DISABLE;
#endif





				G_StallCnt = 0;
				
				Stop_TMR_MotorStall();
				Reset_TMR_MotorStall();

				Stop_TMR_MotorRuning_1Min();
				Reset_TMR_MotorRuning_1Min();
				
				G_ScrewLock = FALSE;
				S_Debounce10V5 = 0;
				S_Debounce10V5_11V= FALSE;
				S_Flg_LowBattery = 0;
				G_Flg_ButtonUp = 1; 
				C_D2_Blue_Off;

		   		if(G_PCBStatus == ePCBStatus_Normal && G_MotorStatus == eMotorStatus_Normal && G_BatteryStatus == eBatteryStatus_Normal) 
		   		{
					C_NG_RED_OFF; //printf("11.Red off\n");
		   		}
				
				G_IsFirstBtnDwn = True;


				Reset_TMR_MotorTempOverHeat_3S();
				Stop_TMR_MotorTempOverHeat_3S();

				
			#if 0

				if(G_Voltage_Battery > LowVoltage_10V5 ) /*電壓過低不要清除 */
				{
					if(G_BuzzerLED != eBehaving_ClutchOK_300ms_G_LED_OFF && G_BuzzerLED != eBehaving_LowBattery ) 
					{
					   if(G_PCBStatus == ePCBStatus_Normal && G_MotorStatus == eMotorStatus_Normal && G_BatteryStatus == eBatteryStatus_Normal)  
					   {
					//	 printf("22. Behaving off\n");
					//	G_BuzzerLED = eBehaving_OFF;  
					//	Buzzer_LED_Behaving(&G_BuzzerLED);/* 讓 S_Count = 0 */
					   }
					}
				}


				if( G_Direction == _Brake && G_Status_Item == eSlowMode /*||G_Voltage_Battery < LowVoltage_10V5)*/)
				{
					printf("11\n");
					C_D2_Blue_On;
				}	
				if(G_IsFirstBtnDwn == False && G_Direction == _Brake)
				{
					if(G_Status_Item == eSlowMode)
					{
						C_D2_Blue_On;
						printf("22\n");
					}				
				}
			#endif
				
				if( G_Status_Item == eFastMode || G_Voltage_Battery  < LowVoltage_10V5)
				{

					Reset_TMR_PWROff_3S();		Stop_TMR_PWROff_3S();
					Reset_TMR_PWROff_24S();	Stop_TMR_PWROff_24S();
					Start_TMR_PWROff_3S();
					Start_TMR_PWROff_24S();	
					
					G_Flg_ButtonDown = 0;
					LED_Flicker_B(OFF);		
				}	


				
				if(G_ClutchBrakFlow != eClutchBrake_Scuess && G_PCBStatus == ePCBStatus_Normal && G_MotorStatus == eMotorStatus_Normal && G_BatteryStatus == eBatteryStatus_Normal && Get_TMR_MotorRuning_1Min() < _1Min)
				{
					

					if(G_Direction == _Brake )
					{	

						if(mu8DoubleCount == 0)
						{
							printf("11=%d\n",((Get_TMR_Double_Enable()*20)/1000) );
							
							Reset_TMR_Double_Enable();
							Start_TMR_Double_Enable();
							mu8DoubleCount++;

						}
						else if(mu8DoubleCount == 1 && (((Get_TMR_Double_Enable()*20)/1000) < 500) )
						{
							printf("22=%d\n",((Get_TMR_Double_Enable()*20)/1000) );
							//LIGHT_ON; 
							mu8DoubleCount = 0;
						}
					/*
						else
						{
							printf("33=%d\n",((Get_TMR_Double_Enable()*20)/1000) );

							//Reset_TMR_Double_Enable();
							//Start_TMR_Double_Enable();
							mu8DoubleCount = 0;
							Reset_TMR_Double_Enable();
							Stop_TMR_Double_Enable();
						}
					*/
					}
					else
					{
						if(gu8BrakeStop == FALSE)
						{
					
							C_NG_RED_ON;			C_Buzzer_ON;
			 				WDT_Close();
							TIMER_Stop(TIMER1); 					
							Delayms(300);				
							Delayms(300);				
							Delayms(21);					
							TIMER_Start(TIMER1); 			
							WDT_Init();						
							C_NG_RED_OFF;			C_Buzzer_OFF;					
							G_ScrewRPM = 0;
							G_MotorRPM = 0;
						}	
					}					
				}


			//	Motor(DISABLE, G_Direction);
				
#if( ScrewType == _BS_PType_4Nm5 || ScrewType == _BS_PType_3Nm)		
			if(G_MotorStatus != eMotorStatus_Over100)
				G_MotorStatus = eMotorStatus_Normal;
#elif( ScrewType == _PTA_B120 ||ScrewType == _PTA_X50F  || ScrewType == _BS_1Nm2)					
		//		G_MotorStatus = eMotorStatus_Normal;
#endif

				
				Reset_TMR_MotorRuning_1Min();
				Stop_TMR_MotorRuning_1Min();
				if( G_Status_Item == eSlowMode)
				{
					C_D1_Red_Off;
					if(G_Voltage_Battery > LowVoltage_11V)
						C_D2_Blue_On;
					G_Flg_ButtonDown = 0;

					Reset_TMR_PWROff_3S();		Stop_TMR_PWROff_3S();
					Reset_TMR_PWROff_24S();		Stop_TMR_PWROff_24S();
					Start_TMR_PWROff_3S();	
					Start_TMR_PWROff_24S();
					LED_Flicker_B(OFF);		
				}
				
				if(G_MotorStatus == eMotorStatus_Stop ||G_MotorStatus == eMotorStatus_OverCurrent||G_MotorStatus == eMotorStatus_Stall)
				{
					Reset_TMR_Stop_10S();		Stop_TMR_Stop_10S();
					Start_TMR_Stop_10S();
				}
				
				G_ClutchBrakFlow = eClutchBrake_None;
				gu8BrakeStop = FALSE;
			}

			
			
//			if( G_Flg_ButtonUp == 1 && S_Debounce10V5_11V > 200 && G_3STimeOut == False && G_Direction != _Brake  && G_PCBStatus == ePCBStatus_Normal && G_MotorStatus == eMotorStatus_Normal && G_BatteryStatus == eBatteryStatus_Normal)
//			{
//				G_BuzzerLED = eBehaving_LowBattery;
//			}

		}


		///////////////////////////////////////	
		/*    處理 非StartButton Down / Up            */
		////////////////////////////////////////	

			if((G_Direction == _Brake ) && (((Get_TMR_Double_Enable()*20)/1000) > 500) && mu8DoubleCount == 1)
			{
				printf("Reset Double\n");
				Reset_TMR_Double_Enable();
				Stop_TMR_Double_Enable();	
				mu8DoubleCount = 0;
			}
		

#if( ScrewType == _PTA_B120 ||ScrewType == _PTA_X50F  || ScrewType == _BT_3Nm || ScrewType == _BT_2NmFast || ScrewType == _BS_1Nm2)
		
		if(G_StartupDirectin != G_Direction)
			S_MiddleBrakeCnt ++;
		else
			S_MiddleBrakeCnt = 0;

		/* 處理 運轉中 去切換 方向版  */
        	if(G_StartupDirectin != G_Direction && G_ScrewRPM != 0 && S_MiddleBrakeCnt >= 2) 
        	{
			printf("*Error*\n");
			G_ScrewRPM = 0;
			G_MotorRPM = 0;
			Motor(DISABLE, G_Direction);
			Stop_TMR_MotorStall();
			Reset_TMR_MotorStall();			
		}
#endif	

/*
		if(StartButton == Up && (S_BeforeDirection == _CW ||S_BeforeDirection == _CCW) && G_Direction == _Brake)
		{			

			printf("12345\n");
		//	G_BuzzerLED = eBehaving_OFF;

			if(G_Status_Item == eSlowMode && G_3STimeOut == False)
			{
				C_D2_Blue_On;
			}			
		}
*/
		Buzzer_LED_Behaving(&G_BuzzerLED);
	
//		S_BeforeDirection = G_Direction;



		PCB_Temperature_Check();
		Motor_Temperature_Check();
		OverCurrent_Check();	
		
 //		BatteryNTC_Check();



		if(Get_TMR_ID_OK() == _1S) // 1S
		{
			C_Buzzer_OFF;
			Stop_TMR_ID_OK();
			Reset_TMR_ID_OK();

		}

		if((Get_TMR_PWROff_24S() > _24S) && G_ID_Status  != _NG)//24S
		{
			 	printf("                                ** 24s  Power off **\n");

				C_OK_GREEN_ON;	C_BAT_RED_ON;	C_NG_RED_ON;

				TIMER_Stop(TIMER1); 	Delayms(50);
				
				C_PWRKeep_OFF;
		}

		if((Get_TMR_Stop_10S() >= _10S) && G_ID_Status  != _NG)//24S
		{

		// 	printf("                                                                               ** 10s  **\n");

			G_MotorStatus = eMotorStatus_Normal;

		}

		

		if((Get_TMR_PWROff_3S() > _3S) && G_ID_Status != _NG)//5s
		{
		 	printf("                 ** 3s LED OFF and Buzzer OFF **\n");


#if( ScrewType == _BS_PType_4Nm5 || ScrewType == _BS_PType_3Nm)					
//				G_MotorStatus = eMotorStatus_Normal;
#elif( ScrewType == _PTA_B120 ||ScrewType == _PTA_X50F  || ScrewType == _BS_1Nm2)					
//				G_MotorStatus = eMotorStatus_Normal;
#endif
			if(G_Direction == _Brake);
			else	LIGHT_OFF;
			
			C_OK_GREEN_OFF;	C_D1_Red_Off;
			C_D2_Blue_Off;	C_NG_RED_OFF;				
			C_Buzzer_OFF;

			G_ScrewRPM = 0;
			G_MotorRPM = 0;
			G_BuzzerLED = eBehaving_OFF;
			G_3STimeOut = True;
			Reset_TMR_PWROff_3S(); Stop_TMR_PWROff_3S();		
		}

		


	}	




}











/**
 * @brief       Port2/Port3/Port4 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Port2/Port3/Port4 default IRQ, declared in startup_M051Series.s.
 */
void GPIOP2P3P4_IRQHandler(void)
{

    	if(GPIO_GET_INT_FLAG(P3, BIT3))
    	{
       	 GPIO_CLR_INT_FLAG(P3, BIT3);
       	 printf("P3.3 INT occurred.\n");
    	}
    	else
    	{
        	P2->ISRC = P2->ISRC;
        	P3->ISRC = P3->ISRC;
        	P4->ISRC = P4->ISRC;
        	printf("Un-expected interrupts.\n");
    	}


}


/*---------------------------------------------------------------------------------------------------------*/
/* ADC interrupt handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#if 0
void ADC_IRQHandler(void)
{
//    g_u32AdcIntFlag = 1;
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT); /* clear the A/D conversion flag */

	//printf("******\n");
}
#endif
uint8_t g_u8DebounceFlag = 0;
extern uint32_t gpio_debunce_10ms;
void EINT0_IRQHandler(void)
{
	static unsigned char S_DownKey,S_UPKey;

	if(GPIO_GET_INT_FLAG(P3, BIT2) && StartButton == Down)
	{
		NVIC_DisableIRQ(TMR0_IRQn);
		NVIC_DisableIRQ(TMR1_IRQn);
	
	    	GPIO_CLR_INT_FLAG(P3, BIT2);

	//	printf("^%d,%d\n",G_ClutchBrakFlow,P32); 

		if(ClutchBrake == Hi && G_ClutchBrakFlow == eClutchBrake_None)
		{
    	//		printf("H\n");

		//	printf("   >_<1 I=%d, Duty=%d,  Dn=%X/Up=%X\n \n", G_Current , G_PwmCMR,G_Flg_ButtonDown , G_Flg_ButtonUp);	
		//	printf("\n   >_<1 Dn=%X/Up=%X\n \n",G_Flg_ButtonDown , G_Flg_ButtonUp);	
					S_DownKey = G_Flg_ButtonDown;
					S_UPKey = G_Flg_ButtonUp;
		
			G_ClutchBrakFlow = eClutchBrake_Hi;
		}



		if(ClutchBrake == Low  && G_ClutchBrakFlow == eClutchBrake_Hi )
		{	
			G_ClutchBrakFlow = eClutchBrake_Low;
		//	printf("  L\n");

		//printf("Motor Disable\n"); 

			Motor(DISABLE, G_Direction);
			Start_TMR_ReStart300ms();
			G_ReStar = DISABLE;
			
		//	printf("Low\n");

		//	printf(">_<3-ClutchBrake I=%d, Duty=%d **\n", G_Current , G_PwmCMR);	

			G_BrakeCurrent = G_Current;
		
			G_Count++;


			if(G_BuzzerLED != eBehaving_LowBattery)	
				G_BuzzerLED = eBehaving_OFF;

			G_ClutchBrakFlow=eClutchBrake_Scuess;

			

			C_D2_Blue_Off;					C_NG_RED_OFF;
			C_OK_GREEN_ON;
		#if(ClutchBrakeWithBuzzer == TRUE)
			C_Buzzer_ON;
		#endif
			
			TIMER_Stop(TIMER1); 
			Delayms(270);
			TIMER_Start(TIMER1); 						
			//Delayms(30);


		//	C_D2_Blue_Off;


		//	G_BuzzerLED = eBehaving_ClutchOK_300ms_G_LED_OFF;  
		//	Buzzer_LED_Behaving(&G_BuzzerLED);


			if( G_Status_Item == eSlowMode && G_Voltage_Battery > LowVoltage_11V)
				C_D2_Blue_On;
				

			Stop_TMR_MotorStall();
			Reset_TMR_MotorStall();




			G_ScrewRPM = 0;
			G_MotorRPM = 0;
			G_ClutchBrakFlow = eClutchBrake_Scuess;


			G_Flg_ButtonDown = S_DownKey;
			G_Flg_ButtonUp = S_UPKey;
			
			NVIC_EnableIRQ(TMR0_IRQn);
			NVIC_EnableIRQ(TMR1_IRQn);

	//		printf("\n>_<3 Dn=%X/Up=%X\n\n",G_Flg_ButtonDown , G_Flg_ButtonUp );

		}




	}

	


	
}









