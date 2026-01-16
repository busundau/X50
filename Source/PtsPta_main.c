/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 15/05/22 2:51p $
 * @brief    Show the usage of GPIO interrupt function.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M051Series.h"
#include "string.h"
#include "PtsPta_Value.h"
#include "PtsPta_Global.h"
#include "PtsPta_TypeDef.h"
#include "PtsPta_Init.h"
#include "PtsPta_InputOutput.h"
#include "PtsPta_NVRAM.h"
#include "PtsPta_FMC.h"
#include "PtsPta_Motor.h"
#include "PtsPta_ADC.h"
#include "pid.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#define PLL_CLOCK 50000000
#if defined(__GNUC__)
#define RXBUFSIZE 128
#else
#define RXBUFSIZE 1024
#endif
extern void default_init(void);
extern void motor_algorithm(void);
extern void parm_init(void);
extern void uvw_check_function(void);
extern void Exponential_PWM_Ramp_CCW(void);
extern int32_t SpdPIDCalc_HI(float NextPoint);
extern int32_t SpdPIDCalc_LO(float NextPoint);
extern void PID_ParamInit_HI_CCW(void);
extern void PID_ParamInit_HI_CW(void);
extern void PID_ParamInit_LO_CCW(void);
extern void PID_ParamInit_LO_CW(void);
extern void detect_enter_boot_speed(uint16_t rpm, uint16_t current);
uint8_t g_u8RecData[RXBUFSIZE] = {0};
volatile uint32_t t = 0;
extern volatile uint32_t t2;

volatile uint8_t g_FoolProof = 0; // 防呆旗標
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz clock */
  //  CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
 //   CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

 /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_PLL_STB_Msk |  CLK_CLKSTATUS_OSC22M_STB_Msk);


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
/*
Code Base On : M051BSPv3.01.001

#if ( MotorType == Motor_PTM100 )	

#elif( MotorType == Motor_TBK )

#endif
//TX pin
#if 1	
	if(P31 == 1)		P31 =0;	else			P31=1;
#endif

//RX pin
#if 1	
	if(P30 == 1)		P30 =0;	else			P30=1;
#endif



ADC:50		Current:0.26A
ADC:73		Current:0.38A
ADC:100		Current:0.50A
ADC:125		Current:0.62A
ADC:150		Current:0.74A
ADC:177		Current:0.85A
ADC:205		Current:0.98A
ADC:235		Current:1.15A
ADC:244		Current:1.18A



POLES2_SLOTS6,
ScrewType=_PTA,
IDByPass=FALSE,
ClutchBrakeWithBuzzer=TRUE,
MODEL=PTA_B120




*/

////////////////////////////////


volatile uint32_t g_u32comRbytes = 0;
volatile uint32_t g_u32comRhead = 0;
volatile uint32_t g_u32comRtail = 0;

void InitSystem(void)
{
	// ... 其他初始化 ...

	// 開機時檢查按鈕是否被壓著
	if (StartButton == 1)
	{
		g_FoolProof = 1; // 進入防呆狀態
	}
}
uint8_t copy_StartButton = 1;
extern unsigned char T_1ms, T_1ms222;
extern unsigned char S_10ms;

/*-------------------------------------------
  範例：P1.3 在 ADC 和 GPIO 間切換
  - ADC 模式：AIN3
  - GPIO 模式：控制馬達或輸出測試
-------------------------------------------*/

extern unsigned int Temp_var[3][50];
extern unsigned int HI_CW_var[3][50];
extern unsigned int HI_CCW_var[3][50];
extern unsigned int LO_CW_var[3][50];
extern unsigned int LO_CCW_var[3][50];
int main(void)
{

	int i;
	unsigned int L_Ver;

    	/* Unlock protected registers */
    	SYS_UnlockReg();

    	/* Init System, peripheral clock and multi-function I/O */
    	SYS_Init();

    	/* Lock protected registers */
    	SYS_LockReg();


	Variable_Init();

    	GPIO_Init();	

	/* Init UART0 for printf */
	UART0_Init();
	PID_ParamInit_LO_CW();
	PID_ParamInit_LO_CCW();
	PID_ParamInit_HI_CW();
	PID_ParamInit_HI_CCW();
	PWM_Init();
	I2C_Init();
	ADC_Init();
	
	L_Ver = FWVer;

 printf("\n\nCPU  %d Hz\n", SystemCoreClock);
    	printf("*****************\n");
		
#ifdef Scott_Debug
	printf("** PTA-150 Scott_Debug ");
#else

#if(ScrewType == _PTA_X50F )	
	printf("** PTA-X50F ");	
#endif

#endif

#if(POLES2_SLOTS6)
	printf("Poles 2__");
#elif (POLES4_SLOTS6)
	printf("Poles 4__ ");	
#endif

#if(C_POLES == Poles_UVW)
	printf("UVW\n");			
#elif(C_POLES == Poles_UWV)
	printf("UWV\n");			
#elif(C_POLES == Poles_VUW)
	printf("VUW\n");			
#elif(C_POLES == Poles_WUV)
	printf("WUV\n");			
#elif(C_POLES == Poles_VWU)
	printf("VWU\n");			
#elif(C_POLES == Poles_WVU)
	printf("WVU\n");						
#endif



#if(IDByPass == TRUE)
	printf("** ID:No, ");
#else
	printf("** ID:Yes,");
#endif

#if(ClutchBrakeWithBuzzer == TRUE)
	printf(" Buzzer:Yes\n");
#else
	printf(" Buzzer: No\n");
#endif





	if(L_Ver%100 <10 )
		printf("** F/W = V%d.0%dA \n", L_Ver/100,L_Ver%100); //20250516 jacob
	else	
		printf("** F/W = V%d.%dA \n", L_Ver/100,L_Ver%100);  //20250516 jacob

	printf("** Max Duty = %d%% \n", (PWMMaxCMR *100)/(PWMA->CNR0));
	
	printf("** SVN = %s   \n", SVN);	
    	printf("*****************\n");		


	printf("1/4 = %d, %d\n", gu32FastRpmQuarter , gu32SlowRpmQuarter);



	GetADCValue();

#if(IDByPass == TRUE)
//	G_ID_Status = _OK;
//	printf("ID By Pass\n");
#else
//  	G_ID_Status = CHECK_ID();
#endif


/*
		printf("0x1F000 = %X\n",ReadFlashData(0x1F000));
		printf("0x1F001 = %X\n",ReadFlashData(0x1F001));
		printf("0x1F002 = %X\n",ReadFlashData(0x1F002));
		printf("0x1F003 = %X\n",ReadFlashData(0x1F003));
		printf("0x1F004 = %X\n",ReadFlashData(0x1F004));
		printf("0x1F005 = %X\n",ReadFlashData(0x1F005));
		printf("0x1F006 = %X\n",ReadFlashData(0x1F006));
		printf("0x1F007 = %X\n",ReadFlashData(0x1F007));		
printf("=========================================\n");
		printf("0x1FE00 = %X\n",ReadFlashData(0x1FE00));
		printf("0x1FE01 = %X\n",ReadFlashData(0x1FE01));
		printf("0x1FE02 = %X\n",ReadFlashData(0x1FE02));
		printf("0x1FE03 = %X\n",ReadFlashData(0x1FE03));		
		printf("0x1FE04 = %X\n",ReadFlashData(0x1FE04));
		printf("0x1FE05 = %X\n",ReadFlashData(0x1FE05));
		printf("0x1FE06 = %X\n",ReadFlashData(0x1FE06));
		printf("0x1FE07 = %X\n",ReadFlashData(0x1FE07));

*/

//	printf("Flash Current ADC = %d\n ", ((ReadFlashData(eAddrCurrent_Page7)<<8) + ReadFlashData(eAddrCurrent_Page7+4)));



/*		
	if(G_ID_Status == _OK)
	{
		printf("ID = OK\n");

		printf("%X\n",ReadFlashData(eAddrSpeed_Page0));
		if(ReadFlashData(eAddrSpeed_Page0)!= _FastRPM && ReadFlashData(eAddrSpeed_Page0)!= _SlowRPM)
		{
			printf("Data Flash is Null \n");
			SaveData2Flash(eAddrSpeed_Page0,_FastRPM);
			G_MotorSpeed = _FastRPM;
		}
		else
		{
			if(ReadFlashData(eAddrSpeed_Page0) == _FastRPM)
			{			
				G_Status_Item  = eFastMode;			
				printf("Fast Speed \n");
			}
			else if(ReadFlashData(eAddrSpeed_Page0) == _SlowRPM)
			{
				G_Status_Item  = eSlowMode;
				printf("Slow Speed \n");

			}
		}


		
	}
	else
	{
		printf("ID = NG\n");

	}
*/	

	printf("Hall Sensor= %d \n",HALL_STATUS);
			
            /* Enable PWMB NVIC */
	NVIC_EnableIRQ((IRQn_Type)(PWMA_IRQn));
	NVIC_EnableIRQ((IRQn_Type)(PWMB_IRQn));


	UH_OFF();	UL_OFF();
	VH_OFF();	VL_OFF();
	WH_OFF();	WL_OFF();

	TIMER_Init();

	
	CheckDirection();
	

#if 1
#if(WatchDog == ENABLE)	
	NVIC_SetPriority(WDT_IRQn, 		0);
#endif
	NVIC_SetPriority(EINT0_IRQn, 		1);	
/////////////////////////////////////////////////////////

	NVIC_SetPriority(GPIO_P0P1_IRQn, 1);

///////////////////////////////////////////////////
  	NVIC_SetPriority(TMR0_IRQn, 		0);

  	NVIC_SetPriority(TMR1_IRQn, 		3);	




#else
	
  	NVIC_SetPriority(TMR0_IRQn, 		0);		
	NVIC_SetPriority(GPIO_P0P1_IRQn, 1);
  	NVIC_SetPriority(EINT1_IRQn, 		2);	
  	NVIC_SetPriority(TMR1_IRQn, 		3);		
#endif
	


	printf("Battery =%d\n",G_Voltage_Battery);
#if(WatchDog == ENABLE)
	WDT_Init();
#endif
	// 開機時檢查按鈕是否被壓著
	// if (StartButton == Down) {
	// StartButton = Down;
	copy_StartButton = Down;
	g_FoolProof = 1; // 進入防呆狀態
	//}

	for (;;)
	{
		
		if (g_FoolProof)
		{
			if (copy_StartButton == Up)
			{
				// 使用者放開按鈕 → 解除防呆
				g_FoolProof = 0;
			}
			// 防呆中 → 不允許起子動作
		}
		//  printf("no press\r\n");
		if (g_FoolProof == 1)
			continue;

		/* if((S_10ms>=2)&&(StartButton == Down))
		{
			S_10ms=0;
			printf("%d,%d,%d,%d,1,1\n", G_ScrewRPM, G_Current, G_PwmCMR, G_Voltage_Battery); // 這個部分要搭配程式開cmk才會過，不然扭力輸出打cmk會不穩
		}
		*/

		if (StartButton == Down)
		{
			// printf("press\r\n");
			//  uvw_check_function();
			// parm_init();
			if (gu8StartSpeed == HI)
			{
				motor_algorithm_HI();
			}
			else
			{
				motor_algorithm_LO();
			}
			
			// OverCurrent_Check();
		}
		else
		{
			// printf("release");

			default_init();
		}

#if 0
#if (ScrewType == _BS_PType_4Nm5 || ScrewType == _BT_4Nm5)	
		
//			if( (((Get_TMR_MotorStall()*10)/1000 ) > 200) && (G_ScrewLock == FALSE) && (StartButton > Down  ) && (G_ClutchBrakFlow != eClutchBrake_Scuess))
			if(G_ClutchBrakFlow == eClutchBrake_None)	
			{	
				//printf("   11\n");
				if(G_ScrewLock == FALSE)
				{
					//printf("   22\n");

			
#if ( ScrewType == _BS_PType_4Nm5)	
					if(StartButton > Down  )			
#elif( ScrewType == _BT_4Nm5)
					if(StartButton == Down  )			
#endif		
					{
						//printf("   33\n");
							if(((Get_TMR_MotorStall()*10)/1000 ) > 200)
							{
								//printf("   44\n");
				
								G_ScrewLock =TRUE;
								printf("\n\n"); /* ³nµ²¦X ´ú¸Õ¨ì ³Ì°ª¹q¬y ADC = 862 */
								printf("                                                 ****************************************\n");
								printf("                                                 ****************************************\n");
								printf("                                                 **                                                               **\n");
								printf("                                                 ** Stall  protection =%dms, I=%d, Duty=%d ,Dir=%d, StrBum=%d **\n", ((Get_TMR_MotorStall()*10)/1000 ), G_Current , G_PwmDuty,G_Direction,StartButton);	
								printf("                                                 **                                                               **\n");
								printf("                                                 ****************************************\n");
								printf("                                                 ****************************************\n");
								printf("\n\n");
								Motor(DISABLE, G_Direction);
							}
					}
				}	
			}		
		#endif

		
	#endif	





/* ¹q ¬y «O Å@ */
    	#if 0
		#if ( ScrewType == _BS_PType_4Nm5)			
			if( G_Current >= 1300 && G_ScrewLock == FALSE)

		#elif( ScrewType == _BT_4Nm5 )							
			if( G_Current >= 750  && G_ScrewLock == FALSE)

		#elif( ScrewType == _BT_3Nm )							
			if( G_Current >= 650  && G_ScrewLock == FALSE)

		#elif( ScrewType == _BT_2Nm )							
			if( G_Current >= 650  && G_ScrewLock == FALSE)
				
		#elif( ScrewType == _BT_2NmFast)									
			if( G_Current >= 800 && G_ScrewLock == FALSE)
				
		#endif		
			{		
				G_ScrewLock =TRUE;
				printf("\n\n");
				printf("                                                 ****************************************\n");
				printf("                                                 ****************************************\n");
				printf("                                                 **                                                               **\n");
				printf("                                                 ** Current protection I=%d, Duty=%d **\n", G_Current , G_PwmCMR);	
				printf("                                                 **                                                               **\n");
				printf("                                                 ****************************************\n");
				printf("                                                 ****************************************\n");
				printf("\n\n");
				Motor(DISABLE, G_Direction);


			//	if(  ( ((ReadFlashData(0x1FE00)&0x0000ff00)<<8) + (ReadFlashData(0x1FE04)&0x000000ff)) <  G_Current )
				{					
				//	printf("Save Current to Flash\n");
				/*
					EraseFlash(eAddrCurrent_Page7);

					SYS_UnlockReg();
				 	//FMC_Open();
				    	FMC->ISPCON |=  FMC_ISPCON_ISPEN_Msk;
				       FMC_Write(eAddrCurrent_Page7, (G_Current & 0x0000ff00)>>8);
				       FMC_Write(eAddrCurrent_Page7 +4, (G_Current & 0x000000ff));
				    	SYS_LockReg();
				*/
						
						
				}

				
			}		
	#endif	

     	}
}
















/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
