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

void Variable_Init(void)
{
	u32 u32temp;
	//G_ID_Status = _NG;
	G_ID_Status = _OK;//id check ok jacob 20250516
	G_PwmCMR = PWMMinCMR;
//	G_ClutchSucess = FALSE;
	G_ClutchBrakFlow = eClutchBrake_None;
	G_IsFirstBtnDwn = False;
#if ( ScrewType == _BS_PType_4Nm5 || ScrewType == _BS_PType_3Nm)	
	G_Direction = _CW;
#elif( ScrewType == _PTA_B120 ||ScrewType == _PTA_X50F ||ScrewType == _BT_3Nm ||  ScrewType == _BT_2NmFast ||  ScrewType == _BS_1Nm2)
	G_Direction = _Brake;
#endif	
	G_3STimeOut = False;
	G_BuzzerLED = eBehaving_OFF;	
	Reset_TMR_PWROff_24S();
	Reset_TMR_PWROff_3S();
	Reset_TMR_Stop_10S();
	Reset_TMR_ReStart300ms();
	Stop_TMR_ReStart300ms();
	Reset_TMR_Double_Enable();
	Stop_TMR_Double_Enable();
	
	G_ScrewLock = FALSE;
	G_ReStar = ENABLE;
	G_Flg_ButtonUp = 0;
	G_Flg_ButtonDown = 0;
	G_MotorStatus = eMotorStatus_Normal;
	G_PCBStatus = ePCBStatus_Normal;
	G_BatteryStatus = eBatteryStatus_Normal;
	G_Speed = eSpeed_Middle;
	u32temp = FAST_RPM;
	gu32FastRpmQuarter= u32temp / 4;
	u32temp = SLOW_RPM;
	gu32SlowRpmQuarter= u32temp / 4;
	u32temp = SLOW_RPM;
	gu32SlowRpmHalf= u32temp / 2;
	gu8TurboCnt = 0;


}
void GPIO_Init(void)
{

    /* Disable the P1.0 - P1.3 digital input path to avoid the leakage current */
    GPIO_DISABLE_DIGITAL_PATH(P1, 0xFF);

    /*					PCB_NTC				Battery_NTC				P_Type Start				Motor_NTC				Current					Battery Voltage  */
    SYS->P1_MFP &= ~(  SYS_MFP_P10_Msk	|	SYS_MFP_P11_Msk		|	SYS_MFP_P12_Msk		|  	SYS_MFP_P14_Msk  	| 	SYS_MFP_P13_Msk 	| 	SYS_MFP_P16_Msk );
    SYS->P1_MFP |=      SYS_MFP_P10_AIN0	| 	SYS_MFP_P11_AIN1 	|	SYS_MFP_P12_AIN2	|	SYS_MFP_P14_AIN4 	|	SYS_MFP_P13_AIN3	|	SYS_MFP_P16_AIN6 ;





//UART0
	/* Set P3 multi-function pins for UART0 RXD and TXD */

#if(Pin7 == _UartFunc)	


	SYS->P3_MFP &= ~( SYS_MFP_P31_Msk);
	SYS->P3_MFP |= ( SYS_MFP_P31_TXD0/* | SYS_MFP_P33_INT1*/);
	GPIO_SetMode(P3, BIT1, 	GPIO_PMD_QUASI);


	SYS->P3_MFP &= ~( SYS_MFP_P30_Msk);
	SYS->P3_MFP |= ( SYS_MFP_P30_GPIO);
	GPIO_SetMode(P3, BIT0, 	GPIO_PMD_OUTPUT);
	P30 = 1;


	SYS->P2_MFP &= ~(SYS_MFP_P26_Msk | SYS_MFP_P27_Msk);
	SYS->P2_MFP |= ( SYS_MFP_P26_GPIO | SYS_MFP_P27_GPIO);

	GPIO_SetMode(P2, BIT6, 	GPIO_PMD_INPUT);
	GPIO_SetMode(P2, BIT7, 	GPIO_PMD_INPUT);



//for Debug Pin
//	SYS->P1_MFP &= ~( SYS_MFP_P13_Msk);
//	SYS->P1_MFP |= ( SYS_MFP_P13_GPIO);
//	GPIO_SetMode(P1, BIT3, 	GPIO_PMD_OUTPUT);
//	P13 = 1;



	
#else

	SYS->P3_MFP &= ~( SYS_MFP_P31_Msk | SYS_MFP_P30_Msk);
	SYS->P3_MFP |= ( SYS_MFP_P31_GPIO|SYS_MFP_P30_GPIO );

	GPIO_SetMode(P3, BIT0, 	GPIO_PMD_OUTPUT);
	GPIO_SetMode(P3, BIT1, 	GPIO_PMD_OUTPUT);

	P31 = 1;
	P30 = 1;

	SYS->P2_MFP &= ~(SYS_MFP_P26_Msk | SYS_MFP_P27_Msk);
	SYS->P2_MFP |= ( SYS_MFP_P26_GPIO | SYS_MFP_P27_GPIO);

	GPIO_SetMode(P2, BIT6, 	GPIO_PMD_INPUT);
	GPIO_SetMode(P2, BIT7, 	GPIO_PMD_INPUT);

	
#endif


	
//////////////////////////////////////////////////////////////////////////////////////

//I2C
    /* Set GPA multi-function pins for I2C0 SDA and SCL */
    SYS->P3_MFP &= ~(SYS_MFP_P34_Msk | SYS_MFP_P35_Msk);
    SYS->P3_MFP |= SYS_MFP_P34_SDA0 | SYS_MFP_P35_SCL0;
	
   GPIO_SetMode(P3, BIT4, 	GPIO_PMD_OPEN_DRAIN);
   GPIO_SetMode(P3, BIT5, 	GPIO_PMD_OPEN_DRAIN);
//////////////////////////////////////////////////////////////////////////////////////
/*	
	HALL Sensor P0.0=>Hall_U
	  		     P0.1=>Hall_V
	  		     P0.2=>Hall_W       
  */				  
	//Hall_U = P0.0

	SYS->P0_MFP &= ~(  SYS_MFP_P00_Msk	|	SYS_MFP_P01_Msk		|	SYS_MFP_P02_Msk);
    	SYS->P0_MFP |=      SYS_MFP_P00_GPIO| 	SYS_MFP_P01_GPIO	|	SYS_MFP_P02_GPIO;


	GPIO_SetMode(P0, BIT0, GPIO_PMD_INPUT);
    	GPIO_EnableInt(P0, 0, GPIO_INT_BOTH_EDGE);   	

	//Hall_V = P0.1	
	GPIO_SetMode(P0, BIT1, GPIO_PMD_INPUT);
    	GPIO_EnableInt(P0, 1, GPIO_INT_BOTH_EDGE);   	

	//Hall_W = P0.2	
	GPIO_SetMode(P0, BIT2, GPIO_PMD_INPUT);
    	GPIO_EnableInt(P0, 2, GPIO_INT_BOTH_EDGE);
 //   	GPIO_EnableInt(P0, 2, GPIO_INT_RISING);

		
   	GPIO_SET_DEBOUNCE_TIME(GPIO_DBCLKSRC_HCLK, GPIO_DBCLKSEL_256);
	GPIO_ENABLE_DEBOUNCE(P0, BIT1);	
	GPIO_ENABLE_DEBOUNCE(P0, BIT1);
	GPIO_ENABLE_DEBOUNCE(P0, BIT2);


    	NVIC_EnableIRQ(GPIO_P0P1_IRQn);

////////////////////////////////////////////////////////////////////////////////////////
/*
PWM0 , PWM2 , PWM4  
*/

//	GPIO_SetMode(P2, BIT0, 	GPIO_PMD_QUASI);
//    	GPIO_SetMode(P2, BIT2, 	GPIO_PMD_QUASI);
//    	GPIO_SetMode(P2, BIT4, 	GPIO_PMD_QUASI);

//	SYS->P2_MFP &= ~(SYS_MFP_P20_Msk 	|SYS_MFP_P22_Msk	| SYS_MFP_P24_Msk );
//	SYS->P2_MFP |= 	(SYS_MFP_P20_PWM0 	|SYS_MFP_P22_PWM2	| SYS_MFP_P24_PWM4 );




//	SYS->P2_MFP &= ~(SYS_MFP_P22_Msk );
//	SYS->P2_MFP |= 	(SYS_MFP_P22_GPIO );



   // 	GPIO_SetMode(P2, BIT2, 	GPIO_PMD_OUTPUT);
//	P22 = 0;
	////////////

	SYS->P2_MFP &= ~(  SYS_MFP_P21_Msk	|	SYS_MFP_P23_Msk		|	SYS_MFP_P25_Msk);
    	SYS->P2_MFP |=      SYS_MFP_P21_GPIO| 	SYS_MFP_P23_GPIO	|	SYS_MFP_P25_GPIO;

	 
	GPIO_SetMode(P2, BIT1, 	GPIO_PMD_OUTPUT); // 3
	GPIO_SetMode(P2, BIT3, 	GPIO_PMD_OUTPUT);//  4
	GPIO_SetMode(P2, BIT5, 	GPIO_PMD_OUTPUT);  // 5
	
	P21 = 0;
	P23 = 0;
	P25 = 0;
////////////////////////////////////////////////////////////////////////////////////////
//Led


	SYS->P0_MFP &= ~(  SYS_MFP_P03_Msk	|	SYS_MFP_P04_Msk		|	SYS_MFP_P05_Msk);
    	SYS->P0_MFP |=      SYS_MFP_P03_GPIO	| 	SYS_MFP_P04_GPIO	|	SYS_MFP_P05_GPIO;


	SYS->P4_MFP &= ~(  SYS_MFP_P41_Msk);
    	SYS->P4_MFP |=      SYS_MFP_P41_GPIO;


	//P0.3 D1 Red
	GPIO_SetMode(P0, BIT3, 	GPIO_PMD_OUTPUT);
	C_D1_Red_Off;
	
	//P0.4 D1 Green
	GPIO_SetMode(P0, BIT4, 	GPIO_PMD_OUTPUT);
	C_OK_GREEN_OFF;	


	//P0.5 D2 Red
	GPIO_SetMode(P0, BIT5, 	GPIO_PMD_OUTPUT);
//	C_D2_Red_Off;
	C_BAT_RED_OFF;
	
	//P4.1 D2 Blue
	GPIO_SetMode(P4, BIT1, 	GPIO_PMD_OUTPUT);
	C_D2_Blue_Off;

////////////////////////////////////////////////////////////////////////////////////////

#if ( ScrewType == _BS_PType_4Nm5 || ScrewType == _BS_PType_3Nm)	
	//P3.3 Dir

	SYS->P3_MFP &= ~(  SYS_MFP_P33_Msk	|	SYS_MFP_P36_Msk		|	SYS_MFP_P37_Msk);
    	SYS->P3_MFP |=      SYS_MFP_P33_GPIO	| 	SYS_MFP_P36_GPIO	|	SYS_MFP_P37_GPIO;


	GPIO_SetMode(P3, BIT3, 	GPIO_PMD_INPUT);	/* CW = 0 , CCW = 1 */

	GPIO_SetMode(P3, BIT6, 	GPIO_PMD_INPUT);	//Don't Use
	GPIO_SetMode(P3, BIT7, 	GPIO_PMD_INPUT);	//Don't Use
	
#elif( ScrewType == _PTA_B120 ||ScrewType == _PTA_X50F || ScrewType == _BT_3Nm ||  ScrewType == _BT_2NmFast ||  ScrewType == _BS_1Nm2)	

	SYS->P3_MFP &= ~(  SYS_MFP_P33_Msk	|	SYS_MFP_P36_Msk		|	SYS_MFP_P37_Msk);
    	SYS->P3_MFP |=      SYS_MFP_P33_GPIO	| 	SYS_MFP_P36_GPIO	|	SYS_MFP_P37_GPIO;

	
	GPIO_SetMode(P3, BIT3, 	GPIO_PMD_OUTPUT); 	//P3.3 Start
	LIGHT_OFF;
	#if 0
   		GPIO_SetMode(P3, BIT3, GPIO_PMD_INPUT);
   		GPIO_DisableEINT1(P3, 3 );
   		NVIC_DisableIRQ(EINT1_IRQn);
    		/* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
		//    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCLKSRC_LIRC, GPIO_DBCLKSEL_4096);
		//    GPIO_DISABLE_DEBOUNCE(P3,  BIT3);
		SYS->P3_MFP &= ~(SYS_MFP_P33_Msk);
		SYS->P3_MFP |= ( SYS_MFP_P33_GPIO);
	#endif
	
	GPIO_SetMode(P3, BIT6, 	GPIO_PMD_INPUT);	//BRAKE
	GPIO_SetMode(P3, BIT7, 	GPIO_PMD_INPUT);	//REV

#endif



////////////////////////////////////////////////////////////////////////////////////////

	SYS->P3_MFP &= ~(SYS_MFP_P32_Msk);
    	SYS->P3_MFP |= (SYS_MFP_P32_INT0 );
		
	GPIO_SetMode(P3, BIT2, 	GPIO_PMD_INPUT);	//Clutch_Brake

   	GPIO_EnableEINT0(P3, 2, GPIO_INT_BOTH_EDGE);
//#if ( ScrewType != _PTS )		
   	NVIC_EnableIRQ(EINT0_IRQn);
//#endif

    	/* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
// 	GPIO_SET_DEBOUNCE_TIME(GPIO_DBCLKSRC_HCLK, GPIO_DBCLKSEL_8);
   	GPIO_SET_DEBOUNCE_TIME(GPIO_DBCLKSRC_HCLK, GPIO_DBCLKSEL_256);

 	GPIO_ENABLE_DEBOUNCE(P3, BIT2);

		
////////////////////////////////////////////////////////////////////////////////////////
//P0.6 Buzzer

	SYS->P0_MFP &= ~(SYS_MFP_P06_Msk);
    	SYS->P0_MFP |= (SYS_MFP_P06_GPIO);
		
	GPIO_SetMode(P0, BIT6, 	GPIO_PMD_OUTPUT);	
	C_Buzzer_OFF;

////////////////////////////////////////////////////////////////////////////////////////
//P0.7 Power Keep

	SYS->P0_MFP &= ~(SYS_MFP_P07_Msk);
    	SYS->P0_MFP |= (SYS_MFP_P07_GPIO);


	GPIO_SetMode(P0, BIT7, 	GPIO_PMD_OUTPUT);	

//	 if(P33 == 1)
		C_PWRKeep_ON; 	
// 	else	
//		C_PWRKeep_OFF;

	
}

void I2C_Init(void)
{

    CLK_EnableModuleClock(I2C0_MODULE);

    CLK_SetModuleClock(I2C0_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_CLKDIV_UART(1));	

    SYS_ResetModule(I2C0_RST);
}


void ADC_Init(void)
{

    CLK_EnableModuleClock(ADC_MODULE);

    /* Select UART module clock source */
//    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_PLL, CLK_CLKDIV_UART(1));

    /* ADC clock source is 22.1184MHz, set divider to 7, ADC clock is 22.1184/7 MHz */
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_ADC_S_HIRC, CLK_CLKDIV_ADC(7));



	ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_CONTINUOUS, AdcCH0|AdcCH1|AdcCH2|AdcCH3|AdcCH4|AdcCH5|AdcCH6);

	/* Power on ADC module */
	ADC_POWER_ON(ADC);


}




void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_PLL, CLK_CLKDIV_UART(1));

	
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 921600);
	 //UART_Open(UART0, 115200);
}

void PWM_Init(void)
{
 /* Enable PWM module clock */
    	CLK_EnableModuleClock(PWM01_MODULE);
    	CLK_EnableModuleClock(PWM23_MODULE);
    	CLK_EnableModuleClock(PWM45_MODULE);
    //CLK_EnableModuleClock(PWM67_MODULE);

    /* Select PWM module clock source */
    	CLK_SetModuleClock(PWM01_MODULE, CLK_CLKSEL1_PWM01_S_HIRC, 0);	
    	CLK_SetModuleClock(PWM23_MODULE, CLK_CLKSEL1_PWM23_S_HIRC, 0);
     	CLK_SetModuleClock(PWM45_MODULE,  CLK_CLKSEL2_PWM45_S_HIRC, 0);



            /* Enable Timer period Interrupt */
           // PWM_EnablePeriodInt(PWMA, PWM_CH0, PWM_PERIOD_INT_UNDERFLOW);
           // PWM_EnablePeriodInt(PWMA, PWM_CH2, PWM_PERIOD_INT_UNDERFLOW);
           // PWM_EnablePeriodInt(PWMA, PWM_CH0, PWM_PERIOD_INT_UNDERFLOW);


	


    /* Reset PWMA channel0~channel3 */
    	SYS_ResetModule(PWM03_RST);
    	SYS_ResetModule(PWM47_RST);

	PWM_EnableOutput(PWMA, PWMA_0 | PWMA_2 );
	PWM_EnableOutput(PWMB, PWMB_4);

	 #if 0
		PWM_ConfigOutputChannel(PWMA, PWM_CH0, PWM_Freq, PWM_Duty);
		PWM_ConfigOutputChannel(PWMA, PWM_CH2, PWM_Freq, PWM_Duty);	
		PWM_ConfigOutputChannel(PWMB, PWM_CH0, PWM_Freq, PWM_Duty); 
	 #else
		PWM_ConfigOutputChannel(PWMA, PWM_CH0, PWM_Freq, G_PwmCMR);
		PWM_ConfigOutputChannel(PWMA, PWM_CH2, PWM_Freq, G_PwmCMR);	
		PWM_ConfigOutputChannel(PWMB, PWM_CH0, PWM_Freq, G_PwmCMR);
	 #endif
	PWM_Start(PWMA,  PWMA_2);
	PWM_Start(PWMA,  PWMA_0);
	PWM_Start(PWMB,  PWMB_4);

//	printf("PWMA->CNR0 = %X , %d\n",PWMA->CNR0,PWMA->CNR0);
//	printf("PWMA->CNR2 = %X , %d\n",PWMA->CNR2,PWMA->CNR2);
//	printf("PWMB->CNR0 = %X , %d\n",PWMB->CNR0,PWMB->CNR0);

	printf("PMWCMR = %d / %d\n",PWMMaxCMR, PWMMinCMR);

	gu32MAXPWM = PWMMaxCMR;
	gu32MINPWM =  PWMMinCMR;

	printf("gu32MAXPWM = %d\n",gu32MAXPWM);
	printf("gu32MINPWM = %d\n",gu32MINPWM);
	
}
#if(WatchDog == ENABLE)
void WDT_Init(void)
{
    SYS_UnlockReg();

    CLK_EnableModuleClock(WDT_MODULE);
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDT_S_LIRC, NULL);
	

    /* Select WDT time-out interval to 2^14 * WDT clock then start WDT counting */
    WDT_Open(WDT_TIMEOUT_2POW12, WDT_RESET_DELAY_3CLK, TRUE,FALSE);


    /* Enable WDT interrupt function */
    WDT_EnableInt();

    /* Enable WDT NVIC */
    NVIC_EnableIRQ(WDT_IRQn);
	
//    SYS_LockReg();

}
#endif


void TIMER_Init(void)
{

#if 1
   /* Enable Timer 0~3 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);    
    CLK_EnableModuleClock(TMR1_MODULE); 
		
    /* Select Timer 0~3 module clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HIRC, NULL);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1_S_HIRC, NULL);


#endif	


#if 0
//100 us
	TIMER0->TCSR = TIMER_PERIODIC_MODE |27;// TIMER_PRESCALE_VALUE;
	TIMER0->TCMPR  =  79 ;  //   (1/22118400)*(255+1)*1728	
	TIMER_EnableInt(TIMER0);
    	NVIC_EnableIRQ(TMR0_IRQn);
    	TIMER_Start(TIMER0);


#else
/* °O±o  */
// 10 us
	TIMER0->TCSR = TIMER_PERIODIC_MODE |3;// TIMER_PRESCALE_VALUE;
//	TIMER0->TCMPR  = (28 * 2)-1 ;  //10us	//   (1/22118400)*(255+1)*1728	
	TIMER0->TCMPR  = 111 ;  		//20us
	TIMER_EnableInt(TIMER0);
    	NVIC_EnableIRQ(TMR0_IRQn);
    	TIMER_Start(TIMER0);
#endif


#if 1
 //   1ms : 86 , 20ms :1728 , 100ms : 8640	
	TIMER1->TCSR = TIMER_PERIODIC_MODE | TIMER_PRESCALE_VALUE;
	//TIMER1->TCMPR  =  43200 * 2 ;  //   (1/22118400)*(255+1)*86400
	TIMER1->TCMPR  =  86 * 5  ;  
	TIMER_EnableInt(TIMER1);
    	NVIC_EnableIRQ(TMR1_IRQn);
    	TIMER_Start(TIMER1);
#endif


		
}
