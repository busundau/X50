#ifndef  TBK_TypeDef
#define  TBK_TypeDef
#include "M051Series.h"
#include "PtsPta_Motor.h"

//#define IDByPass			TRUE


#define _BS_PType_4Nm5		0
#define _BT_4Nm5			1
#define _BT_3Nm				2
#define _BT_2Nm				3
#define _BT_2NmFast			4
#define _BS_PType_3Nm		5
#define _BS_1Nm2			6
#define _PTS					7		
#define _PTA_B120			8		
#define _PTA_L300			9		
#define _PTA_L150F			10	
#define _PTA_X50F			11	




typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

#if ( ScrewType == _PTA_B120 ||ScrewType == _PTA_X50F  )	

#define FWVer			101
#define SVN				"20250516" //jacob modify 20250516
//#define IDCHECK			
//#define MODEL			PTA_150	
//#define MODEL			PTA_80	
#endif


#define PTA_220		220
#define PTA_150		150
#define PTA_120		120
#define PTA_80		80







#define 	Poles_UVW 	1	
#define 	Poles_UWV 	2	
#define 	Poles_VUW 	3
#define 	Poles_WUV 	4	
#define 	Poles_VWU 	5
#define 	Poles_WVU 	6



#define BatteryNTCMin	110 // 1.1V
#define BatteryNTCMax	360 // 3.6V
#define OVER_BatteryNTC_MAX_DELAY	      	(500/5)    /*500ms 保護住, 因為TMR1為5ms*/




#define ForWard_Phas 	1 /* 相位 領先 */
#define Center_Phas		2 /* 相位 中立 */
#define BackWard_Phas 	3 /* 相位 落後 */

#define Pin7			_UartFunc
#define _UartFunc	0x08
#define _GPIOFunc	0x80

#define WatchDog	ENABLE


#define HALL_STATUS   (C_Hall_W() + C_Hall_V() + C_Hall_U())
#define 	C_HallU	P00
#define 	C_HallV	P01
#define 	C_HallW	P02

#if( ScrewType == _PTA_B120 ||ScrewType == _PTA_X50F  )	
/*                        尺比為 24.42
空轉	 MCU 算出來			    + 離合器 實際用轉 速 計量到	
	12.3V = 27000 rpm					1088 rpm
	10.5V = 22900 rpm					925	 rpm

*/
#define 	Turbo		ENABLE//DISABLE//ENABLE
#define 	MotorPhase	Center_Phas







/*	使用 SJT-15  軟結合 測試座
    	8成成扭力150 * 0.8 = 120  	*/
    	
#define FAST_RPM			2200+50	//B120
#define SLOW_RPM			1400		//B120
#define _CCWRPM_Value		9999
#define _RPM_Torlance		15

#define   OVER_CURRENT_PROTECT  	                  	1400	
#define   OVER_CURRENT_PROTECT_DELAY  	            				(3600/5)  /*600ms 保護住, 因為TMR1為5ms*/
#define   OVER_CURRENT_OVERLOAD                  		2300/* 約 74A		實際用示波器 電流勾表量測 約為 35A */
#define   OVER_CURRENT_OVERLOAD_DELAY             				(240/5)  /*100ms 保護住, 因為TMR1為5ms*/
#define   OVER_CURRENT_SHORT_MAX		             	3800/* 約 122A	實際用示波器 電流勾表量測 約為 60A */
#define   OVER_CURRENT_SHORT_MAX_DELAY	      					(96/5)    /*30ms 保護住, 因為TMR1為5ms*/




#define PWM_Freq	5000
#define PWMMinCMR	100
#define PWMMaxCMR	 (((PWMA->CNR0) *100)/100)



//#define C_POLES		Poles_UVW
//#define C_POLES		Poles_UWV
//#define C_POLES		Poles_VUW
//#define C_POLES		Poles_WUV
#define C_POLES		Poles_VWU
//#define C_POLES		Poles_WVU







/* 因為 起子內部 走線問題  紅 黃線 需對調 */
#define C_Hall_U() 	((C_HallU)	?(0):(	4	)) 
#define C_Hall_V() 	((C_HallV) 	?(0):(	2	)) 
#define C_Hall_W() 	((C_HallW)	?(0):(	1	)) 


///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

#define Down		1
#define Up		0
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
//#define 	StartButton		P33	
#define Brake 	((P36 )?(ON):(OFF))
#define Revers 	((P37 )?(ON):(OFF))
#define SPDEED	(P27)
/*
=============> 10.5V <===============> 11V <==============
不 可 啟 動 + 警示音                    可 啟 動 + 警示音                          正常啟動
*/

#define LowVoltage_10V5		1750 //jacob 20250516
#define LowVoltage_11V		1000 //jacob 20250516



//#define HighVoltage_18V		1800
//#define LowVoltage_17V3		1730	


#define SPEED   			P27

#define Speed1			P26
#define Speed2			P27
/*
	5 /4096 = 0.00122
	0.00122 歐姆兩顆並聯 等於 0.0005歐姆
	OP 放大76 倍率
	0.00122 /0.0005 /76  = 32105
*/
#define CURR_FACTOR_0R0005  	32105 
#define START_THRESHOLD		270
#endif

#define HI	1
#define LO	0



#define Hi	1
#define Low	0

#define ON	1
#define OFF	0

#define _OK	0x01
#define _NG	0x00

#define Delayms(x)        CLK_SysTickLongDelay((x*1000))
#define DelayS(x)          (Delayms(1000*x))

#define C_Buzzer_ON      	(P06  = 0) //DrvGPIO_ClrBit(E_GPE, 6)
#define C_Buzzer_OFF    	(P06  = 1) // DrvGPIO_SetBit(E_GPE, 6)

#define C_PtsPta_Buzzer_ON      	(P06  = 0) //DrvGPIO_ClrBit(E_GPE, 6)
#define C_PtsPta_Buzzer_OFF    	(P06  = 1) // DrvGPIO_SetBit(E_GPE, 6)



#define C_PWRKeep_ON      	(P07  = 1) //DrvGPIO_ClrBit(E_GPE, 6)
#define C_PWRKeep_OFF    	(P07  = 0) // DrvGPIO_SetBit(E_GPE, 6)

	

#if 1
//20us
#define _1Min	(3000*1000) 	//20us = 3000,000 		* 20 = 60,000,000
#define _30S		(1500*1000) 	//20us = 1500,000		* 20 = 30,000,000

#ifdef Scott_Debug
#define _24S		(250*1000) 	//20us = 1200,000		* 20 = 24,000,000
#else
#define _24S		(1200*1000) 	//20us = 1200,000		* 20 = 24,000,000
#endif



#define _10S		(  500*1000) 	//20us =  500,000		* 20 = 10,000,000
#define _5S		(  250*1000) 	//20us = 	250,000		* 20 =   5,000,000
#define _3S		(  150*1000) 	//20us =  150,000		* 20 =   3,000,000
#define _1S		(    50*1000) 	//20us =  50,000		* 20 =   1,000,000



/*
// 10us
#define _1Min	  	(6000*1000) 	//10us = 6000,000 	* 10 = 60,000,000
#define _24S		(2400*1000) 	//10us = 2400,000		* 10 = 24,000,000
#define _10S		(1000*1000) 	//10us = 1000,000		* 10 = 10,000,000
#define _5S		(500*1000) 	//10us = 500,000		* 10 =   5,000,000
#define _3S		(300*1000) 	//10us =  300,000		* 10 =   3,000,000
#define _1S		(100*1000) 	//10us =  100,000		* 10 =   1,000,000
*/


#else
// 5us
#define _1Min			(12000*1000) //5us = 12000000 * 5 = 60,000,000
#define _24S				(4800*1000) 	//5us = 4800000 * 5 = 24,000,000
#define _10S				(2000*1000) 	//5us = 2000000 * 5 = 10,000,000
#define _5S				(1000*1000) 	//5us = 1000000 * 5 =  5,000,000
#define _3S				(600*1000) 	//5us = 600000 * 5 =  3,000,000
#define _1S				(200*1000) 	//5us = 200000 * 5 =  1,000,000
#endif




//#define _1S_By20ms		(50)       //20ms =  50 *20 =  1000ms
#define _2S_By20ms		(100) 	//20ms = 100 *20 = 2000ms
#define _3S_By20ms		(150) 	//20ms = 150 *20 = 3000ms
//#define _5S_By20ms		(250) 	//20ms = 250 *20 = 5000ms
//#define _24S_By20ms		(1200) 	//20ms = 1200 *20 = 24000ms


#define _1S_By5ms		(200) 	// 200 *5 = 1000ms
#define _2S_By5ms		(400) 	// 400 *5 = 2000ms
#define _3S_By5ms		(600) 	// 600 *5 = 3000ms







#define C_BAT_RED_ON	(P05  = 0)		
#define C_BAT_RED_OFF	(P05  = 1)

#define C_D1_Red_On	(P05  = 0)		
#define C_D1_Red_Off	(P05  = 1)
////////////////////////////////////////////////////////////
//#define C_D1_Green_On	(P04  = 0)		
//#define C_D1_Green_Off	(P04  = 1)

#define C_OK_GREEN_ON	(P04  = 0)		
#define C_OK_GREEN_OFF	(P04  = 1)

////////////////////////////////////////////
//#define C_D2_Red_On	(P03  = 0)		
//#define C_D2_Red_Off	(P03  = 1)

#define C_NG_RED_ON	(P03  = 0)		
#define C_NG_RED_OFF	(P03  = 1)
////////////////////////////////////////////



#define C_D2_Blue_On	(P41  = 0)		
#define C_D2_Blue_Off	(P41  = 1)		

#define LIGHT_ON		(P33 = 1)
#define LIGHT_OFF		(P33 = 0)

#define 	ClutchBrake  	P32

  //Copy From BS Source Code--------------------------------------------------End// 

  //--------------------------------------------------------------------------End// 
  //------------------------------Temprature Check-------------------------------//	
		
		//	85度 NTC(mid) = 1.450k
		//	5V * [10k /(10k + 1.450k)] = 4.366V  (4.366V / 5) * 4096 = 3576

		//	80度 NTC(mid) = 1.667k
		//	5V * [10k /(10k + 1.667k)] = 4.285V  (4.285V / 5) * 4096 = 3510

		//	75度 NTC(mid) = 1.924k
		//	5V * [10k /(10k + 1.924k)] = 4.193V  (4.193V / 5) * 4096 = 3435

		//	70度 NTC(mid) = 2.228k
		//	5V * [10k /(10k + 2.228k)] = 4.088V  (4.088V / 5) * 4096 = 3348
		
		//  6144/ADC - 1.5 = NTC 阻值
/*      VDD
	      |
	      |
	    &&   NTC 10K
	      |
	      |----------------->ADC
	    &&    10K 
	      |
	      |
            GND
*/	
#define PCB_TEMP_85    	3576
#define PCB_TEMP_80    	3510
#define PCB_TEMP_75    	3435
#define PCB_TEMP_70    	3348

#if 0  //Motor NTC 100K

  //------------------------Motor Temprature Check-------------------------------//	

/*      VDD
	      |
	      |
	    &&   NTC 100K
	      |
	      |----------------->ADC
	    &&    1.5K 
	      |
	      |
            GND
*/	
		//	100度 NTC(mid) = 5.28k
		//	5V * [10K /(10K + 5.28K)] = 3.2722V  (3.2722V / 5) * 4096 = 2680

		//	95度 NTC(mid) = 6.155k
		//	5V * [10K /(10K + 6.155K)] = 3.0950V  (3.0950V / 5) * 4096 = 2535

		//	90度 NTC(mid) = 7.215k
		//	5V * [10K /(10K + 7.215K)] = 2.9044V  (2.9044V / 5) * 4096 = 2379
				
		//	80度 NTC(mid) = 10.081k
		//	5V * [10K /(10K + 10.081K)] = 2.4899V  (2.4899V / 5) * 4096 = 2039	

		
		//  40960/ADC - 10 = NTC阻值


#define MOTOR_TEMP_100     2680
#define MOTOR_TEMP_95      2535
#define MOTOR_TEMP_90      2379
#define MOTOR_TEMP_80      2039


#else
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
/*      VDD
	      |
	      |
	    &&   NTC 10K
	      |
	      |----------------->ADC
	    &&    10K 
	      |
	      |
            GND
*/	
		//	100度 NTC(mid) = 0.97k
		//	5V * [10K /(10K + 0.97K)] = 4.557V  (4.557V / 5) * 4096 = 3734

		//	95度 NTC(mid) = 1.10k
		//	5V * [10K /(10K + 1.10K)] = 4.504V  (4.504V / 5) * 4096 = 3690

		//	90度 NTC(mid) = 1.26k
		//	5V * [10K /(10K + 1.26K)] = 4.440V  (4.440V / 5) * 4096 = 3638
				
		//	80度 NTC(mid) = 1.66k
		//	5V * [10K /(10K + 1.66K)] = 4.288V  (4.288V / 5) * 4096 = 3513	

		
		//  40960/ADC - 10 = NTC阻值


#define MOTOR_TEMP_100     	3734
#define MOTOR_TEMP_95      	3690
#define MOTOR_TEMP_90      	3638
#define MOTOR_TEMP_80      	3513
#endif



enum
{

	eForward,          		
	eBrake,          				
	eReverse,          		
};

//#define LED_Green_Flicker() 	(G_LedFlicker_G = ON)


#define _CW         0  //Forward
#define _Brake	1    
#define _CCW       2  //Reverse


#define 	AdcCH0 	BIT0
#define 	AdcCH1 	BIT1
#define 	AdcCH2 	BIT2
#define 	AdcCH3 	BIT3
#define 	AdcCH4 	BIT4
#define 	AdcCH5 	BIT5
#define 	AdcCH6 	BIT6
#define 	AdcCH7 	BIT7



#define PWMA_0	0x01	
#define PWMA_1	0x02
#define PWMA_2	0x04
#define PWMA_3	0x08

#define PWMB_4	0x01
#define PWMB_5	0x02
#define PWMB_6	0x04
#define PWMB_7	0x08



#define _FastRPM		0xA0A0A0A0
#define _SlowRPM	0x0A0A0A0A

enum
{
	NVRM_Addr_RICO 		= 	0x01, 
	NVRM_Addr_TYPE 		= 	0x03, 
	NVRM_Addr_CHECK 		= 	0x0D, 

	NVRM_Addr_Layer 		= 	0x0E, 
	NVRM_Addr_Cell			= 	0x0F, 

	

	NVRM_Addr_Time 		= 	0x08, 
	NVRM_Addr_Times 		= 	0x04,
	
	NVRM_Addr_BatteryID	= 	0x20,

	NVRM_Addr_Year		= 	0x34,
	NVRM_Addr_Month		= 	0x35,
	NVRM_Addr_Day			= 	0x36,

	NVRM_Addr_Cell1Voltage = 	0x50,
	NVRM_Addr_Cell2Voltage = 	0x52,
	NVRM_Addr_Cell3Voltage = 	0x54,
	NVRM_Addr_Cell4Voltage = 	0x56,

};


enum //LED and Buzzer
{	
	eBehaving_OFF,
	eBehaving_LowBattery,	//    17.3 ~ 18V	
	eBehaving_OutOfBattery,	//   < 17.3V	
	eBehaving_TempOverHeat,
	eBehaving_ClutchOK_300ms_G_LED_OFF,
	eBehaving_PCBTemp_70_80,
	eBehaving_MotorTemp_90_100,
	eBehaving_MotorStall,
	eBehaving_BatteryNTCErr,
	eBehaving_CurrentOver,
	eBehaving_ConnectButIDNotPass,
	eBehaving_Idle_1S,
	eBehaving_Idle_0s5,
	eBehaving_BeOnce,	//    > 18V 
	eBehaving_BeOnce_OutofBattery,
		eBehaving_Uvw_Error, // jacob
};

enum //Motor Status
{	
	eBatteryStatus_Normal,
	eBatteryStatus_Error,

};


enum //Motor Status
{	
	eMotorStatus_Normal,
	eMotorStatus_Over100,
	eMotorStatus_Over90_100,
	eMotorStatus_Stall,
	eMotorStatus_OverCurrent,
	eMotorStatus_Stop,
};

enum //PCB Status
{	
	ePCBStatus_Normal,
	ePCBStatus_Over80,
	ePCBStatus_Over70_80
};

enum //PCB Status
{	
	eSpeed_Low,
	eSpeed_Middle,
	eSpeed_High,
};

typedef struct
{

	unsigned int  	u32_Count;   
	unsigned char  	u8_OnOFF;

	 
} Timer_TypeDef;
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
enum DataFlash
{
	eAddrSpeed_Page0    	= 0x1F000,
	eAddrxxxxx_Page1    	= 0x1F200,
	eAddrxxxxx_Page2    	= 0x1F400,
	eAddrxxxxx_Page3    	= 0x1F600,
	eAddrxxxxx_Page4    	= 0x1F800,
	eAddrxxxxx_Page5    	= 0x1FA00,
	eAddrxxxxx_Page6    	= 0x1FC00,
	eAddrCurrent_Page7		= 0x1FE00,     	
};
enum
{
	eFastMode,  // 0        		
       eSlowMode,  // 1       
       eIDError,
       eLowBatteryUnder10V5,
};
enum
{
	eClutchBrake_None = 0,
	eClutchBrake_Hi ,
	eClutchBrake_Low,	
	eClutchBrake_Scuess,	
};

typedef enum bool_enum
{
    False,
    True,
} Bool;



enum
{
	eEEPROM_Type_NONE = 0,
	eEEPROM_Type_24LC02 = 2,
	eEEPROM_Type_24LC64 = 64,
	eEEPROM_Type_MCU =      128,
	eEEPROM_Type_IICBusError =      0xE1,

};


enum
{
	eIICType_None		= 0x00,
	eIICType_Nvram		= 0xA0,
	eIICType_MCU 		= 0xC6,

};

#if(C_POLES == Poles_UVW)// 1

			 #define UH_ON() 	(SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P24_Msk)) | SYS_MFP_P24_PWM4)
			 #define UH_OFF()	((SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P24_Msk)) | SYS_MFP_P24_GPIO),(P24 = 0))
			 #define UL_ON() 		(P25 = 1)
			 #define UL_OFF() 	(P25 = 0)
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 #define VH_ON() 	(SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P20_Msk)) | SYS_MFP_P20_PWM0)
			 #define VH_OFF() 	((SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P20_Msk)) | SYS_MFP_P20_GPIO),(  P20 = 0))			 
			 #define VL_ON() 		(P21 = 1)
			 #define VL_OFF() 	(P21 = 0)
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 #define WH_ON() 	(SYS->P2_MFP  =  (SYS->P2_MFP & ~ SYS_MFP_P22_Msk) | SYS_MFP_P22_PWM2)
			 #define WH_OFF() 	((SYS->P2_MFP =  (SYS->P2_MFP & (~SYS_MFP_P22_Msk))|SYS_MFP_P22_GPIO),(P22 = 0))
			 #define WL_ON() 	(P23 = 1)
			 #define WL_OFF() 	(P23 = 0)

		 
#elif(C_POLES == Poles_UWV)// 2
			 /**********************************************************************************************************************************************************************************************/
			 #define UH_ON() 	(SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P24_Msk)) | SYS_MFP_P24_PWM4)
			 #define UH_OFF()	((SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P24_Msk)) | SYS_MFP_P24_GPIO),(P24 = 0))
			 #define UL_ON() 		(P25 = 1)
			 #define UL_OFF() 	(P25 = 0)
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 #define WH_ON() 	(SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P20_Msk)) | SYS_MFP_P20_PWM0)
			 #define WH_OFF() 	((SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P20_Msk)) | SYS_MFP_P20_GPIO),(  P20 = 0))			 
			 #define WL_ON() 	(P21 = 1)
			 #define WL_OFF() 	(P21 = 0)
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 #define VH_ON() 	(SYS->P2_MFP  =  (SYS->P2_MFP & ~ SYS_MFP_P22_Msk) | SYS_MFP_P22_PWM2)
			 #define VH_OFF() 	((SYS->P2_MFP =  (SYS->P2_MFP & (~SYS_MFP_P22_Msk))|SYS_MFP_P22_GPIO),(P22 = 0))
			 #define VL_ON() 		(P23 = 1)
			 #define VL_OFF() 	(P23 = 0)


			 /**********************************************************************************************************************************************************************************************/
#elif(C_POLES == Poles_VUW)// 3
			 /**********************************************************************************************************************************************************************************************/
			 #define VH_ON() 	(SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P24_Msk)) | SYS_MFP_P24_PWM4)
			 #define VH_OFF()	((SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P24_Msk)) | SYS_MFP_P24_GPIO),(P24 = 0))
			 #define VL_ON() 		(P25 = 1)
			 #define VL_OFF() 	(P25 = 0)
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 #define UH_ON() 	(SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P20_Msk)) | SYS_MFP_P20_PWM0)
			 #define UH_OFF() 	((SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P20_Msk)) | SYS_MFP_P20_GPIO),(  P20 = 0))			 
			 #define UL_ON() 	(P21 = 1)
			 #define UL_OFF() 	(P21 = 0)
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 #define WH_ON() 	(SYS->P2_MFP  =  (SYS->P2_MFP & ~ SYS_MFP_P22_Msk) | SYS_MFP_P22_PWM2)
			 #define WH_OFF() 	((SYS->P2_MFP =  (SYS->P2_MFP & (~SYS_MFP_P22_Msk))|SYS_MFP_P22_GPIO),(P22 = 0))
			 #define WL_ON() 		(P23 = 1)
			 #define WL_OFF() 	(P23 = 0)


		 
			 /**********************************************************************************************************************************************************************************************/
#elif(C_POLES == Poles_WUV)// 4

			 #define WH_ON() 	(SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P24_Msk)) | SYS_MFP_P24_PWM4)
			 #define WH_OFF()	((SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P24_Msk)) | SYS_MFP_P24_GPIO),(P24 = 0))
			 #define WL_ON() 	(P25 = 1)
			 #define WL_OFF() 	(P25 = 0)
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 #define UH_ON() 	(SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P20_Msk)) | SYS_MFP_P20_PWM0)
			 #define UH_OFF() 	((SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P20_Msk)) | SYS_MFP_P20_GPIO),(  P20 = 0))			 
			 #define UL_ON() 		(P21 = 1)
			 #define UL_OFF() 	(P21 = 0)
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 #define VH_ON() 	(SYS->P2_MFP  =  (SYS->P2_MFP & ~ SYS_MFP_P22_Msk) | SYS_MFP_P22_PWM2)
			 #define VH_OFF() 	((SYS->P2_MFP =  (SYS->P2_MFP & (~SYS_MFP_P22_Msk))|SYS_MFP_P22_GPIO),(P22 = 0))
			 #define VL_ON() 		(P23 = 1)
			 #define VL_OFF() 	(P23 = 0)
			 
#elif(C_POLES == Poles_VWU)// 5

			 #define VH_ON() 	(SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P24_Msk)) | SYS_MFP_P24_PWM4)
			 #define VH_OFF()	((SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P24_Msk)) | SYS_MFP_P24_GPIO),(P24 = 0))
			 #define VL_ON() 		(P25 = 1)
			 #define VL_OFF() 	(P25 = 0)
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 #define WH_ON() 	(SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P20_Msk)) | SYS_MFP_P20_PWM0)
			 #define WH_OFF() 	((SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P20_Msk)) | SYS_MFP_P20_GPIO),(  P20 = 0))			 
			 #define WL_ON() 	(P21 = 1)
			 #define WL_OFF() 	(P21 = 0)
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 #define UH_ON() 	(SYS->P2_MFP  =  (SYS->P2_MFP & ~ SYS_MFP_P22_Msk) | SYS_MFP_P22_PWM2)
			 #define UH_OFF() 	((SYS->P2_MFP =  (SYS->P2_MFP & (~SYS_MFP_P22_Msk))|SYS_MFP_P22_GPIO),(P22 = 0))
			 #define UL_ON() 		(P23 = 1)
			 #define UL_OFF() 	(P23 = 0)	
#elif(C_POLES == Poles_WVU) //6

			 /**********************************************************************************************************************************************************************************************/
			 #define WH_ON() 	(SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P24_Msk)) | SYS_MFP_P24_PWM4)
			 #define WH_OFF()	((SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P24_Msk)) | SYS_MFP_P24_GPIO),(P24 = 0))
			 #define WL_ON() 	(P25 = 1)
			 #define WL_OFF() 	(P25 = 0)
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 #define VH_ON() 	(SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P20_Msk)) | SYS_MFP_P20_PWM0)
			 #define VH_OFF() 	((SYS->P2_MFP = (SYS->P2_MFP & (~SYS_MFP_P20_Msk)) | SYS_MFP_P20_GPIO),(  P20 = 0))			 
			 #define VL_ON() 		(P21 = 1)
			 #define VL_OFF() 	(P21 = 0)
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 /////////////////////////////////////////////////////////////////////////////////////////////////
			 #define UH_ON() 	(SYS->P2_MFP  =  (SYS->P2_MFP & ~ SYS_MFP_P22_Msk) | SYS_MFP_P22_PWM2)
			 #define UH_OFF() 	((SYS->P2_MFP =  (SYS->P2_MFP & (~SYS_MFP_P22_Msk))|SYS_MFP_P22_GPIO),(P22 = 0))
			 #define UL_ON() 		(P23 = 1)
			 #define UL_OFF() 	(P23 = 0)
			 /**********************************************************************************************************************************************************************************************/


#endif
#endif 
