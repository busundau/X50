#ifndef  TBK_Global
#define  TBK_Global

#include "PtsPta_TypeDef.h"

extern Timer_TypeDef TMR_Double_Enalbe;
extern Timer_TypeDef TMR_ID_OK;
extern Timer_TypeDef TMR_CalcRPM;
extern Timer_TypeDef TMR_PWROff_24S;
extern Timer_TypeDef TMR_PWROff_3S;
extern Timer_TypeDef TMR_Stop_10S;
extern Timer_TypeDef TMR_MotorRuning_1Min;
extern Timer_TypeDef TMR_PCBTempOverHeat_3S;
extern Timer_TypeDef TMR_MotorTempOverHeat_3S;
extern Timer_TypeDef TMR_MotorStall;
extern Timer_TypeDef TMR_ReStart300ms;

extern unsigned char 	G_StartupDirectin;
extern unsigned char 	G_Direction;
extern unsigned char 	G_StartOn;
extern unsigned int 	G_PwmCMR;

extern unsigned int 	G_ScrewRPM;
extern unsigned int 	G_MotorRPM;
extern unsigned int 	G_MotorSpeed;
extern unsigned char 	G_MotorStatus;
extern unsigned char 	G_PCBStatus;
extern unsigned char 	G_BatteryStatus;

extern unsigned int 	G_NTC_Motor;
extern unsigned int 	G_NTC_PCB;
extern unsigned int 	G_NTC_Battery;
extern unsigned int 	G_Current;
extern u32			gu32MaxCurrent;
extern u32			gu32Current;
extern u8				gu8StartSpeed;
extern u8				gu8BrakeStop;
extern unsigned int 	G_BrakeCurrent;
extern unsigned int	G_Voltage_Battery;
extern unsigned char   	G_Error_EEPROM;
extern u32			gu32MAXPWM;
extern u32			gu32MINPWM;
extern u32			gu32FastRpmQuarter;
extern u32			gu32SlowRpmQuarter;
extern u32			gu32SlowRpmHalf;

extern unsigned char 	G_ScrewLock;

extern unsigned char  	G_LedFlicker_B;
extern unsigned char  	G_Status_Item;
//extern unsigned char  	G_ClutchSucess;
extern unsigned char  	G_ClutchBrakFlow;
extern unsigned char 	G_BuzzerLED;

extern unsigned int 	G_StartInAdc;

extern unsigned int 	G_Count;
extern unsigned int  	G_ID_Status;
extern unsigned char 	G_ReStar;
extern unsigned char 	G_Flg_ButtonUp;
extern unsigned char 	G_Flg_ButtonDown;
extern unsigned int 	StartButton;
extern unsigned char  	G_Speed;
extern volatile u8	       	gu8TurboCnt;

extern Bool    		G_Battery10V5_11V;
extern Bool     	G_IsFirstBtnDwn;
extern Bool     	G_3STimeOut;


#define Start_TMR_CalcRPM()  	(TMR_CalcRPM.u8_OnOFF= ON)
#define Stop_TMR_ICalcRPM()  	(TMR_CalcRPM.u8_OnOFF= OFF)
#define Get_TMR_CalcRPM()   		(TMR_CalcRPM.u32_Count)
#define Reset_TMR_CalcRPM()   	(TMR_CalcRPM.u32_Count = 0)

#define Start_TMR_ID_OK()  		(TMR_ID_OK.u8_OnOFF= ON)
#define Stop_TMR_ID_OK()  		(TMR_ID_OK.u8_OnOFF= OFF)
#define Get_TMR_ID_OK()   		(TMR_ID_OK.u32_Count)
#define Reset_TMR_ID_OK()   		(TMR_ID_OK.u32_Count = 0)



#define Start_TMR_Double_Enable()  		(TMR_Double_Enalbe.u8_OnOFF= ON)
#define Stop_TMR_Double_Enable()  		(TMR_Double_Enalbe.u8_OnOFF= OFF)
#define Get_TMR_Double_Enable()   		(TMR_Double_Enalbe.u32_Count)
#define Reset_TMR_Double_Enable()   		(TMR_Double_Enalbe.u32_Count = 0)

#define Start_TMR_PWROff_24S()  		(TMR_PWROff_24S.u8_OnOFF= ON)
#define Stop_TMR_PWROff_24S()  		(TMR_PWROff_24S.u8_OnOFF= OFF)
#define Get_TMR_PWROff_24S()   		(TMR_PWROff_24S.u32_Count)
#define Reset_TMR_PWROff_24S()   	(TMR_PWROff_24S.u32_Count = 0)

#define Start_TMR_Stop_10S()  		(TMR_Stop_10S.u8_OnOFF= ON)
#define Stop_TMR_Stop_10S()  		(TMR_Stop_10S.u8_OnOFF= OFF)
#define Get_TMR_Stop_10S()   			(TMR_Stop_10S.u32_Count)
#define Reset_TMR_Stop_10S()	   	(TMR_Stop_10S.u32_Count = 0)


#define Start_TMR_PWROff_3S()  		(TMR_PWROff_3S.u8_OnOFF= ON)
#define Stop_TMR_PWROff_3S()  		(TMR_PWROff_3S.u8_OnOFF= OFF)
#define Get_TMR_PWROff_3S()   		(TMR_PWROff_3S.u32_Count)
#define Reset_TMR_PWROff_3S()   		(TMR_PWROff_3S.u32_Count = 0)

#define Start_TMR_MotorRuning_1Min()  	(TMR_MotorRuning_1Min.u8_OnOFF= ON)
#define Stop_TMR_MotorRuning_1Min()  	(TMR_MotorRuning_1Min.u8_OnOFF= OFF)
#define Get_TMR_MotorRuning_1Min()   	(TMR_MotorRuning_1Min.u32_Count)
#define Reset_TMR_MotorRuning_1Min()   	(TMR_MotorRuning_1Min.u32_Count = 0)



#define Start_TMR_PCBTempOverHeat_3S()  	(TMR_PCBTempOverHeat_3S.u8_OnOFF= ON)
#define Stop_TMR_PCBTempOverHeat_3S()  	(TMR_PCBTempOverHeat_3S.u8_OnOFF= OFF)
#define Get_TMR_PCBTempOverHeat_3S()  	 	(TMR_PCBTempOverHeat_3S.u32_Count)
#define Reset_TMR_PCBTempOverHeat_3S()   	(TMR_PCBTempOverHeat_3S.u32_Count = 0)

#define Start_TMR_MotorTempOverHeat_3S()  	(TMR_MotorTempOverHeat_3S.u8_OnOFF= ON)
#define Stop_TMR_MotorTempOverHeat_3S()  	(TMR_MotorTempOverHeat_3S.u8_OnOFF= OFF)
#define Get_TMR_MotorTempOverHeat_3S()  	 	(TMR_MotorTempOverHeat_3S.u32_Count)
#define Reset_TMR_MotorTempOverHeat_3S()   	(TMR_MotorTempOverHeat_3S.u32_Count = 0)

#define Start_TMR_MotorStall()	(TMR_MotorStall.u8_OnOFF= ON)
#define Stop_TMR_MotorStall()  	(TMR_MotorStall.u8_OnOFF= OFF)
#define Get_TMR_MotorStall() 		(TMR_MotorStall.u32_Count)
#define Reset_TMR_MotorStall()   	(TMR_MotorStall.u32_Count = 0)


#define Start_TMR_ReStart300ms()		(TMR_ReStart300ms.u8_OnOFF= ON)
#define Stop_TMR_ReStart300ms()  	(TMR_ReStart300ms.u8_OnOFF= OFF)
#define Get_TMR_ReStart300ms() 		(TMR_ReStart300ms.u32_Count)
#define Reset_TMR_ReStart300ms()   	(TMR_ReStart300ms.u32_Count = 0)
#endif 
