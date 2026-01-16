#ifndef  TBK_Value
#define  TBK_Value

#include "PtsPta_TypeDef.h"

unsigned int 	G_Count;
unsigned int  	G_ID_Status;




Timer_TypeDef TMR_MotorTempOverHeat_3S;
Timer_TypeDef TMR_PCBTempOverHeat_3S;
Timer_TypeDef TMR_PWROff_3S;
Timer_TypeDef TMR_PWROff_24S;
Timer_TypeDef TMR_Stop_10S;
Timer_TypeDef TMR_MotorRuning_1Min;
Timer_TypeDef TMR_ID_OK;
Timer_TypeDef TMR_CalcRPM;
Timer_TypeDef TMR_MotorStall;
Timer_TypeDef TMR_ReStart300ms;
Timer_TypeDef TMR_Double_Enalbe;

unsigned char 	G_StartupDirectin;
unsigned char 	G_Direction;
unsigned char 	G_StartOn;
unsigned int 	G_PwmCMR;
unsigned int 	G_ScrewRPM;
unsigned int 	G_MotorRPM;
unsigned int 	G_MotorSpeed;
unsigned char 	G_MotorStatus;
unsigned char 	G_PCBStatus;
unsigned char 	G_BatteryStatus;

unsigned char  	G_Status_Item;
unsigned char  	G_LedFlicker_B;



unsigned char 	G_BuzzerLED;
unsigned char 	G_ScrewLock;

Bool     	G_Battery10V5_11V;
Bool     	G_IsFirstBtnDwn;
Bool     	G_3STimeOut;

unsigned int 	StartButton;
unsigned int 	G_NTC_Motor;
unsigned int 	G_NTC_PCB;
unsigned int 	G_NTC_Battery;

unsigned int 	G_StartInAdc;

unsigned int 	G_Current;
u32			gu32MaxCurrent;
u32			gu32Current;
u8			gu8StartSpeed;
u8			gu8BrakeStop;
u32			gu32MAXPWM;
u32			gu32MINPWM;
u32			gu32FastRpmQuarter;
u32			gu32SlowRpmQuarter;
u32			gu32SlowRpmHalf;
volatile u8	gu8TurboCnt;

unsigned int 	G_BrakeCurrent;

unsigned int 	G_Voltage_Battery;
//unsigned char  	G_ClutchSucess;
unsigned char  	G_ClutchBrakFlow;
unsigned char  	G_Speed;


unsigned char  	G_Error_EEPROM;

unsigned char 		G_ReStar;
unsigned char 		G_Flg_ButtonUp;
unsigned char 		G_Flg_ButtonDown;
unsigned char       T_1ms,T_1ms222;


#endif 
