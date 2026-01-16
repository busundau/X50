#ifndef TBK_Pid
#define TBK_Pid

#include "stdint.h"
void default_init(void);
void motor_algorithm(void);
void parm_init(void);
void uvw_check_function(void);
void Exponential_PWM_Ramp_CCW(void);
void Exponential_PWM_Ramp_CW(void);
int32_t SpdPIDCalc_HI(float NextPoint);
int32_t SpdPIDCalc_LO(float NextPoint);
void PID_ParamInit_HI_CCW(void);
void PID_ParamInit_HI_CW(void);
void PID_ParamInit_LO_CCW(void);
void PID_ParamInit_LO_CW(void);
void detect_enter_dis_speed(uint16_t rpm, uint16_t current);
void detect_enter_boot_speed(uint16_t rpm, uint16_t current);
extern unsigned char 	G_BatteryStatus;
extern uint8_t			gu8StartSpeed;
#endif
