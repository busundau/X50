#include <stdio.h>
#include "M051Series.h"
#include "string.h"
#include "PtsPta_TypeDef.h"
#include "PtsPta_Global.h"
#include "pid.h"

uint8_t CW_HI_BootSpeed_Flag = 0;
uint8_t CW_LO_BootSpeed_Flag = 0;
uint8_t CCW_LO_BootSpeed_Flag = 0;
uint8_t CCW_HI_BootSpeed_Flag = 0;
int32_t SpdPIDCalc_HI(float NextPoint);
int32_t SpdPIDCalc_LO(float NextPoint);

extern unsigned char  T_1ms222;
void Motor_Control_cw_Task();
void Motor_Control_ccw_Task();
#define OVERCURRENT_LIMIT 1400   /// 1200	 // 單位：mA
#define OVERCURRENT_CLEAR 200    // 清除門檻
#define OVERCURRENT_HOLD_LOOP 20 // 每 10ms 呼叫一次，25 次約 250ms
/*
#define SPD_P_DATA_LO 0.080f // 低轉速 p曲線值
#define SPD_I_DATA_LO 0.05f  // 低轉速 i曲線值
#define SPD_D_DATA_LO 0.0f   // 低轉速 d曲線值
#define TARGET_SPEED_LO 1425 // 設定630==>實際650 // 目標低轉速
#define TARGET_SPEED_HI 2200 // 350 // 目標高轉速
#define SPD_P_DATA_HI 0.08f  // 0.57f  // 高轉速 p曲線值
#define SPD_I_DATA_HI 0.07f  // 0.029f // 高轉速 i曲線值
#define SPD_D_DATA_HI 0.0f   // 高轉速 d曲線值
*/
#define close_cW_Flag 0


#define Q16(x)   ((int32_t)((x) * 65536.0f))

#define SPD_P_DATA_LO_Q16   Q16(0.080f )
#define SPD_I_DATA_LO_Q16   Q16(0.05f)
#define SPD_D_DATA_LO_Q16   Q16(0.0f)

#define TARGET_SPEED_LO_Q16 Q16(1430.0f)


#define SPD_P_DATA_HI_Q16   Q16(0.18)//0.12==>0.18(low 1000)==>0.3f
#define SPD_I_DATA_HI_Q16   Q16(0.08f)
#define SPD_D_DATA_HI_Q16   Q16(0.00f)

#define TARGET_SPEED_HI_Q16 Q16(2040.0f)
/*typedef struct
{
    __IO int32_t SetPoint;
    __IO float SumError;
    __IO float Proportion;
    __IO float Integral;
    __IO float Derivative;
    __IO int LastError;
    __IO int PrevError;
} PID_TypeDef; // pid結構
*/
typedef struct {
    int32_t SetPoint_q16;   // Q16.16
    int32_t Kp_q16;
    int32_t Ki_q16;
    int32_t Kd_q16;

    int32_t LastError_q16;
    int32_t PrevError_q16;
} PID_Q16;
float k = 0;

extern unsigned int G_Current;
uint8_t enter_boot_speed_mode = 0;

PID_Q16 sPID_LO_CW, sPID_LO_CCW, sPID_HI_CW, sPID_HI_CCW;

uint16_t prev_rpm_cw = 0;
uint16_t prev_current_cw = 0;
uint16_t prev_pwm_cw = 0;
uint16_t prev_rpm_ccw = 0;
uint16_t prev_current_ccw = 0;
uint16_t rpm_now = 0, rpm_prev = 0;
uint16_t current_now = 0, current_prev = 0;

extern unsigned long int LO_count_300ms;
signed int jump_pwm = 0;
extern unsigned long int HI_count_300ms;
signed int jump_pwm222 = 0;
unsigned int bat = 0;
extern unsigned int L_TargetCount;
float iError_HI_CW = 0, iError_HI_CCW = 0, iError_LO_CW = 0, iError_LO_CCW = 0, iIncpid_HI_CW = 0, iIncpid_HI_CCW = 0, iIncpid_LO_CW = 0, iIncpid_LO_CCW = 0;
extern unsigned int G_StallCnt;
extern unsigned char errorflag;

extern int Enter_HI_Loop;
extern int Enter_LO_Loop;
int temppwm = 0, temppwm222 = 0;
int LO_Loop = 0;

unsigned long int HI_count_300ms;
unsigned long int LO_count_300ms;

extern unsigned int G_StallCnt;
extern unsigned char errorflag;

int Enter_HI_Loop = 0;
int Enter_LO_Loop = 0;
int HI_Loop = 0;
extern int temppwm, temppwm222;
extern int LO_Loop;
extern int HI_Loop;
unsigned long int HI_count_300ms;
unsigned long int LO_count_300ms;
//extern volatile uint32_t t;
//extern volatile uint32_t t2;
#define ROUND_TO_INT32(x) ((int32_t)(x) + 0.5f) >= (x) ? ((int32_t)(x)) : ((uint32_t)(x) + 1)

uint32_t oc_timestamp = 0;
//extern uint32_t t3;
void delayms(uint32_t ms)
{
    CLK_SysTickDelay(ms * 1000);
}

// ======================
// 參數設定區
// ======================
#define RPM_AVG_SIZE 1 // 移動平均樣本數
#define CUR_AVG_SIZE 1
#define LOAD_CONFIRM_CNT 1   // 連續成立次數
#define RPM_DROP_TH_HI 20.0f // rpm 下降閾值 //high7

#define RPM_DROP_TH_LO 20.0f // rpm 下降閾值 //high7

// ======================
// 全域變數
// ======================
float rpm_buf[RPM_AVG_SIZE] = {0};
float cur_buf[CUR_AVG_SIZE] = {0};

float rpm_avg = 0, cur_avg = 0;
float rpm_prev_avg = 0, cur_prev_avg = 0;

// ======================
// 移動平均函式
// ======================
float moving_average(float *buf, uint8_t *idx, uint8_t size, float new_val)
{
    uint8_t i;
    float sum;
    buf[*idx] = new_val;
    (*idx)++;
    if (*idx >= size)
        *idx = 0;

    sum = 0;
    for (i = 0; i < size; i++)
        sum += buf[i];

    return sum / size;
}
#include <stdbool.h>
#define AVG_WINDOW 3
float current_buf[AVG_WINDOW] = {0};
uint8_t idx = 0;
float avg_current = 0;

void UpdateCurrent(float current_sample)
{
    int i;
    float sum = 0;
    current_buf[idx] = current_sample;
    idx = (idx + 1) % AVG_WINDOW;
    sum = 0;
    for (i = 0; i < AVG_WINDOW; i++)
        sum += current_buf[i];
    avg_current = sum / AVG_WINDOW;
}

#define SPEED_AVG_WINDOW 3 // 移動平均窗長，可調整
float speed_buf[SPEED_AVG_WINDOW] = {0};
uint8_t speed_idx = 0;
float avg_speed = 0;

#define SPEED_AVG_WINDOW2 3 // 移動平均窗長，可調整
float speed_buf2[SPEED_AVG_WINDOW2] = {0};
uint8_t speed_idx2 = 0;
float avg_speed2 = 0;

void UpdateSpeed_CW(float speed_sample)
{
    float sum = 0;
    int i;
    speed_buf[speed_idx] = speed_sample;
    speed_idx = (speed_idx + 1) % SPEED_AVG_WINDOW;

    sum = 0;
    for (i = 0; i < SPEED_AVG_WINDOW; i++)
        sum += speed_buf[i];
    avg_speed = sum / SPEED_AVG_WINDOW;
}

void UpdateSpeed_CCW(float speed_sample)
{
    float sum = 0;
    int i;
    speed_buf2[speed_idx2] = speed_sample;
    speed_idx2 = (speed_idx2 + 1) % SPEED_AVG_WINDOW2;

    sum = 0;
    for (i = 0; i < SPEED_AVG_WINDOW2; i++)
        sum += speed_buf2[i];
    avg_speed2 = sum / SPEED_AVG_WINDOW2;
}
////////////////////////////////////////////////////低速部分

#include "M051Series.h"
#include <stdint.h>
#include <stdbool.h>

uint32_t varcount = 0;
unsigned int Temp_var[3][200] = {1};
/*
void PID_ParamInit_LO_CW(void) // 正轉的pid的低轉速初始化
{
    sPID_LO_CW.LastError = 0;
    sPID_LO_CW.PrevError = 0;

    sPID_LO_CW.Proportion = SPD_P_DATA_LO;
    sPID_LO_CW.Integral = SPD_I_DATA_LO;
    sPID_LO_CW.Derivative = SPD_D_DATA_LO;
    sPID_LO_CW.SetPoint = TARGET_SPEED_LO; // 設定目標轉速
}
*/
void PID_ParamInit_LO_CW(void)   // 正轉低轉速 PID 初始化（2 nm 用）
{
    // error history reset
    sPID_LO_CW.LastError_q16 = 0;
    sPID_LO_CW.PrevError_q16 = 0;

    // PID coefficients (Q16.16)
    sPID_LO_CW.Kp_q16 = SPD_P_DATA_LO_Q16;
    sPID_LO_CW.Ki_q16 = SPD_I_DATA_LO_Q16;
    sPID_LO_CW.Kd_q16 = SPD_D_DATA_LO_Q16;

    // target speed (Q16.16)
    sPID_LO_CW.SetPoint_q16 = TARGET_SPEED_LO_Q16;
}
/*
void PID_ParamInit_LO_CCW(void) // 逆轉的低轉速pid參數
{
    sPID_LO_CCW.LastError = 0;
    sPID_LO_CCW.PrevError = 0;
    
    sPID_LO_CCW.Proportion = SPD_P_DATA_LO;
    sPID_LO_CCW.Integral = SPD_I_DATA_LO;
    sPID_LO_CCW.Derivative = SPD_D_DATA_LO;
    sPID_LO_CCW.SetPoint = TARGET_SPEED_LO; // 設定目標轉速
}
*/
void PID_ParamInit_LO_CCW(void)   // 逆轉低轉速 PID 初始化（2 nm 用）
{
    // Reset error history
    sPID_LO_CCW.LastError_q16 = 0;
    sPID_LO_CCW.PrevError_q16 = 0;

    // PID coefficients (Q16.16 fixed-point)
    sPID_LO_CCW.Kp_q16 = SPD_P_DATA_LO_Q16;
    sPID_LO_CCW.Ki_q16 = SPD_I_DATA_LO_Q16;
    sPID_LO_CCW.Kd_q16 = SPD_D_DATA_LO_Q16;

    // Target speed (Q16.16)
    sPID_LO_CCW.SetPoint_q16 = TARGET_SPEED_LO_Q16;
}
/*
void PID_ParamInit_HI_CW(void) // 設定正轉高轉速的pid參數
{
    sPID_HI_CW.LastError = 0;
    sPID_HI_CW.PrevError = 0;
   
    sPID_HI_CW.Proportion = SPD_P_DATA_HI;
    sPID_HI_CW.Integral = SPD_I_DATA_HI;
    sPID_HI_CW.Derivative = SPD_D_DATA_HI;
    sPID_HI_CW.SetPoint = TARGET_SPEED_HI; // 設定目標轉速
}
*/
void PID_ParamInit_HI_CW(void)   // 正轉高轉速 PID 初始化（2 nm 用）
{
    // Reset error history
    sPID_HI_CW.LastError_q16 = 0;
    sPID_HI_CW.PrevError_q16 = 0;

    // PID coefficients (Q16.16 fixed-point)
    sPID_HI_CW.Kp_q16 = SPD_P_DATA_HI_Q16;
    sPID_HI_CW.Ki_q16 = SPD_I_DATA_HI_Q16;
    sPID_HI_CW.Kd_q16 = SPD_D_DATA_HI_Q16;

    // Target speed (Q16.16)
    sPID_HI_CW.SetPoint_q16 = TARGET_SPEED_HI_Q16;
}
/*
void PID_ParamInit_HI_CCW(void) // 設定高轉逆轉的pid參數初始化
{
    sPID_HI_CCW.LastError = 0;
    sPID_HI_CCW.PrevError = 0;
  
    sPID_HI_CCW.Proportion = SPD_P_DATA_HI;
    sPID_HI_CCW.Integral = SPD_I_DATA_HI;
    sPID_HI_CCW.Derivative = SPD_D_DATA_HI;
    sPID_HI_CCW.SetPoint = TARGET_SPEED_HI; // 設定目標轉速
}
*/

void PID_ParamInit_HI_CCW(void)   // 高轉速逆轉 PID 初始化（2 nm 用）
{
    // Reset error history
    sPID_HI_CCW.LastError_q16 = 0;
    sPID_HI_CCW.PrevError_q16 = 0;

    // PID coefficients (Q16.16 fixed-point)
    sPID_HI_CCW.Kp_q16 = SPD_P_DATA_HI_Q16;
    sPID_HI_CCW.Ki_q16 = SPD_I_DATA_HI_Q16;
    sPID_HI_CCW.Kd_q16 = SPD_D_DATA_HI_Q16;   // Cmk 時建議 = 0

    // Target speed (Q16.16)
    sPID_HI_CCW.SetPoint_q16 = TARGET_SPEED_HI_Q16;
}

/*
int32_t SpdPIDCalc_LO_CW(float NextPoint) // pid低轉速正轉運行函數
{

    iError_LO_CW = (float)sPID_LO_CW.SetPoint - NextPoint;
    if ((iError_LO_CW < 30.0f) && (iError_LO_CW > -30.0f))
        iError_LO_CW = 0.0f;

    iIncpid_LO_CW = (sPID_LO_CW.Proportion * iError_LO_CW) - (sPID_LO_CW.Integral * sPID_LO_CW.LastError) + (sPID_LO_CW.Derivative * sPID_LO_CW.PrevError);

    sPID_LO_CW.PrevError = sPID_LO_CW.LastError;
    sPID_LO_CW.LastError = iError_LO_CW;
   
    return (ROUND_TO_INT32(iIncpid_LO_CW));
}
*/
int32_t SpdPIDCalc_LO_CW_Q16(int32_t NextPoint_q16)
{
    int32_t error_q16;
    int64_t incpid_q32;

    // error = setpoint - feedback
    error_q16 = sPID_LO_CW.SetPoint_q16 - NextPoint_q16;

    // deadband: ±30 (Q16.16)
    if (error_q16 < (30 << 16) && error_q16 > -(30 << 16))
        error_q16 = 0;

    // Incremental PID (use int64 to avoid overflow)
    incpid_q32 =
        (int64_t)sPID_LO_CW.Kp_q16 * error_q16
      - (int64_t)sPID_LO_CW.Ki_q16 * sPID_LO_CW.LastError_q16
      + (int64_t)sPID_LO_CW.Kd_q16 * sPID_LO_CW.PrevError_q16;

    // Back to Q16.16
    incpid_q32 >>= 16;

    // update error history
    sPID_LO_CW.PrevError_q16 = sPID_LO_CW.LastError_q16;
    sPID_LO_CW.LastError_q16 = error_q16;

    // 只做一次 truncation（不 round）
    return (int32_t)(incpid_q32>>16);
}
/*
int32_t SpdPIDCalc_LO_CCW(float NextPoint) // pid低轉速逆轉運行函數
{

    iError_LO_CCW = (float)sPID_LO_CCW.SetPoint - NextPoint;
    if ((iError_LO_CCW < 30.0f) && (iError_LO_CCW > -30.0f))

        iError_LO_CCW = 0.0f;

    iIncpid_LO_CCW = (sPID_LO_CCW.Proportion * iError_LO_CCW) - (sPID_LO_CCW.Integral * sPID_LO_CCW.LastError) + (sPID_LO_CCW.Derivative * sPID_LO_CCW.PrevError);

    sPID_LO_CCW.PrevError = sPID_LO_CCW.LastError;
    sPID_LO_CCW.LastError = iError_LO_CCW;
    // printf("ROUND_TO_INT32(iIncpid)=%d",ROUND_TO_INT32(iIncpid));
    // t++;
    return (ROUND_TO_INT32(iIncpid_LO_CCW));
}
*/
int32_t SpdPIDCalc_LO_CCW_Q16(int32_t NextPoint_q16)
{
    int32_t error_q16;
    int64_t incpid_q32;

    // error = setpoint - feedback
    error_q16 = sPID_LO_CCW.SetPoint_q16 - NextPoint_q16;

    // deadband: ±30 (Q16.16)
    if (error_q16 < (30 << 16) && error_q16 > -(30 << 16))
        error_q16 = 0;

    // Incremental PID (Q16.16 math, use int64 to avoid overflow)
    incpid_q32 =
        (int64_t)sPID_LO_CCW.Kp_q16 * error_q16
      - (int64_t)sPID_LO_CCW.Ki_q16 * sPID_LO_CCW.LastError_q16
      + (int64_t)sPID_LO_CCW.Kd_q16 * sPID_LO_CCW.PrevError_q16;

    // Back to Q16.16
    incpid_q32 >>= 16;

    // update error history
    sPID_LO_CCW.PrevError_q16 = sPID_LO_CCW.LastError_q16;
    sPID_LO_CCW.LastError_q16 = error_q16;

    // 最後只做一次 truncation（不 round）
     return (int32_t)(incpid_q32>>16);
}
/*
int32_t SpdPIDCalc_HI_CW(float NextPoint) // pid高轉速正轉運行函數
{
    iError_HI_CW = (float)sPID_HI_CW.SetPoint - NextPoint;

    if ((iError_HI_CW < 30.0f) && (iError_HI_CW > -30.0f))
        iError_HI_CW = 0.0f;

    iIncpid_HI_CW = (sPID_HI_CW.Proportion * iError_HI_CW) - (sPID_HI_CW.Integral * sPID_HI_CW.LastError) + (sPID_HI_CW.Derivative * sPID_HI_CW.PrevError);

    sPID_HI_CW.PrevError = sPID_HI_CW.LastError; //
    sPID_HI_CW.LastError = iError_HI_CW;

    //
    // printf("ROUND_TO_INT32(iIncpid)=%d",ROUND_TO_INT32(iIncpid));
    return (ROUND_TO_INT32(iIncpid_HI_CW)); //
}
*/
int32_t SpdPIDCalc_HI_CW_Q16(int32_t NextPoint_q16)
{
    int32_t error_q16;
    int64_t incpid_q32;

    // error = setpoint - feedback
    error_q16 = sPID_HI_CW.SetPoint_q16 - NextPoint_q16;

    // deadband: ±30 → 30 << 16
    if (error_q16 < (30 << 16) && error_q16 > -(30 << 16))
        error_q16 = 0;

    // Incremental PID (Q16.16 math, use int64 to avoid overflow)
    incpid_q32 =
        (int64_t)sPID_HI_CW.Kp_q16 * error_q16
      - (int64_t)sPID_HI_CW.Ki_q16 * sPID_HI_CW.LastError_q16
      + (int64_t)sPID_HI_CW.Kd_q16 * sPID_HI_CW.PrevError_q16;

    // shift back to Q16.16
    incpid_q32 >>= 16;

    // update error history
    sPID_HI_CW.PrevError_q16 = sPID_HI_CW.LastError_q16;
    sPID_HI_CW.LastError_q16 = error_q16;
   
    // 最後只做一次 truncation（不 round）
    return (int32_t)(incpid_q32>>16);
}
/*
int32_t SpdPIDCalc_HI_CCW(float NextPoint) // pid高轉速逆轉運行函數
{
    iError_HI_CCW = (float)sPID_HI_CCW.SetPoint - NextPoint;
    if ((iError_HI_CCW < 30.0f) && (iError_HI_CCW > -30.0f))
        iError_HI_CCW = 0.0f;

    iIncpid_HI_CCW = (sPID_HI_CCW.Proportion * iError_HI_CCW) - (sPID_HI_CCW.Integral * sPID_HI_CCW.LastError) + (sPID_HI_CCW.Derivative * sPID_HI_CCW.PrevError);

    sPID_HI_CCW.PrevError = sPID_HI_CCW.LastError;
    sPID_HI_CCW.LastError = iError_HI_CCW;

    //
    // printf("ROUND_TO_INT32(iIncpid)=%d",ROUND_TO_INT32(iIncpid));
    return (ROUND_TO_INT32(iIncpid_HI_CCW));
}
*/
int32_t SpdPIDCalc_HI_CCW_Q16(int32_t NextPoint_q16)
{
    int32_t error_q16;
    int64_t incpid_q32;

    // error = setpoint - feedback
    error_q16 = sPID_HI_CCW.SetPoint_q16 - NextPoint_q16;

    // deadband: ±30 (Q16.16)
    if (error_q16 < (30 << 16) && error_q16 > -(30 << 16))
        error_q16 = 0;

    // Incremental PID (Q16.16 math, use int64 to avoid overflow)
    incpid_q32 =
        (int64_t)sPID_HI_CCW.Kp_q16 * error_q16
      - (int64_t)sPID_HI_CCW.Ki_q16 * sPID_HI_CCW.LastError_q16
      + (int64_t)sPID_HI_CCW.Kd_q16 * sPID_HI_CCW.PrevError_q16;

    // Back to Q16.16
    incpid_q32 >>= 16;

    // update error history
    sPID_HI_CCW.PrevError_q16 = sPID_HI_CCW.LastError_q16;
    sPID_HI_CCW.LastError_q16 = error_q16;

    // 最後只做一次 truncation（不 round）
    return (int32_t)(incpid_q32>>16);
}
#define PWM_MAX 2210
//extern uint32_t t3, t4;

void detect_enter_boot_speed_CW_HI(uint16_t rpm, uint16_t current) // 正轉高速強力鎖付函數
{
	//CW_HI_BootSpeed_Flag=1;
    if ((varcount > 5) && (((int)prev_rpm_cw - (int)G_ScrewRPM) > 40)) // 過高速正轉cmk專用項 // 3以上才能正確跳脫 2以下有時抖動會有雜訊變成條件意外進入加速狀態，建議判斷至少(int)prev_rpm_cw - (int)G_ScrewRPM)都要2以上，意思就是遇到有負載
    {
        CW_HI_BootSpeed_Flag = 1; // 確定有遇到負載
    }
    else if (G_Current > 500)
    {
        CW_HI_BootSpeed_Flag = 1;
    }
    else
    {
    }

    /* if (varcount <= 199)
     {

         Temp_var[0][varcount] = G_ScrewRPM;
         Temp_var[1][varcount] = G_Current;
         Temp_var[2][varcount] = G_PwmCMR;
     }
         */
    /*
     */

    varcount++;

    if (CW_HI_BootSpeed_Flag == 1) // 第一種強力鎖付模式，先進入第一階段全力鎖付力道
    {

        G_PwmCMR = 2210;
    }

    else
    {
    }

    prev_rpm_cw = rpm;
    prev_current_cw = G_Current;
    prev_pwm_cw = G_PwmCMR;
}

void detect_enter_boot_speed_CW_LO(uint16_t rpm, uint16_t current) // 正轉高速強力鎖付函數
{

    uint8_t i;

    if ((varcount > 5) && (((int)prev_rpm_cw - (int)G_ScrewRPM) > 25)) // 3以上才能正確跳脫 2以下有時抖動會有雜訊變成條件意外進入加速狀態，建議判斷至少(int)prev_rpm_cw - (int)G_ScrewRPM)都要2以上，意思就是遇到有負載
    {
        CW_LO_BootSpeed_Flag = 1;
    }
    else if (G_Current > 500)
    {
        CW_LO_BootSpeed_Flag = 1;
    }
    else
    {
    }

    /*if (varcount <= 199)
    {

        Temp_var[0][varcount] = G_ScrewRPM;
        Temp_var[1][varcount] = G_Current;
        Temp_var[2][varcount] = G_PwmCMR;
    }
     */
    varcount++;

    if (CW_LO_BootSpeed_Flag == 1) // 第一種強力鎖付模式，先進入第一階段全力鎖付力道
    {

        G_PwmCMR = 2210;
    }

    else
    {
    }

    prev_rpm_cw = rpm;
    prev_current_cw = G_Current;
    prev_pwm_cw = G_PwmCMR;
}

void detect_enter_boot_speed_CCW_HI(uint16_t rpm, uint16_t current) // 偵測進入加速模式在反轉階段
{

    varcount++;

    if ((varcount == 10) && (G_ScrewRPM < 500))
    {
        CCW_HI_BootSpeed_Flag = 1;
    }

    if (CCW_HI_BootSpeed_Flag == 1) // 進入全力鎖付模式
    {

        enter_boot_speed_mode = 1;
        G_PwmCMR = 2210;
    }

    prev_rpm_ccw = rpm;
    prev_current_ccw = current;
}
void detect_enter_boot_speed_CCW_LO(uint16_t rpm, uint16_t current) // 偵測進入加速模式在反轉階段
{

    varcount++;

    if ((varcount == 10) && (G_ScrewRPM < 600))
    {
        CCW_LO_BootSpeed_Flag = 1;
    }

    if (CCW_LO_BootSpeed_Flag == 1) // 第一階段旗標和第二階段判斷旗標都成立時
    {

        enter_boot_speed_mode = 1;
        G_PwmCMR = 2210;
    }

    prev_rpm_ccw = rpm;
    prev_current_ccw = current;
}

void Exponential_PWM_Ramp_CCW(void) // 增量型pid加速程式-逆轉
{

    WDT_RESET_COUNTER();

    G_PwmCMR = PWM_MAX;
}

extern uint32_t oc_timestamp;

void default_init(void) // 每次解除按壓馬達按鍵還原參數值
{
    int i, i2;
    // LIGHT_OFF;
    // C_PtsPta_Buzzer_OFF;
    /*if (varcount > 0)
    {
        for (i = 0; i < varcount; i++)
        {

            printf("%d G_ScrewRPM = %d,\t\t,G_Current=%d,\t\tG_PwmCMR = %d\r\n",i+1, Temp_var[0][i], Temp_var[1][i], Temp_var[2][i]);
        }

        varcount = 0;
    }
      */

    CW_HI_BootSpeed_Flag = 0;
    CW_LO_BootSpeed_Flag = 0;
    CCW_LO_BootSpeed_Flag = 0;
    CCW_HI_BootSpeed_Flag = 0;
    varcount = 0;

    Enter_HI_Loop = 0;
    HI_Loop = 0;

    HI_count_300ms = 0;
    jump_pwm = 0;

    Enter_LO_Loop = 0;
    LO_Loop = 0;
    LO_count_300ms = 0;
    jump_pwm222 = 0;
    bat = G_Voltage_Battery;
    L_TargetCount = 42;

    rpm_now = 0, rpm_prev = 0;
    current_now = 0, current_prev = 0;

    /*sPID_LO_CW.SetPoint = TARGET_SPEED_LO;  //
    sPID_HI_CW.SetPoint = TARGET_SPEED_HI;  //
    sPID_LO_CCW.SetPoint = TARGET_SPEED_LO; //
    sPID_HI_CCW.SetPoint = TARGET_SPEED_HI; //
    */
		
		sPID_HI_CW.SetPoint_q16 = TARGET_SPEED_HI_Q16;
	  sPID_HI_CCW.SetPoint_q16 = TARGET_SPEED_HI_Q16; //	 
		sPID_LO_CW.SetPoint_q16 = TARGET_SPEED_LO_Q16;
	  sPID_LO_CCW.SetPoint_q16 = TARGET_SPEED_LO_Q16; //	 
			 
    prev_rpm_cw = 0;
    prev_current_cw = 0;
    prev_rpm_ccw = 0;
    prev_current_ccw = 0;

    enter_boot_speed_mode = 0;

    //t = 0;
   // t2 = 0;
   // t3 = 0;

    G_StallCnt = 0;

    oc_timestamp = 0;
    G_PwmCMR = 0;

    PID_ParamInit_HI_CCW();
    PID_ParamInit_HI_CW();
    PID_ParamInit_LO_CCW();
    PID_ParamInit_LO_CW();
    /*for (i = 0; i < AVG_WINDOW; i++)
    {
        current_buf[i] = 0;
    }
    for (i = 0; i < SPEED_AVG_WINDOW; i++)
        speed_buf[i] = 0;
        */
    // avg_current = 0;
    //  avg_speed = 0;
    G_ScrewRPM = 0;
    varcount = 0;
    prev_pwm_cw = 0;
    parm_init();
		//////////
	
}

void uvw_check_function(void) // uvw異常的判斷程式
{

    if (gu8StartSpeed == HI)
    {
        if (G_StallCnt > 10 && G_MotorStatus == eMotorStatus_Normal && HI_count_300ms < 20)
        {
            errorflag = eBehaving_Uvw_Error; // 1.eBehaving_eBehaving_Uvw_Error
            G_BuzzerLED = errorflag;
            Buzzer_LED_Behaving(&G_BuzzerLED);
        }
    }
    else
    {
        if (G_StallCnt > 10 && G_MotorStatus == eMotorStatus_Normal && LO_count_300ms < 20)
        {
            errorflag = eBehaving_Uvw_Error; // 1.eBehaving_eBehaving_Uvw_Error
            G_BuzzerLED = errorflag;
            Buzzer_LED_Behaving(&G_BuzzerLED);
        }
    }
}

void parm_init(void) // 馬達壓板按壓後第一次會先還原預設值
{

    if (gu8StartSpeed == HI)
    {

        if (Enter_HI_Loop == 0) // real 2ms
        {

            
            enter_boot_speed_mode = 0;
          

            sPID_HI_CW.PrevError_q16 = 0;
            sPID_HI_CW.LastError_q16 = 0;
            sPID_HI_CW.Kp_q16 = SPD_P_DATA_HI_Q16;
            sPID_HI_CW.Ki_q16 = SPD_I_DATA_HI_Q16;
            iError_HI_CW = 0;
            iIncpid_HI_CW = 0;
            sPID_HI_CCW.PrevError_q16 = 0;
            sPID_HI_CCW.LastError_q16 = 0;
            sPID_HI_CCW.Kp_q16 = SPD_P_DATA_HI_Q16;
            sPID_HI_CCW.Ki_q16 = SPD_I_DATA_HI_Q16;
            iError_HI_CCW = 0;
            iIncpid_HI_CCW = 0;
            HI_count_300ms = 0;

            T_1ms222 = 0;

        } // 20.4v 114.5pwm  20.1 119.5pwm
    }
    else
    {
        if (Enter_LO_Loop == 0) // real 2ms
        {

            //  Enter_LO_Loop = 1;
            enter_boot_speed_mode = 0;
        

            sPID_LO_CW.PrevError_q16 = 0;
            sPID_LO_CW.LastError_q16 = 0;
            sPID_LO_CW.Kp_q16 = SPD_P_DATA_LO_Q16;
            sPID_LO_CW.Ki_q16 = SPD_I_DATA_LO_Q16;
            iError_LO_CW = 0;
            iIncpid_LO_CW = 0;
            sPID_LO_CCW.PrevError_q16 = 0;
            sPID_LO_CCW.LastError_q16 = 0;
            sPID_LO_CCW.Kp_q16 = SPD_P_DATA_LO_Q16;
            sPID_LO_CCW.Ki_q16 = SPD_I_DATA_LO_Q16;
            iError_LO_CCW = 0;
            iIncpid_LO_CCW = 0;
            sPID_LO_CW.SetPoint_q16 = TARGET_SPEED_LO_Q16;
            LO_count_300ms = 0;

            T_1ms222 = 0;
        }
    }
}
#define PWM_MIN   (0)
int32_t pwm_inc;      // PID 回傳（signed）
int32_t pwm_next;     // 下一步 PWM（signed）
unsigned int pwm_clamp_to_uint(int32_t val)
{
    if (val < PWM_MIN) return PWM_MIN;
    if (val > PWM_MAX) return PWM_MAX;
    return (unsigned int)val;
}
void motor_algorithm_HI(void) // 馬達演算法
{

    if (T_1ms222 >= 3)
    {
        T_1ms222 = 0;
				
        HI_Loop++;
        if (G_Direction == _CW) // 正轉部分
        {
            if (HI_Loop == 1)
            {
               /* 目前 PWM（轉成 signed 用來算） */
 temppwm222 = G_PwmCMR;

/* PID 增量（signed） */
temppwm = SpdPIDCalc_HI_CW_Q16(Q16(G_ScrewRPM));
							//printf("temppwm=%d\n",temppwm);
							G_PwmCMR=pwm_clamp_to_uint(temppwm + temppwm222);
							/*
							❌ 錯誤 3：超界時「什麼都不做」
							if (overflow) { } else { G_PwmCMR = ...; }    // ❌


							👉 這三種在 2 nm Cmk 都很容易炸
							*/

            }
            else if (HI_Loop == 3)
            {
                // printf("%d,%d,%d,%d,1,1\n", G_ScrewRPM, G_Current, G_PwmCMR, G_Voltage_Battery); // 這個部分要搭配程式開cmk才會過，不然扭力輸出打cmk會不穩
                // printf("iError_HI_CW=%f\n\n",iError_HI_CW);
							 detect_enter_boot_speed_CW_HI(G_ScrewRPM, G_Current); // 偵測馬達是否遇到阻力,則進加速模式 
				if(CW_HI_BootSpeed_Flag==0)
                {
                  
                   
                   
                     
                }
                
                 else
                
                {}
                             HI_Loop = 0;  
            
                
                                                                      // HI_Loop = 0;
								
                
            }
           /* else if (HI_Loop >= 4) // 2m*5=10ms
            {
            }
						*/
            else
            {
            }
        }
        else // 反轉部分 if (G_Direction == _CCW)
        {
            if (HI_Loop == 1)
            {
                             /* 目前 PWM（轉成 signed 用來算） */
  temppwm222 = G_PwmCMR;

/* PID 增量（signed） */
temppwm = SpdPIDCalc_HI_CCW_Q16(Q16(G_ScrewRPM));
	G_PwmCMR=pwm_clamp_to_uint(temppwm + temppwm222);
            }
            else if (HI_Loop == 3)
            {
							 detect_enter_boot_speed_CCW_HI(G_ScrewRPM, G_Current); // 偵測馬達是否遇到阻力,則進加速模式
                if (enter_boot_speed_mode == 0)
                {
                   
                    
                    HI_Loop = 0;
                }
                else
                {
                    // Exponential_PWM_Ramp_CCW();
                }
                // printf("%d,%d,%d,%d,1,1\n", G_ScrewRPM, G_Current, G_PwmCMR, G_Voltage_Battery); // 這個部分要搭配程式開cmk才會過，不然扭力輸出打cmk會不穩
                //  printf("iError_HI_CCW=%f\n\n",iError_HI_CCW);
            }
           /* else if (HI_Loop >= 4) // 2m*5=10ms
            {
            }
						*/
            else
            {
            }
        }
    }
}

void motor_algorithm_LO(void) // 馬達演算法
{

    if (T_1ms222 >= 3) // 1ms以內
    {
        T_1ms222 = 0;
		 
        LO_Loop++;
        if (G_Direction == _CW) // 正轉部分
        {
            if (LO_Loop == 1)
            {
                             /* 目前 PWM（轉成 signed 用來算） */
  temppwm222 = G_PwmCMR;

/* PID 增量（signed） */
temppwm = SpdPIDCalc_LO_CW_Q16(Q16(G_ScrewRPM));

 	G_PwmCMR=pwm_clamp_to_uint(temppwm + temppwm222);

            }
            else if (LO_Loop == 3)
            {
							 detect_enter_boot_speed_CW_LO(G_ScrewRPM, G_Current); // 偵測馬達是否遇到阻力,則進加速模式
               /* if(CW_LO_BootSpeed_Flag==0)
                {
                  
                   
                   
                     
                }
                
                 else
                
                {}
                */
                // printf("%d,%d,%d,%d,1,1\n", G_ScrewRPM, G_Current, G_PwmCMR, G_Voltage_Battery); // 這個部分要搭配程式開cmk才會過，不然扭力輸出打cmk會不穩
                //  printf("%d,%d,%d,%d,1,1\n", G_ScrewRPM, G_Current, G_PwmCMR, G_Voltage_Battery); // 這個部分要搭配程式開cmk才會過，不然扭力輸出打cmk會不穩
                 LO_Loop = 0;
            }
           /* else if (LO_Loop >= 4) // 2m*5=10ms
            {

                // LO_Loop = 0;
            }
						*/
            else
            {
            }
        }
        else // 反轉部分 if (G_Direction == _CCW) //
        {
            if (LO_Loop == 1) // 2m*5=10ms
            {
                                            /* 目前 PWM（轉成 signed 用來算） */
  temppwm222 = G_PwmCMR;

/* PID 增量（signed） */
temppwm = SpdPIDCalc_LO_CCW_Q16(Q16(G_ScrewRPM));
 	G_PwmCMR=pwm_clamp_to_uint(temppwm + temppwm222);

            }
            else if (LO_Loop == 3)
            {
							  detect_enter_boot_speed_CCW_LO(G_ScrewRPM, G_Current); // 偵測馬達是否遇到阻力,則進加速模式
                if (enter_boot_speed_mode == 0)
                {
                  
                                                                 // if (enter_boot_speed_mode == 0)
                    LO_Loop = 0;
                }
                else
                {
                    // Exponential_PWM_Ramp_CCW();
                }

                // printf("%d,%d,%d,%d,1,1\n", G_ScrewRPM, G_Current, G_PwmCMR, G_Voltage_Battery); // 這個部分要搭配程式開cmk才會過，不然扭力輸出打cmk會不穩
                // printf("%d,%d,%d,%d,1,1\n", G_ScrewRPM, G_Current, G_PwmCMR, G_Voltage_Battery); // 這個部分要搭配程式開cmk才會過，不然扭力輸出打cmk會不穩

               // LO_Loop = 0;
            }
          /*  else if (LO_Loop >= 4) // 2m*5=10ms
            {
            }
						*/
            else
            {
            }
        }
    }
}
