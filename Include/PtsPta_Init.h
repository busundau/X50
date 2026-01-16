#ifndef  TBK_Init
#define  TBK_Init

void GPIO_Init(void);
void UART0_Init(void);
void PWM_Init(void);
void TIMER_Init(void);
void ADC_Init(void);
void I2C_Init(void);
void Variable_Init(void);
#if(WatchDog == ENABLE)
void WDT_Init(void);
#endif
#if 0



void UART1_Init(void);
void I2C_Init(void);
void TIMER_Init(void);
#endif


#endif 
