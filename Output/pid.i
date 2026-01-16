#line 1 "Source\\pid.c"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


  



    typedef unsigned int size_t;    









 
 

 



    typedef struct __va_list __va_list;






   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

#line 136 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 166 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));


#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int __ARM_vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int __ARM_vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int __ARM_vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));
   








 

extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 1021 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"



 

#line 2 "Source\\pid.c"
#line 1 ".\\Include\\M051Series.h"
 









 


















 









































 









 



 
typedef enum IRQn
{
     
    NonMaskableInt_IRQn       = -14,       
    HardFault_IRQn            = -13,       
    SVCall_IRQn               = -5,        
    PendSV_IRQn               = -2,        
    SysTick_IRQn              = -1,        

     
    BOD_IRQn                  = 0,         
    WDT_IRQn                  = 1,         
    EINT0_IRQn                = 2,         
    EINT1_IRQn                = 3,         
    GPIO_P0P1_IRQn            = 4,         
    GPIO_P2P3P4_IRQn          = 5,         
    PWMA_IRQn                 = 6,         
    PWMB_IRQn                 = 7,         
    TMR0_IRQn                 = 8,         
    TMR1_IRQn                 = 9,         
    TMR2_IRQn                 = 10,        
    TMR3_IRQn                 = 11,        
    UART0_IRQn                = 12,        
    UART1_IRQn                = 13,        
    SPI0_IRQn                 = 14,        
    SPI1_IRQn                 = 15,        
    I2C0_IRQn                 = 18,        
    I2C1_IRQn                 = 19,        
    ACMP01_IRQn               = 25,        
    ACMP23_IRQn               = 26,        
    PWRWU_IRQn                = 28,        
    ADC_IRQn                  = 29         

} IRQn_Type;






 

 





#line 1 ".\\Include\\core_cm0.h"
 




















 













 












 




 


 

 













#line 89 ".\\Include\\core_cm0.h"


 







#line 114 ".\\Include\\core_cm0.h"

#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 116 ".\\Include\\core_cm0.h"
#line 1 ".\\Include\\core_cmInstr.h"
 




















 






 


 



 


 









 







 







 






 








 







 







 









 









 
__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 
__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}









 



#line 268 ".\\Include\\core_cmInstr.h"



#line 619 ".\\Include\\core_cmInstr.h"

   

   

#line 117 ".\\Include\\core_cm0.h"
#line 1 ".\\Include\\core_cmFunc.h"
 




















 






 

 



 


 





 
 






 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}







 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}







 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}







 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}







 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}







 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}







 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}







 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}


#line 260 ".\\Include\\core_cmFunc.h"


#line 296 ".\\Include\\core_cmFunc.h"


#line 615 ".\\Include\\core_cmFunc.h"

 

   

#line 118 ".\\Include\\core_cm0.h"








 
#line 143 ".\\Include\\core_cm0.h"

 






 
#line 159 ".\\Include\\core_cm0.h"

 










 


 





 


 
typedef union
{
  struct
  {

    uint32_t _reserved0:27;               





    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       

    uint32_t _reserved0:15;               





    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 


 
typedef struct
{
  volatile uint32_t ISER[1];                  
       uint32_t RESERVED0[31];
  volatile uint32_t ICER[1];                  
       uint32_t RSERVED1[31];
  volatile uint32_t ISPR[1];                  
       uint32_t RESERVED2[31];
  volatile uint32_t ICPR[1];                  
       uint32_t RESERVED3[31];
       uint32_t RESERVED4[64];
  volatile uint32_t IP[8];                    
}  NVIC_Type;

 






 


 
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
       uint32_t RESERVED0;
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
       uint32_t RESERVED1;
  volatile uint32_t SHP[2];                   
  volatile uint32_t SHCSR;                    
} SCB_Type;

 















 



























 















 









 






 



 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 








 
 






 

 










 









 

 



 




 

 
 










 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}











 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}







 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}










 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] = (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] = (((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
}












 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) >> (8 - 2)));  }  
  else {
    return((uint32_t)((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[ ( ((uint32_t)(IRQn) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) >> (8 - 2)));  }  
}





 
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FA << 16)      |
                 (1UL << 2));
  __dsb(0xF);                                                      
  while(1);                                                     
}

 



 




 

















 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if (ticks > (0xFFFFFFUL << 0))  return (1);             

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (ticks & (0xFFFFFFUL << 0)) - 1;       
  NVIC_SetPriority (SysTick_IRQn, (1<<2) - 1);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2) |
                   (1UL << 1)   |
                   (1UL << 0);                     
  return (0);                                                   
}



 








   

#line 136 ".\\Include\\M051Series.h"
#line 1 ".\\Include\\system_M051Series.h"
 









 







 
 
 











 




 





extern uint32_t SystemCoreClock;     
extern uint32_t CyclesPerUs;         
extern uint32_t PllClock;            

#line 65 ".\\Include\\system_M051Series.h"












 
extern void SystemInit(void);











 
extern void SystemCoreClockUpdate(void);







 
#line 137 ".\\Include\\M051Series.h"



#pragma anon_unions











 
extern void SystemInit(void);


 
 
 





 

 




 

typedef struct
{







































 

    volatile uint32_t CR[2];          
    volatile uint32_t SR;             

} ACMP_T;






 

 















 












 











   
   


 



 

typedef struct
{











































































































































































































 

    volatile const  uint32_t ADDR[8];        
    volatile uint32_t ADCR;           
    volatile uint32_t ADCHER;         
    volatile uint32_t ADCMPR[2];      
    volatile uint32_t ADSR;           
    volatile const  uint32_t RESERVED[4];
    volatile uint32_t ADTDCR;         
} ADC_T;







 
 









 



























 






 


















 





















 


   
   


 




 


typedef struct
{































































































































































































































































































































 

    volatile uint32_t PWRCON;         
    volatile uint32_t AHBCLK;         
    volatile uint32_t APBCLK;         
    volatile uint32_t CLKSTATUS;      
    volatile uint32_t CLKSEL0;        
    volatile uint32_t CLKSEL1;        
    volatile uint32_t CLKDIV;         
    volatile uint32_t CLKSEL2;        
    volatile uint32_t PLLCON;         
    volatile uint32_t FRQDIV;         

} CLK_T;






 

 




























 









 



























































 



















 






 

































 












 









 





















 








   
   


 



 

typedef struct
{


















































 

    volatile uint32_t EBICON;         
    volatile uint32_t EXTIME;         
    volatile uint32_t EBICON2;        

} EBI_T;







 
 












 












 








   
   


 





 

typedef struct
{



















































































































 

    volatile uint32_t ISPCON;         
    volatile uint32_t ISPADR;         
    volatile uint32_t ISPDAT;         
    volatile uint32_t ISPCMD;         
    volatile uint32_t ISPTRG;         
    volatile const  uint32_t DFBADR;         
    volatile uint32_t FATCON;         
    volatile const  uint32_t RESERVED[9];   
    volatile uint32_t ISPSTA;         
} FMC_T;






 
 



















 









 



 



 











   
   


 



 

typedef struct
{



























































































































 

    volatile uint32_t PMD;            
    volatile uint32_t OFFD;           
    volatile uint32_t DOUT;           
    volatile uint32_t DMASK;          
    volatile uint32_t PIN;            
    volatile uint32_t DBEN;           
    volatile uint32_t IMD;            
    volatile uint32_t IEN;            
    volatile uint32_t ISRC;           

} GPIO_T;



typedef struct
{
































 

    volatile uint32_t DBNCECON;       

} GPIO_DBNCECON_T;







 

 
























 



 



 



 



 



 



 






 



 








   
   



 



 

typedef struct
{













































 

    volatile int32_t  DIVIDEND;       
    volatile int32_t  DIVISOR;        
    volatile int32_t  DIVQUO;         
    volatile int32_t  DIVREM;         
    volatile uint32_t DIVSTS;         

} HDIV_T;







 







   
   



 




 

typedef struct
{
























































































































































































 

    volatile uint32_t I2CON;          
    volatile uint32_t I2CADDR0;       
    volatile uint32_t I2CDAT;         
    volatile const  uint32_t I2CSTATUS;      
    volatile uint32_t I2CLK;          
    volatile uint32_t I2CTOC;         
    volatile uint32_t I2CADDR1;       
    volatile uint32_t I2CADDR2;       
    volatile uint32_t I2CADDR3;       
    volatile uint32_t I2CADM0;        
    volatile uint32_t I2CADM1;        
    volatile uint32_t I2CADM2;        
    volatile uint32_t I2CADM3;        
    volatile const  uint32_t RESERVED0[2]; 
    volatile uint32_t I2CWKUPCON;     
    volatile uint32_t I2CWKUPSTS;     

} I2C_T;






 

 


















 






 



 



 



 









 



 



 



   
   



 




 

typedef struct
{























































































































































































































































































































































































































































































































































































































































 

    volatile uint32_t PPR;            
    volatile uint32_t CSR;            
    volatile uint32_t PCR;            
    volatile uint32_t CNR0;           
    volatile uint32_t CMR0;           
    volatile const  uint32_t PDR0;           
    volatile uint32_t CNR1;           
    volatile uint32_t CMR1;           
    volatile const  uint32_t PDR1;           
    volatile uint32_t CNR2;           
    volatile uint32_t CMR2;           
    volatile const  uint32_t PDR2;           
    volatile uint32_t CNR3;           
    volatile uint32_t CMR3;           
    volatile const  uint32_t PDR3;           
    volatile const  uint32_t RESERVE0;     
    volatile uint32_t PIER;           
    volatile uint32_t PIIR;           
    volatile const  uint32_t RESERVE1[2];  
    volatile uint32_t CCR0;           
    volatile uint32_t CCR2;           
    volatile uint32_t CRLR0;          
    volatile uint32_t CFLR0;          
    volatile uint32_t CRLR1;          
    volatile uint32_t CFLR1;          
    volatile uint32_t CRLR2;          
    volatile uint32_t CFLR2;          
    volatile uint32_t CRLR3;          
    volatile uint32_t CFLR3;          
    volatile uint32_t CAPENR;         
    volatile uint32_t POE;            
    volatile uint32_t TCON;           
    volatile uint32_t TSTATUS;        
    volatile const  uint32_t RESERVE2[4];  
    volatile uint32_t PSCR;           

} PWM_T;






 

 












 












 




























































 



 



 




 




































 
























 










































 










































 



 



 












 












 
























 












 











   
   



 



 

typedef struct
{


































































































































































































































































































































































































 

    volatile uint32_t CNTRL;          
    volatile uint32_t DIVIDER;        
    volatile uint32_t SSR;            
    volatile const  uint32_t RESERVE0;     
    volatile const  uint32_t RX0;            
    volatile const  uint32_t RX1;            
    volatile const  uint32_t RESERVE1[2];  
    volatile  uint32_t TX0;            
    volatile  uint32_t TX1;            
    volatile const  uint32_t RESERVE2[3];  
    volatile uint32_t VARCLK;         
    volatile const  uint32_t RESERVE3;     
    volatile uint32_t CNTRL2;         
    volatile uint32_t FIFO_CTL;       
    volatile uint32_t STATUS;         

} SPI_T;






 

 






















































 






 















 





















 
























 



































   
   



 




 

typedef struct
{



























































































































































































































































































































































































































































































































































 

    volatile const  uint32_t PDID;           
    volatile uint32_t RSTSRC;         
    volatile uint32_t IPRSTC1;        
    volatile uint32_t IPRSTC2;        
    volatile const  uint32_t RESERVED0[2]; 
    volatile uint32_t BODCR;          
    volatile uint32_t TEMPCR;         
    volatile const  uint32_t RESERVED1;    
    volatile uint32_t PORCR;          
    volatile const  uint32_t RESERVED2[2]; 
    volatile uint32_t P0_MFP;         
    volatile uint32_t P1_MFP;         
    volatile uint32_t P2_MFP;         
    volatile uint32_t P3_MFP;         
    volatile uint32_t P4_MFP;         
    volatile const  uint32_t RESERVED3[47];
    volatile uint32_t REGWRPROT;      

} GCR_T;







 

 





















 












 


















































 





















 



 



 












 









 









 









 









 





   



typedef struct
{

























































































































 

    volatile const  uint32_t IRQSRC[32];     
    volatile uint32_t NMISEL;         

} GCR_INT_T;






 

 



 





   
   



 



 

typedef struct
{








































































































































 

    volatile uint32_t TCSR;           
    volatile uint32_t TCMPR;          
    volatile uint32_t TISR;           
    volatile uint32_t TDR;            
    volatile uint32_t TCAP;           
    volatile uint32_t TEXCON;         
    volatile uint32_t TEXISR;         

} TIMER_T;






 

 










































 



 






 



 



 





















 


   
   



 




 

typedef struct
{




















































































































































































































































































































































































































 

    union {
        volatile uint32_t DATA;           
        volatile uint32_t THR;            
        volatile uint32_t RBR;            
    };
    volatile uint32_t IER;            
    volatile uint32_t FCR;            
    volatile uint32_t LCR;            
    volatile uint32_t MCR;            
    volatile uint32_t MSR;            
    volatile uint32_t FSR;            
    volatile uint32_t ISR;            
    volatile uint32_t TOR;            
    volatile uint32_t BAUD;           
    volatile uint32_t IRCR;           
    volatile uint32_t ALT_CSR;        
    volatile uint32_t FUN_SEL;        
} UART_T;






 

 



 



 

































 















 


















 









 









 







































 










































 






 












 









 
























 


   
   



 



 

typedef struct
{




















































 

    volatile uint32_t WTCR;           
    volatile uint32_t WTCRALT;        

} WDT_T;






 

 






























 


   
   


 



 

typedef struct
{

























































 

    volatile uint32_t WWDTRLD;        
    volatile uint32_t WWDTCR;         
    volatile uint32_t WWDTSR;         
    volatile uint32_t WWDTCVR;        

} WWDT_T;






 

 



 















 






 


   
   
   


 
 
 



 
 






 
#line 5821 ".\\Include\\M051Series.h"









































   

 
 
 




 
#line 5878 ".\\Include\\M051Series.h"







































   



typedef volatile unsigned char  vu8;
typedef volatile unsigned long  vu32;
typedef volatile unsigned short vu16;




#line 5934 ".\\Include\\M051Series.h"

#line 5941 ".\\Include\\M051Series.h"













 
#line 5987 ".\\Include\\M051Series.h"


 











 
 
 
#line 1 ".\\Include\\sys.h"
 









 











 



 



 

 
 
 
#line 54 ".\\Include\\sys.h"


 
 
 
#line 65 ".\\Include\\sys.h"


 
 
 







 





#line 89 ".\\Include\\sys.h"

#line 96 ".\\Include\\sys.h"

#line 103 ".\\Include\\sys.h"

#line 110 ".\\Include\\sys.h"











































#line 159 ".\\Include\\sys.h"

#line 166 ".\\Include\\sys.h"













































#line 217 ".\\Include\\sys.h"






#line 229 ".\\Include\\sys.h"

#line 236 ".\\Include\\sys.h"







#line 249 ".\\Include\\sys.h"












#line 267 ".\\Include\\sys.h"











































   



 







 








 








 








 








 









 








 








 








 












 








 








 








 









 








 








 








 








 








 








 








 














 








 
static __inline void SYS_LockReg(void)
{
    ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->REGWRPROT = 0;
}







 
static __inline void SYS_UnlockReg(void)
{
    while(((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->REGWRPROT != (1ul << 0))
    {
        ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->REGWRPROT = 0x59;
        ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->REGWRPROT = 0x16;
        ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->REGWRPROT = 0x88;
    }
}




void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetBODStatus(void);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
uint32_t SYS_ReadPDID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
void SYS_DisableBOD(void);


   

   

   






#line 6005 ".\\Include\\M051Series.h"
#line 1 ".\\Include\\clk.h"
 









 













 



 



 







 
 
 
#line 53 ".\\Include\\clk.h"


 
 
 
#line 99 ".\\Include\\clk.h"


 
 
 
#line 118 ".\\Include\\clk.h"


 
 
 





 
 
 














 
 
 
#line 157 ".\\Include\\clk.h"
 
 
 





#line 185 ".\\Include\\clk.h"


   



 







 
static __inline uint32_t CLK_GetPLLClockFreq(void)
{
    uint32_t u32PllFreq = 0, u32PllReg;
    uint32_t u32FIN, u32NF, u32NR, u32NO;
    uint8_t au8NoTbl[4] = {1, 2, 2, 4};

    u32PllReg = ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->PLLCON;

    if(u32PllReg & ((1ul << 16) | (1ul << 18)))
        return 0;            

    if(u32PllReg & 0x00080000UL)
        u32FIN = (22118400UL);     
    else
        u32FIN = (12000000UL);      

    if(u32PllReg & (1ul << 17))
        return u32FIN;       

     
    u32NO = au8NoTbl[((u32PllReg & (3ul << 14)) >> 14)];
    u32NF = ((u32PllReg & (0x1FFul << 0)) >> 0) + 2;
    u32NR = ((u32PllReg & (0x1Ful << 9)) >> 9) + 2;

     
    u32PllFreq = (((u32FIN >> 2) * u32NF) / (u32NR * u32NO) << 2);

    return u32PllFreq;
}









 
static __inline void CLK_SysTickDelay(uint32_t us)
{
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = us * CyclesPerUs;
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL  = (0x00);
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2) | (1UL << 0);

     
    while((((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL & (1UL << 16)) == 0);
    
     
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = 0;    
}
static __inline void CLK_SysTickLongDelay(uint32_t us)
{
    uint32_t delay;
        
     
    delay = 335544UL;

    do
    {
        if(us > delay)
        {
            us -= delay;
        }
        else
        {
            delay = us;
            us = 0UL;
        }        
        
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = delay * CyclesPerUs;
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL  = (0x0UL);
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2) | (1UL << 0);

         
        while((((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL & (1UL << 16)) == 0UL);

         
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = 0UL;
    
    }while(us > 0UL);
    
}

void CLK_DisableCKO(void);
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En);
void CLK_PowerDown(void);
void CLK_Idle(void);
uint32_t CLK_GetHXTFreq(void);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetPCLKFreq(void);
uint32_t CLK_GetCPUFreq(void);
uint32_t CLK_SetCoreClock(uint32_t u32Hclk);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq);
void CLK_DisablePLL(void);
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask);
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count);
void CLK_DisableSysTick(void);




   

   

   











 
#line 6006 ".\\Include\\M051Series.h"
#line 1 ".\\Include\\adc.h"
 








 











 



 



 
 
 
 

























 
 
 




 
 
 
#line 75 ".\\Include\\adc.h"

 
 
 




 
 
 





 
 
 





 
 
 



 
 
 





   



 











 








 












 












 








 









 









 







 







 















 
#line 235 ".\\Include\\adc.h"






 















 
#line 267 ".\\Include\\adc.h"






 









 











 







 














 


void ADC_Open(ADC_T *adc,
              uint32_t u32InputMode,
              uint32_t u32OpMode,
              uint32_t u32ChMask);
void ADC_Close(ADC_T *adc);
void ADC_EnableHWTrigger(ADC_T *adc,
                         uint32_t u32Source,
                         uint32_t u32Param);
void ADC_DisableHWTrigger(ADC_T *adc);
void ADC_EnableInt(ADC_T *adc, uint32_t u32Mask);
void ADC_DisableInt(ADC_T *adc, uint32_t u32Mask);



   

   

   







 
#line 6007 ".\\Include\\M051Series.h"

#line 1 ".\\Include\\fmc.h"
 









 



#line 1 ".\\Include\\M051Series.h"
 









 


















 









































 

#line 6020 ".\\Include\\M051Series.h"

 



#line 16 ".\\Include\\fmc.h"









 



 



 


 
 
 







 
 
 



 
 
 
#line 62 ".\\Include\\fmc.h"


   



 

 
 
 









 












 













 













 














 











 













 












 













 















 














 



 
 
 











 
static __inline void FMC_Write(uint32_t u32addr, uint32_t u32data)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x21;    
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = u32addr;               
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT = u32data;               
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = 0x1;                   
    __isb(0xF);                             
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG);                  
}










 
static __inline uint32_t FMC_Read(uint32_t u32addr)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x00;  
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = u32addr;          
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = 0x1;              
    __isb(0xF);                        
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG);             

    return ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT;
}













 
static __inline int32_t FMC_Erase(uint32_t u32addr)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x22;  
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = u32addr;                
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = 0x1;                    
    __isb(0xF);                              
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG);                   

     
    if(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCON & (1ul << 6))
    {
        ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCON |= (1ul << 6);
        return -1;
    }
    return 0;
}










 
static __inline uint32_t FMC_ReadUID(uint8_t u8index)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x04;  
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = (u8index << 2);       
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = 0x1;                  
    __isb(0xF);                            
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG);                 

    return ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT;
}











 
static __inline uint32_t FMC_ReadCID(void)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x0B;            
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = 0x0;                            
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = (1ul << 0);           
    __isb(0xF);                                      
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG & (1ul << 0)) ;   

    return ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT;
}










 
static __inline uint32_t FMC_ReadPID(void)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x0C;           
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = 0x04;                          
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = (1ul << 0);          
    __isb(0xF);                                     
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG & (1ul << 0));   

    return ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT;
}










 
static __inline uint32_t FMC_ReadUCID(uint32_t u32Index)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x04;           
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = (0x04 * u32Index) + 0x10;      
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = (1ul << 0);          
    __isb(0xF);                                     
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG & (1ul << 0));   

    return ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT;
}















 
static __inline void FMC_SetVectorPageAddr(uint32_t u32PageAddr)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x2e;  
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = u32PageAddr;        
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = 0x1;                
    __isb(0xF);                          
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG);               
}














 
static __inline uint32_t FMC_GetVECMAP(void)
{
    return (((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPSTA & (0xffful << 9));
}

extern void FMC_Open(void);
extern void FMC_Close(void);
extern void FMC_EnableAPUpdate(void);
extern void FMC_DisableAPUpdate(void);
extern void FMC_EnableConfigUpdate(void);
extern void FMC_DisableConfigUpdate(void);
extern void FMC_EnableLDUpdate(void);
extern void FMC_DisableLDUpdate(void);
extern int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count);
extern int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count);
extern void FMC_SetBootSource(int32_t i32BootSrc);
extern int32_t FMC_GetBootSource(void);
extern uint32_t FMC_ReadDataFlashBaseAddr(void);

   

   

   








#line 6009 ".\\Include\\M051Series.h"
#line 1 ".\\Include\\gpio.h"
 









 











 



 



 


 
 
 





 
 
 






 
 
 



 
 
 






#line 82 ".\\Include\\gpio.h"














 
#line 138 ".\\Include\\gpio.h"
   




 











 












 











 












 












 












 












 













 



















 










 











 










 













 












 














 












 



void GPIO_SetMode(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_EnableInt(GPIO_T *port, uint32_t u32Pin, uint32_t u32IntAttribs);
void GPIO_DisableInt(GPIO_T *port, uint32_t u32Pin);


   

   

   







 
#line 6010 ".\\Include\\M051Series.h"
#line 1 ".\\Include\\i2c.h"
 









 



#line 16 ".\\Include\\i2c.h"









 



 



 

 
 
 
#line 47 ".\\Include\\i2c.h"




   



 









 










 










 










 











 










 











 











 










 


 
 
 









 
static __inline void I2C_STOP(I2C_T *i2c)
{
    (i2c)->I2CON |= ((1ul << 3) | (1ul << 4));
    while((i2c)->I2CON & (1ul << 4));
}

void I2C_ClearTimeoutFlag(I2C_T *i2c);
void I2C_Close(I2C_T *i2c);
void I2C_Trigger(I2C_T *i2c, uint8_t u8Start, uint8_t u8Stop, uint8_t u8Si, uint8_t u8Ack);
void I2C_DisableInt(I2C_T *i2c);
void I2C_EnableInt(I2C_T *i2c);
uint32_t I2C_GetBusClockFreq(I2C_T *i2c);
uint32_t I2C_SetBusClockFreq(I2C_T *i2c, uint32_t u32BusClock);
uint32_t I2C_GetIntFlag(I2C_T *i2c);
uint32_t I2C_GetStatus(I2C_T *i2c);
uint32_t I2C_Open(I2C_T *i2c, uint32_t u32BusClock);
uint8_t I2C_GetData(I2C_T *i2c);
void I2C_SetSlaveAddr(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddr, uint8_t u8GCMode);
void I2C_SetSlaveAddrMask(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddrMask);
void I2C_EnableTimeout(I2C_T *i2c, uint8_t u8LongTimeout);
void I2C_DisableTimeout(I2C_T *i2c);
void I2C_EnableWakeup(I2C_T *i2c);
void I2C_DisableWakeup(I2C_T *i2c);
void I2C_SetData(I2C_T *i2c, uint8_t u8Data);

   

   

   

#line 6011 ".\\Include\\M051Series.h"
#line 1 ".\\Include\\pwm.h"
 








 











 



 



 
#line 47 ".\\Include\\pwm.h"
 
 
 






   




 










 
#line 81 ".\\Include\\pwm.h"










 
#line 100 ".\\Include\\pwm.h"










 
#line 120 ".\\Include\\pwm.h"









 










 













 

















 













 














 














 
#line 225 ".\\Include\\pwm.h"


uint32_t PWM_ConfigCaptureChannel(PWM_T *pwm,
                                  uint32_t u32ChannelNum,
                                  uint32_t u32UnitTimeNsec,
                                  uint32_t u32CaptureEdge);
uint32_t PWM_ConfigOutputChannel(PWM_T *pwm,
                                 uint32_t u32ChannelNum,
                                 uint32_t u32Frequncy,
                                 uint32_t u32DutyCycle);
void PWM_Start(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_Stop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_ForceStop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
void PWM_DisableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearADCTriggerFlag(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
uint32_t PWM_GetADCTriggerFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableCapture(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_DisableCapture(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_DisableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Duration);
void PWM_DisableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableCaptureInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void PWM_DisableCaptureInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void PWM_ClearCaptureIntFlag(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
uint32_t PWM_GetCaptureIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableDutyInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType);
void PWM_DisableDutyInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearDutyIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetDutyIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType);
void PWM_DisablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);



   

   

   







 
#line 6012 ".\\Include\\M051Series.h"

#line 1 ".\\Include\\timer.h"
 








 











 



 



 
#line 48 ".\\Include\\timer.h"

   




 













 












 











 













 










 
static __inline void TIMER_Start(TIMER_T *timer)
{
    timer->TCSR |= (1ul << 30);
}









 
static __inline void TIMER_Stop(TIMER_T *timer)
{
    timer->TCSR &= ~(1ul << 30);
}











 
static __inline void TIMER_EnableWakeup(TIMER_T *timer)
{
    timer->TCSR |= (1ul << 23);
}









 
static __inline void TIMER_DisableWakeup(TIMER_T *timer)
{
    timer->TCSR &= ~(1ul << 23);
}









 
static __inline void TIMER_EnableCaptureDebounce(TIMER_T *timer)
{
    timer->TEXCON |= (1ul << 6);
}









 
static __inline void TIMER_DisableCaptureDebounce(TIMER_T *timer)
{
    timer->TEXCON &= ~(1ul << 6);
}









 
static __inline void TIMER_EnableEventCounterDebounce(TIMER_T *timer)
{
    timer->TEXCON |= (1ul << 7);
}









 
static __inline void TIMER_DisableEventCounterDebounce(TIMER_T *timer)
{
    timer->TEXCON &= ~(1ul << 7);
}









 
static __inline void TIMER_EnableInt(TIMER_T *timer)
{
    timer->TCSR |= (1ul << 29);
}









 
static __inline void TIMER_DisableInt(TIMER_T *timer)
{
    timer->TCSR &= ~(1ul << 29);
}









 
static __inline void TIMER_EnableCaptureInt(TIMER_T *timer)
{
    timer->TEXCON |= (1ul << 5);
}









 
static __inline void TIMER_DisableCaptureInt(TIMER_T *timer)
{
    timer->TEXCON &= ~(1ul << 5);
}










 
static __inline uint32_t TIMER_GetIntFlag(TIMER_T *timer)
{
    return (timer->TISR & (1ul << 0) ? 1 : 0);
}









 
static __inline void TIMER_ClearIntFlag(TIMER_T *timer)
{
    timer->TISR = (1ul << 0);
}










 
static __inline uint32_t TIMER_GetCaptureIntFlag(TIMER_T *timer)
{
    return timer->TEXISR;
}









 
static __inline void TIMER_ClearCaptureIntFlag(TIMER_T *timer)
{
    timer->TEXISR = (1ul << 0);
}










 
static __inline uint32_t TIMER_GetWakeupFlag(TIMER_T *timer)
{
    return (timer->TISR & (1ul << 1) ? 1 : 0);
}









 
static __inline void TIMER_ClearWakeupFlag(TIMER_T *timer)
{
    timer->TISR = (1ul << 1);
}









 
static __inline uint32_t TIMER_GetCaptureData(TIMER_T *timer)
{
    return timer->TCAP;
}









 
static __inline uint32_t TIMER_GetCounter(TIMER_T *timer)
{
    return timer->TDR;
}

uint32_t TIMER_Open(TIMER_T *timer, uint32_t u32Mode, uint32_t u32Freq);
void TIMER_Close(TIMER_T *timer);
void TIMER_Delay(TIMER_T *timer, uint32_t u32Usec);
void TIMER_EnableCapture(TIMER_T *timer, uint32_t u32CapMode, uint32_t u32Edge);
void TIMER_DisableCapture(TIMER_T *timer);
void TIMER_EnableEventCounter(TIMER_T *timer, uint32_t u32Edge);
void TIMER_DisableEventCounter(TIMER_T *timer);
uint32_t TIMER_GetModuleClock(TIMER_T *timer);

   

   

   







 
#line 6014 ".\\Include\\M051Series.h"
#line 1 ".\\Include\\wdt.h"
 








 











 



 



 
 
 
 
#line 42 ".\\Include\\wdt.h"

 
 
 





   




 









 










 










 











 











 











 













 










 
static __inline void WDT_Close(void)
{
    ((WDT_T *) ((( uint32_t)0x40000000) + 0x4000))->WTCR = 0;
    return;
}









 
static __inline void WDT_EnableInt(void)
{
    ((WDT_T *) ((( uint32_t)0x40000000) + 0x4000))->WTCR |= (1ul << 6);
    return;
}









 
static __inline void WDT_DisableInt(void)
{
    
    ((WDT_T *) ((( uint32_t)0x40000000) + 0x4000))->WTCR &= ~((1ul << 6) | (1ul << 2) | (1ul << 3));
    return;
}

void WDT_Open(uint32_t u32TimeoutInterval, uint32_t u32ResetDelay, uint32_t u32EnableReset, uint32_t u32EnableWakeup);

   

   

   







 
#line 6015 ".\\Include\\M051Series.h"

#line 1 ".\\Include\\uart.h"
 








 












 



 



 

 
 
 



    
 
 
 











 
 
 
















 
 
 



 
 
 



 
 
 





 
 
 





   




 











 











 












 











 












 












 











 











 











 












 











 












 












 





















 




















 



























 











 
static __inline void UART_CLEAR_RTS(UART_T* uart)
{
    (uart)->MCR |= (1ul << 9);
    (uart)->MCR &= ~(1ul << 1);
}








 
static __inline void UART_SET_RTS(UART_T* uart)
{
    (uart)->MCR |= (1ul << 9) | (1ul << 1);
}










 












 



void UART_ClearIntFlag(UART_T* uart , uint32_t u32InterruptFlag);
void UART_Close(UART_T* uart);
void UART_DisableFlowCtrl(UART_T* uart);
void UART_DisableInt(UART_T*  uart, uint32_t u32InterruptFlag);
void UART_EnableFlowCtrl(UART_T* uart);
void UART_EnableInt(UART_T*  uart, uint32_t u32InterruptFlag);
void UART_Open(UART_T* uart, uint32_t u32baudrate);
uint32_t UART_Read(UART_T* uart, uint8_t *pu8RxBuf, uint32_t u32ReadBytes);
void UART_SetLine_Config(UART_T* uart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t  u32stop_bits);
void UART_SetTimeoutCnt(UART_T* uart, uint32_t u32TOC);
void UART_SelectIrDAMode(UART_T* uart, uint32_t u32Buadrate, uint32_t u32Direction);
void UART_SelectRS485Mode(UART_T* uart, uint32_t u32Mode, uint32_t u32Addr);
void UART_SelectLINMode(UART_T* uart, uint32_t u32Mode, uint32_t u32BreakLength);
uint32_t UART_Write(UART_T* uart, uint8_t *pu8TxBuf, uint32_t u32WriteBytes);


   

   

   







 

#line 6017 ".\\Include\\M051Series.h"




 



#line 3 "Source\\pid.c"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
 
 
 
 




 








 












#line 38 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"


  



    typedef unsigned int size_t;    
#line 54 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"




extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 193 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 209 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 232 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 247 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 270 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 502 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"



 

#line 4 "Source\\pid.c"
#line 1 ".\\Include\\PtsPta_TypeDef.h"
#line 4 ".\\Include\\PtsPta_TypeDef.h"
#line 1 ".\\Include\\PtsPta_Motor.h"
 
 
 
 
 




 void Motor(unsigned char Enable_Motor,unsigned char Direction);
 void SetPWMCMR(unsigned int PwmCMR);
 void MotorStart(unsigned char Enable_Motor,unsigned char Direction);


 
 
 
#line 5 ".\\Include\\PtsPta_TypeDef.h"




#line 21 ".\\Include\\PtsPta_TypeDef.h"




typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;






















#line 56 ".\\Include\\PtsPta_TypeDef.h"
































 










 
    	





#line 112 ".\\Include\\PtsPta_TypeDef.h"























 



















 



















 
































	





























 


#line 250 ".\\Include\\PtsPta_TypeDef.h"



















































  

  
  
		
		
		

		
		

		
		

		
		
		
		










 	





#line 373 ".\\Include\\PtsPta_TypeDef.h"












 	
		
		

		
		

		
		
				
		
		

		
		










enum
{

	eForward,          		
	eBrake,          				
	eReverse,          		
};









#line 434 ".\\Include\\PtsPta_TypeDef.h"


















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


enum 
{	
	eBehaving_OFF,
	eBehaving_LowBattery,	
	eBehaving_OutOfBattery,	
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
	eBehaving_BeOnce,	
	eBehaving_BeOnce_OutofBattery,
		eBehaving_Uvw_Error, 
};

enum 
{	
	eBatteryStatus_Normal,
	eBatteryStatus_Error,

};


enum 
{	
	eMotorStatus_Normal,
	eMotorStatus_Over100,
	eMotorStatus_Over90_100,
	eMotorStatus_Stall,
	eMotorStatus_OverCurrent,
	eMotorStatus_Stop,
};

enum 
{	
	ePCBStatus_Normal,
	ePCBStatus_Over80,
	ePCBStatus_Over70_80
};

enum 
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
	eFastMode,  
       eSlowMode,  
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

#line 678 ".\\Include\\PtsPta_TypeDef.h"





			 
			 




			 
			 
#line 5 "Source\\pid.c"
#line 1 ".\\Include\\PtsPta_Global.h"



#line 5 ".\\Include\\PtsPta_Global.h"

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


























































#line 6 "Source\\pid.c"
#line 1 ".\\Include\\pid.h"



#line 5 ".\\Include\\pid.h"
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
#line 7 "Source\\pid.c"

uint8_t CW_HI_BootSpeed_Flag = 0;
uint8_t CW_LO_BootSpeed_Flag = 0;
uint8_t CCW_LO_BootSpeed_Flag = 0;
uint8_t CCW_HI_BootSpeed_Flag = 0;
int32_t SpdPIDCalc_HI(float NextPoint);
int32_t SpdPIDCalc_LO(float NextPoint);

extern unsigned char  T_1ms222;
void Motor_Control_cw_Task();
void Motor_Control_ccw_Task();












 



























 
typedef struct {
    int32_t SetPoint_q16;   
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




uint32_t oc_timestamp = 0;

void delayms(uint32_t ms)
{
    CLK_SysTickDelay(ms * 1000);
}














float rpm_buf[1] = {0};
float cur_buf[1] = {0};

float rpm_avg = 0, cur_avg = 0;
float rpm_prev_avg = 0, cur_prev_avg = 0;




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
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdbool.h"
 






 





#line 25 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdbool.h"



#line 161 "Source\\pid.c"

float current_buf[3] = {0};
uint8_t idx = 0;
float avg_current = 0;

void UpdateCurrent(float current_sample)
{
    int i;
    float sum = 0;
    current_buf[idx] = current_sample;
    idx = (idx + 1) % 3;
    sum = 0;
    for (i = 0; i < 3; i++)
        sum += current_buf[i];
    avg_current = sum / 3;
}


float speed_buf[3] = {0};
uint8_t speed_idx = 0;
float avg_speed = 0;


float speed_buf2[3] = {0};
uint8_t speed_idx2 = 0;
float avg_speed2 = 0;

void UpdateSpeed_CW(float speed_sample)
{
    float sum = 0;
    int i;
    speed_buf[speed_idx] = speed_sample;
    speed_idx = (speed_idx + 1) % 3;

    sum = 0;
    for (i = 0; i < 3; i++)
        sum += speed_buf[i];
    avg_speed = sum / 3;
}

void UpdateSpeed_CCW(float speed_sample)
{
    float sum = 0;
    int i;
    speed_buf2[speed_idx2] = speed_sample;
    speed_idx2 = (speed_idx2 + 1) % 3;

    sum = 0;
    for (i = 0; i < 3; i++)
        sum += speed_buf2[i];
    avg_speed2 = sum / 3;
}


#line 216 "Source\\pid.c"
#line 217 "Source\\pid.c"
#line 218 "Source\\pid.c"

uint32_t varcount = 0;
unsigned int Temp_var[3][200] = {1};











 
void PID_ParamInit_LO_CW(void)   
{
    
    sPID_LO_CW.LastError_q16 = 0;
    sPID_LO_CW.PrevError_q16 = 0;

    
    sPID_LO_CW.Kp_q16 = ((int32_t)((0.080f) * 65536.0f));
    sPID_LO_CW.Ki_q16 = ((int32_t)((0.05f) * 65536.0f));
    sPID_LO_CW.Kd_q16 = ((int32_t)((0.0f) * 65536.0f));

    
    sPID_LO_CW.SetPoint_q16 = ((int32_t)((1430.0f) * 65536.0f));
}











 
void PID_ParamInit_LO_CCW(void)   
{
    
    sPID_LO_CCW.LastError_q16 = 0;
    sPID_LO_CCW.PrevError_q16 = 0;

    
    sPID_LO_CCW.Kp_q16 = ((int32_t)((0.080f) * 65536.0f));
    sPID_LO_CCW.Ki_q16 = ((int32_t)((0.05f) * 65536.0f));
    sPID_LO_CCW.Kd_q16 = ((int32_t)((0.0f) * 65536.0f));

    
    sPID_LO_CCW.SetPoint_q16 = ((int32_t)((1430.0f) * 65536.0f));
}











 
void PID_ParamInit_HI_CW(void)   
{
    
    sPID_HI_CW.LastError_q16 = 0;
    sPID_HI_CW.PrevError_q16 = 0;

    
    sPID_HI_CW.Kp_q16 = ((int32_t)((0.18) * 65536.0f));
    sPID_HI_CW.Ki_q16 = ((int32_t)((0.08f) * 65536.0f));
    sPID_HI_CW.Kd_q16 = ((int32_t)((0.00f) * 65536.0f));

    
    sPID_HI_CW.SetPoint_q16 = ((int32_t)((2040.0f) * 65536.0f));
}











 

void PID_ParamInit_HI_CCW(void)   
{
    
    sPID_HI_CCW.LastError_q16 = 0;
    sPID_HI_CCW.PrevError_q16 = 0;

    
    sPID_HI_CCW.Kp_q16 = ((int32_t)((0.18) * 65536.0f));
    sPID_HI_CCW.Ki_q16 = ((int32_t)((0.08f) * 65536.0f));
    sPID_HI_CCW.Kd_q16 = ((int32_t)((0.00f) * 65536.0f));   

    
    sPID_HI_CCW.SetPoint_q16 = ((int32_t)((2040.0f) * 65536.0f));
}
















 
int32_t SpdPIDCalc_LO_CW_Q16(int32_t NextPoint_q16)
{
    int32_t error_q16;
    int64_t incpid_q32;

    
    error_q16 = sPID_LO_CW.SetPoint_q16 - NextPoint_q16;

    
    if (error_q16 < (30 << 16) && error_q16 > -(30 << 16))
        error_q16 = 0;

    
    incpid_q32 =
        (int64_t)sPID_LO_CW.Kp_q16 * error_q16
      - (int64_t)sPID_LO_CW.Ki_q16 * sPID_LO_CW.LastError_q16
      + (int64_t)sPID_LO_CW.Kd_q16 * sPID_LO_CW.PrevError_q16;

    
    incpid_q32 >>= 16;

    
    sPID_LO_CW.PrevError_q16 = sPID_LO_CW.LastError_q16;
    sPID_LO_CW.LastError_q16 = error_q16;

    
    return (int32_t)(incpid_q32>>16);
}

















 
int32_t SpdPIDCalc_LO_CCW_Q16(int32_t NextPoint_q16)
{
    int32_t error_q16;
    int64_t incpid_q32;

    
    error_q16 = sPID_LO_CCW.SetPoint_q16 - NextPoint_q16;

    
    if (error_q16 < (30 << 16) && error_q16 > -(30 << 16))
        error_q16 = 0;

    
    incpid_q32 =
        (int64_t)sPID_LO_CCW.Kp_q16 * error_q16
      - (int64_t)sPID_LO_CCW.Ki_q16 * sPID_LO_CCW.LastError_q16
      + (int64_t)sPID_LO_CCW.Kd_q16 * sPID_LO_CCW.PrevError_q16;

    
    incpid_q32 >>= 16;

    
    sPID_LO_CCW.PrevError_q16 = sPID_LO_CCW.LastError_q16;
    sPID_LO_CCW.LastError_q16 = error_q16;

    
     return (int32_t)(incpid_q32>>16);
}

















 
int32_t SpdPIDCalc_HI_CW_Q16(int32_t NextPoint_q16)
{
    int32_t error_q16;
    int64_t incpid_q32;

    
    error_q16 = sPID_HI_CW.SetPoint_q16 - NextPoint_q16;

    
    if (error_q16 < (30 << 16) && error_q16 > -(30 << 16))
        error_q16 = 0;

    
    incpid_q32 =
        (int64_t)sPID_HI_CW.Kp_q16 * error_q16
      - (int64_t)sPID_HI_CW.Ki_q16 * sPID_HI_CW.LastError_q16
      + (int64_t)sPID_HI_CW.Kd_q16 * sPID_HI_CW.PrevError_q16;

    
    incpid_q32 >>= 16;

    
    sPID_HI_CW.PrevError_q16 = sPID_HI_CW.LastError_q16;
    sPID_HI_CW.LastError_q16 = error_q16;
   
    
    return (int32_t)(incpid_q32>>16);
}
















 
int32_t SpdPIDCalc_HI_CCW_Q16(int32_t NextPoint_q16)
{
    int32_t error_q16;
    int64_t incpid_q32;

    
    error_q16 = sPID_HI_CCW.SetPoint_q16 - NextPoint_q16;

    
    if (error_q16 < (30 << 16) && error_q16 > -(30 << 16))
        error_q16 = 0;

    
    incpid_q32 =
        (int64_t)sPID_HI_CCW.Kp_q16 * error_q16
      - (int64_t)sPID_HI_CCW.Ki_q16 * sPID_HI_CCW.LastError_q16
      + (int64_t)sPID_HI_CCW.Kd_q16 * sPID_HI_CCW.PrevError_q16;

    
    incpid_q32 >>= 16;

    
    sPID_HI_CCW.PrevError_q16 = sPID_HI_CCW.LastError_q16;
    sPID_HI_CCW.LastError_q16 = error_q16;

    
    return (int32_t)(incpid_q32>>16);
}



void detect_enter_boot_speed_CW_HI(uint16_t rpm, uint16_t current) 
{
	
    if ((varcount > 5) && (((int)prev_rpm_cw - (int)G_ScrewRPM) > 40)) 
    {
        CW_HI_BootSpeed_Flag = 1; 
    }
    else if (G_Current > 500)
    {
        CW_HI_BootSpeed_Flag = 1;
    }
    else
    {
    }

    






 
    
 

    varcount++;

    if (CW_HI_BootSpeed_Flag == 1) 
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

void detect_enter_boot_speed_CW_LO(uint16_t rpm, uint16_t current) 
{

    uint8_t i;

    if ((varcount > 5) && (((int)prev_rpm_cw - (int)G_ScrewRPM) > 25)) 
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

    






 
    varcount++;

    if (CW_LO_BootSpeed_Flag == 1) 
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

void detect_enter_boot_speed_CCW_HI(uint16_t rpm, uint16_t current) 
{

    varcount++;

    if ((varcount == 10) && (G_ScrewRPM < 500))
    {
        CCW_HI_BootSpeed_Flag = 1;
    }

    if (CCW_HI_BootSpeed_Flag == 1) 
    {

        enter_boot_speed_mode = 1;
        G_PwmCMR = 2210;
    }

    prev_rpm_ccw = rpm;
    prev_current_ccw = current;
}
void detect_enter_boot_speed_CCW_LO(uint16_t rpm, uint16_t current) 
{

    varcount++;

    if ((varcount == 10) && (G_ScrewRPM < 600))
    {
        CCW_LO_BootSpeed_Flag = 1;
    }

    if (CCW_LO_BootSpeed_Flag == 1) 
    {

        enter_boot_speed_mode = 1;
        G_PwmCMR = 2210;
    }

    prev_rpm_ccw = rpm;
    prev_current_ccw = current;
}

void Exponential_PWM_Ramp_CCW(void) 
{

    (((WDT_T *) ((( uint32_t)0x40000000) + 0x4000))->WTCR = (((WDT_T *) ((( uint32_t)0x40000000) + 0x4000))->WTCR & ~((1ul << 3) | (1ul << 5) | (1ul << 2))) | (1ul << 0));

    G_PwmCMR = 2210;
}

extern uint32_t oc_timestamp;

void default_init(void) 
{
    int i, i2;
    
    
    









 

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

    



 
		
		sPID_HI_CW.SetPoint_q16 = ((int32_t)((2040.0f) * 65536.0f));
	  sPID_HI_CCW.SetPoint_q16 = ((int32_t)((2040.0f) * 65536.0f)); 
		sPID_LO_CW.SetPoint_q16 = ((int32_t)((1430.0f) * 65536.0f));
	  sPID_LO_CCW.SetPoint_q16 = ((int32_t)((1430.0f) * 65536.0f)); 
			 
    prev_rpm_cw = 0;
    prev_current_cw = 0;
    prev_rpm_ccw = 0;
    prev_current_ccw = 0;

    enter_boot_speed_mode = 0;

    
   
   

    G_StallCnt = 0;

    oc_timestamp = 0;
    G_PwmCMR = 0;

    PID_ParamInit_HI_CCW();
    PID_ParamInit_HI_CW();
    PID_ParamInit_LO_CCW();
    PID_ParamInit_LO_CW();
    





 
    
    
    G_ScrewRPM = 0;
    varcount = 0;
    prev_pwm_cw = 0;
    parm_init();
		
	
}

void uvw_check_function(void) 
{

    if (gu8StartSpeed == 1)
    {
        if (G_StallCnt > 10 && G_MotorStatus == eMotorStatus_Normal && HI_count_300ms < 20)
        {
            errorflag = eBehaving_Uvw_Error; 
            G_BuzzerLED = errorflag;
            Buzzer_LED_Behaving(&G_BuzzerLED);
        }
    }
    else
    {
        if (G_StallCnt > 10 && G_MotorStatus == eMotorStatus_Normal && LO_count_300ms < 20)
        {
            errorflag = eBehaving_Uvw_Error; 
            G_BuzzerLED = errorflag;
            Buzzer_LED_Behaving(&G_BuzzerLED);
        }
    }
}

void parm_init(void) 
{

    if (gu8StartSpeed == 1)
    {

        if (Enter_HI_Loop == 0) 
        {

            
            enter_boot_speed_mode = 0;
          

            sPID_HI_CW.PrevError_q16 = 0;
            sPID_HI_CW.LastError_q16 = 0;
            sPID_HI_CW.Kp_q16 = ((int32_t)((0.18) * 65536.0f));
            sPID_HI_CW.Ki_q16 = ((int32_t)((0.08f) * 65536.0f));
            iError_HI_CW = 0;
            iIncpid_HI_CW = 0;
            sPID_HI_CCW.PrevError_q16 = 0;
            sPID_HI_CCW.LastError_q16 = 0;
            sPID_HI_CCW.Kp_q16 = ((int32_t)((0.18) * 65536.0f));
            sPID_HI_CCW.Ki_q16 = ((int32_t)((0.08f) * 65536.0f));
            iError_HI_CCW = 0;
            iIncpid_HI_CCW = 0;
            HI_count_300ms = 0;

            T_1ms222 = 0;

        } 
    }
    else
    {
        if (Enter_LO_Loop == 0) 
        {

            
            enter_boot_speed_mode = 0;
        

            sPID_LO_CW.PrevError_q16 = 0;
            sPID_LO_CW.LastError_q16 = 0;
            sPID_LO_CW.Kp_q16 = ((int32_t)((0.080f) * 65536.0f));
            sPID_LO_CW.Ki_q16 = ((int32_t)((0.05f) * 65536.0f));
            iError_LO_CW = 0;
            iIncpid_LO_CW = 0;
            sPID_LO_CCW.PrevError_q16 = 0;
            sPID_LO_CCW.LastError_q16 = 0;
            sPID_LO_CCW.Kp_q16 = ((int32_t)((0.080f) * 65536.0f));
            sPID_LO_CCW.Ki_q16 = ((int32_t)((0.05f) * 65536.0f));
            iError_LO_CCW = 0;
            iIncpid_LO_CCW = 0;
            sPID_LO_CW.SetPoint_q16 = ((int32_t)((1430.0f) * 65536.0f));
            LO_count_300ms = 0;

            T_1ms222 = 0;
        }
    }
}

int32_t pwm_inc;      
int32_t pwm_next;     
unsigned int pwm_clamp_to_uint(int32_t val)
{
    if (val < (0)) return (0);
    if (val > 2210) return 2210;
    return (unsigned int)val;
}
void motor_algorithm_HI(void) 
{

    if (T_1ms222 >= 3)
    {
        T_1ms222 = 0;
				
        HI_Loop++;
        if (G_Direction == 0) 
        {
            if (HI_Loop == 1)
            {
                
 temppwm222 = G_PwmCMR;

 
temppwm = SpdPIDCalc_HI_CW_Q16(((int32_t)((G_ScrewRPM) * 65536.0f)));
							
							G_PwmCMR=pwm_clamp_to_uint(temppwm + temppwm222);
							





 

            }
            else if (HI_Loop == 3)
            {
                
                
							 detect_enter_boot_speed_CW_HI(G_ScrewRPM, G_Current); 
				if(CW_HI_BootSpeed_Flag==0)
                {
                  
                   
                   
                     
                }
                
                 else
                
                {}
                             HI_Loop = 0;  
            
                
                                                                      
								
                
            }
           


 
            else
            {
            }
        }
        else 
        {
            if (HI_Loop == 1)
            {
                              
  temppwm222 = G_PwmCMR;

 
temppwm = SpdPIDCalc_HI_CCW_Q16(((int32_t)((G_ScrewRPM) * 65536.0f)));
	G_PwmCMR=pwm_clamp_to_uint(temppwm + temppwm222);
            }
            else if (HI_Loop == 3)
            {
							 detect_enter_boot_speed_CCW_HI(G_ScrewRPM, G_Current); 
                if (enter_boot_speed_mode == 0)
                {
                   
                    
                    HI_Loop = 0;
                }
                else
                {
                    
                }
                
                
            }
           


 
            else
            {
            }
        }
    }
}

void motor_algorithm_LO(void) 
{

    if (T_1ms222 >= 3) 
    {
        T_1ms222 = 0;
		 
        LO_Loop++;
        if (G_Direction == 0) 
        {
            if (LO_Loop == 1)
            {
                              
  temppwm222 = G_PwmCMR;

 
temppwm = SpdPIDCalc_LO_CW_Q16(((int32_t)((G_ScrewRPM) * 65536.0f)));

 	G_PwmCMR=pwm_clamp_to_uint(temppwm + temppwm222);

            }
            else if (LO_Loop == 3)
            {
							 detect_enter_boot_speed_CW_LO(G_ScrewRPM, G_Current); 
               










 
                
                
                 LO_Loop = 0;
            }
           




 
            else
            {
            }
        }
        else 
        {
            if (LO_Loop == 1) 
            {
                                             
  temppwm222 = G_PwmCMR;

 
temppwm = SpdPIDCalc_LO_CCW_Q16(((int32_t)((G_ScrewRPM) * 65536.0f)));
 	G_PwmCMR=pwm_clamp_to_uint(temppwm + temppwm222);

            }
            else if (LO_Loop == 3)
            {
							  detect_enter_boot_speed_CCW_LO(G_ScrewRPM, G_Current); 
                if (enter_boot_speed_mode == 0)
                {
                  
                                                                 
                    LO_Loop = 0;
                }
                else
                {
                    
                }

                
                

               
            }
          


 
            else
            {
            }
        }
    }
}
