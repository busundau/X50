/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2009 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef TBK_Motor
#define TBK_Motor

 void Motor(unsigned char Enable_Motor,unsigned char Direction);
 void SetPWMCMR(unsigned int PwmCMR);
 void MotorStart(unsigned char Enable_Motor,unsigned char Direction);

#endif
 
 
 
