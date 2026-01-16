/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2009 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef TBK_FMC
#define TBK_FMC

void EraseFlash(unsigned int u32StartAddr); // 1 page = 512 byte 0x1f000~ 0x1f1ff
unsigned int ReadFlashData(unsigned int u32StartAddr);
void SaveData2Flash(unsigned int  u32Addr, unsigned int u32Data);

#endif
 
 
 
