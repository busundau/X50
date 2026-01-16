
#include <stdio.h>
#include "M051Series.h"
#include "string.h"
#include "PtsPta_Global.h"
/*
0x0001 FFFF
0x0001 FE00

0x0001 FDFF
0x0001 FC00

0x0001 FBFF
0x0001 FA00

0x0001 F9FF
0x0001 F800

0x0001 F7FF
0x0001 F600

0x0001 F5FF
0x0001 F400

0x0001 F3FF
0x0001 F200

0x0001 F1FF
0x0001 F000
*/
unsigned int FlashData[512];

void EraseFlash(unsigned int u32StartAddr) // 1 page = 512 byte 0x1f000~ 0x1f1ff
{
	SYS_UnlockReg();
 	//FMC_Open();
    	FMC->ISPCON |=  FMC_ISPCON_ISPEN_Msk;
      FMC_Erase(u32StartAddr);
    	SYS_LockReg();
}



unsigned int ReadFlashData(unsigned int u32StartAddr)
{
	unsigned int u32RData;
	SYS_UnlockReg();
 	//FMC_Open();
    	FMC->ISPCON |=  FMC_ISPCON_ISPEN_Msk;
      u32RData = FMC_Read(u32StartAddr);
    	SYS_LockReg();
	return u32RData;
}

void SaveData2Flash(unsigned int  u32Addr, unsigned int u32Data)
{

	

      EraseFlash(u32Addr);

	SYS_UnlockReg();
 	//FMC_Open();
    	FMC->ISPCON |=  FMC_ISPCON_ISPEN_Msk;

 //      FMC_Erase(u32Addr);
       FMC_Write(u32Addr, u32Data);

    	SYS_LockReg();

}




