/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2009 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef TBK_NVRAM
#define TBK_NVRAM


void Write_24LC02( unsigned char address, unsigned char data, unsigned char *EEPROM_STATUS );
unsigned char Read_24LC02( unsigned char  address, unsigned char *EEPROM_STATUS );
unsigned char CHECK_ID(void);
uint8_t CheckBatteryType(uint8_t DevAddr);

#if 0
void Write_24LC64( unsigned int address, unsigned char data, unsigned char *EEPROM_STATUS );
unsigned char  Read_24LC64( unsigned int address, unsigned char *EEPROM_STATUS );

unsigned char CheckVoltage_CurrentAndNVRAM(void);
unsigned char CheckEepromType(void);
#endif
#endif
 
 
 
