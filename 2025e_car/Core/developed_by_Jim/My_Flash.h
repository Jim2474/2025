#ifndef __MY_FLASH_H
#define __MY_FLASH_H

#include "board.h"
#define STRING_STORAGE_ADDR  ((uint32_t)0x08010010)
static uint32_t STMFLASH_ReadWord(uint32_t faddr);
uint8_t STMFLASH_GetFlashSector(uint32_t addr);
void WriteFlashData(uint32_t WriteAddress, uint8_t *data, uint32_t length);
uint8_t SaveNumber(uint32_t addr, uint32_t number);
uint32_t ReadNumber(uint32_t addr);
uint8_t SaveNumberArray(uint32_t addr, uint32_t *numbers, uint8_t count);
void ReadNumberArray(uint32_t addr, uint32_t *numbers, uint8_t count);
uint8_t SaveNumberArray(uint32_t addr, uint32_t *numbers, uint8_t count);
void ReadNumberArray(uint32_t addr, uint32_t *numbers, uint8_t count);
void SaveStringToFlash(void);
void ReadStringFromFlash(char* buffer, uint32_t max_len);


#endif

