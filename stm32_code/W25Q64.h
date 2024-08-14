#ifndef __W25Q64_H
#define __W25Q64_H

#include "mytask.h"


void W25Q64_Init(void);

void W25Q64_ReadID(uint8_t *MID,uint16_t *DID);
void W25Q64_ENABLE(void);
void W25Q64_WaitBusy(void);
void W25Q64_Page(uint32_t Addr,uint8_t *DataArr,uint16_t Count);
void W25Q64_SectorErase(uint32_t Addr);
void W25Q64_ReadData(uint32_t Addr,uint8_t *DataArr,uint16_t Count);
#endif

