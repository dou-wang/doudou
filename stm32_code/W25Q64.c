#include "W25Q64.h"

void W25Q64_Init(void)
{
	SPI_Init();
}

void W25Q64_ReadID(uint8_t *MID,uint16_t *DID)
{
	SPI_Start();
	SPI_Swap_byte(W25Q64_JEDEC_ID);
	*MID = SPI_Swap_byte(W25Q64_DUMMY_BYTE);
	
	*DID = SPI_Swap_byte(W25Q64_DUMMY_BYTE);		
	*DID <<= 8;				//高8位
	
	*DID |= SPI_Swap_byte(W25Q64_DUMMY_BYTE);	//低8位
	
	SPI_Stop();
}

void W25Q64_ENABLE(void)
{
	SPI_Start();
	SPI_Swap_byte(W25Q64_WRITE_ENABLE);
	SPI_Stop();
}

void W25Q64_WaitBusy(void)
{
	uint32_t Timeout;
	SPI_Start();
	SPI_Swap_byte(W25Q64_READ_STATUS_REGISTER_1);
	Timeout = 100000;
	while((SPI_Swap_byte(W25Q64_DUMMY_BYTE) & 0x01) == 0x01)
	{
		Timeout--;
		if(Timeout == 0)
		{
			break;
		}
	}
	SPI_Stop();
}


void W25Q64_Page(uint32_t Addr,uint8_t *DataArr,uint16_t Count)		//不能跨页写入
{
	W25Q64_ENABLE();
	uint16_t i;
	SPI_Start();
	SPI_Swap_byte(W25Q64_PAGE_PROGRAM);
	SPI_Swap_byte(Addr >> 16);
	SPI_Swap_byte(Addr >> 8);
	SPI_Swap_byte(Addr);
	for(i = 0;i < Count;i++)
	{
		SPI_Swap_byte(DataArr[i]);
	}
	SPI_Stop();
	W25Q64_WaitBusy();
}

void W25Q64_SectorErase(uint32_t Addr)
{
	W25Q64_ENABLE();
	SPI_Start();
	SPI_Swap_byte(W25Q64_SECTOR_ERASE_4KB);
	SPI_Swap_byte(Addr >> 16);
	SPI_Swap_byte(Addr >> 8);
	SPI_Swap_byte(Addr);
	SPI_Stop();
	W25Q64_WaitBusy();
}

void W25Q64_ReadData(uint32_t Addr,uint8_t *DataArr,uint16_t Count)
{
	uint32_t i;
	SPI_Start();
	SPI_Swap_byte(W25Q64_READ_DATA);
	SPI_Swap_byte(Addr >> 16);
	SPI_Swap_byte(Addr >> 8);
	SPI_Swap_byte(Addr);
	for(i = 0;i < Count;i ++)
	{
		DataArr[i] = SPI_Swap_byte(W25Q64_DUMMY_BYTE);
	}
	SPI_Stop();
}
