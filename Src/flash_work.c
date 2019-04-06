/*
 * flash_work.c
 * Work with flash memory controller.
 * Record of 512 bytes in one page with saving and restoring unmodifiable information.
 *  Created on: 16 мар. 2019 г.
 *      Author: NKP
 */

#include "flash_work.h"


uint64_t RAM_BUF[256];   // uint64_t=8байт 8x256=2048


void flash_512block_write (uint32_t BASE_ADDR, uint32_t blk_num, uint8_t *data_buf)  // Запись во флеш блока 512 байт
{
 /*
  * @param  blk_num:   Logical block unit number
  * @param  *data_buf: data
 */

/*Variable used for Erase procedure*/
FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t Address = 0, StartAddress = 0, PAGEError = 0, FirstPage = 0;
uint64_t DATA_64=0;

/*Определяем страницу, которую необходимо будет стереть
В одной странице 4 блока*/
StartAddress = BASE_ADDR + (blk_num / 4) * FLASH_PAGE_SIZE; // Номер первого блока = 0

//Сохраняем содержимое страницы в массив
for (int i = 0; i < 256; i++)
{
	RAM_BUF[i] = *((volatile uint64_t*)(StartAddress + i*8));
}

//Поместить в массив новые данные
for (int k = 0; k < 64; k++)
{
 for (int i = 0; i < 8; i++)  //We form data for record
 {
 	DATA_64 |= data_buf[(7 - i) + k * 8];
	if (i != 7) DATA_64 <<= 8;
 }
 RAM_BUF[k + (blk_num % 4) * 64] = DATA_64; //Записываем данные блока в массив данных всей страницы (в странице 4 блока, надо записать по правильному адресу)
 DATA_64 = 0;
}


/*Erase flash*/
/* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

/*Scratch the number of the first page to erase*/
FirstPage = (StartAddress - FLASH_START_ADDR) / FLASH_PAGE_SIZE;

/* Clear OPTVERR bit set on virgin samples */
 __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

 /* Fill EraseInit structure*/
 EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
 EraseInitStruct.Banks       = FLASH_BANK_1;
 EraseInitStruct.Page        = FirstPage;
 EraseInitStruct.NbPages     = 1;

 /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
    you have to make sure that these data are rewritten before they are accessed during code
    execution. If this cannot be done safely, it is recommended to flush the caches by setting the
    DCRST and ICRST bits in the FLASH_CR register. */
 if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
 {
   /*
     Error occurred while page erase.
     User can add here some code to deal with this error.
     PAGEError will contain the faulty page and then to know the code error on this page,
     user can call function 'HAL_FLASH_GetError()'
   */
 }

// Write data to flash page
Address =  StartAddress;
for (int i = 0; i < 256; i++)
{
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, RAM_BUF[i]);
	Address += 8;
}

HAL_FLASH_Lock();

}


