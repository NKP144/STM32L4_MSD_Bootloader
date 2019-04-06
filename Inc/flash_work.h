/*
 * flash_work.h
 *
 *  Created on: 16 мар. 2019 г.
 *      Author: NKP
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FLASH_WORK_H_
#define FLASH_WORK_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32l4xx_hal.h"

/* Private defines -----------------------------------------------------------*/

//#define FLASH_PAGE_SIZE	   2048 					         	             //2 Kbyte per page
#define FLASH_START_ADDR	   0x08000000					                     //Origin
#define FLASH_MAX_SIZE		   0x00080000					                     //Max FLASH size - 512 Kbyte
#define FLASH_END_ADDR		   (FLASH_START_ADDR + FLASH_MAX_SIZE)	             //FLASH end address
#define FLASH_BOOT_START_ADDR  (FLASH_START_ADDR)				                 //Bootloader start address
#define FLASH_BOOT_SIZE		   0x00010000				   	                     //64 Kbyte for bootloader
#define FLASH_USER_START_ADDR  (FLASH_BOOT_START_ADDR + FLASH_BOOT_SIZE)         //User application start address
#define FLASH_USER_SIZE		   0x00032000					                     //200 Kbyte for user application
#define FLASH_MSD_START_ADDR   (FLASH_USER_START_ADDR + FLASH_USER_SIZE)         //USB MSD start address
#define FLASH_MSD_SIZE	       0x00032000					                     //200 Kbyte for USB MASS Storage
#define FLASH_OTHER_START_ADDR (FLASH_MSD_START_ADDR + FLASH_MSD_SIZE)		     //Other free memory start address
#define FLASH_OTHER_SIZE	   (FLASH_END_ADDR - FLASH_OTHER_START_ADDR) 		 //Free memory size

/* Exported variables----------- ---------------------------------------------*/

extern uint64_t RAM_BUF[];

/* Exported functions prototypes ---------------------------------------------*/

void flash_512block_write (uint32_t BASE_ADDR, uint32_t blk_num, uint8_t *data_buf);  // Запись во флеш блока 512 байт


#ifdef __cplusplus
}
#endif

#endif /* FLASH_WORK_H_ */
