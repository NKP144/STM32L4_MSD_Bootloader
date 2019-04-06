/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flash_work.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CopyAppToUserMemory(void);
void JumpToMainProgram(void);
void PeriphDeinit(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char str[80] = {0};
FRESULT FatFs_Status = FR_OK;
FSIZE_t appSize = 0;
uint8_t AppDataByte[512] = {0};
UINT readBytes = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
int i=0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  sprintf (str, "-----START LOG----- \n\r\n\r");
  HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);

  sprintf (str, "FLASH_START_ADDR: %#x \n\r", FLASH_START_ADDR);
  HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);
  sprintf (str, "FLASH_MAX_SIZE: %dKByte \n\r", FLASH_MAX_SIZE/1024);
  HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);
  sprintf (str, "BOOT_START_ADDR: %#x \n\r", FLASH_BOOT_START_ADDR);
  HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);
  sprintf (str, "BOOT_MEM_SIZE: %dKByte \n\r", FLASH_BOOT_SIZE/1024);
  HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);
  sprintf (str, "USER_START_ADDR: %#x \n\r", FLASH_USER_START_ADDR);
  HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);
  sprintf (str, "USER_MEM_SIZE: %dKByte \n\r", FLASH_USER_SIZE/1024);
  HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);
  sprintf (str, "FLASH_MSD_START_ADDR: %#x \n\r", FLASH_MSD_START_ADDR);
  HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);
  sprintf (str, "MSD_MEM_SIZE: %dKByte \n\r", FLASH_MSD_SIZE/1024);
  HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);
  sprintf (str, "OTHER_START_ADDR: %#x \n\r", FLASH_OTHER_START_ADDR);
  HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);
  sprintf (str, "OTHER_MEM_SIZE: %dKByte \n\r\n\r", FLASH_OTHER_SIZE/1024);
  HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

/*---------------------- Mass storage device Mode----------------------- */
	  	  	if (HAL_GPIO_ReadPin(B1_GPIO_Port,B1_Pin) == GPIO_PIN_SET)  //Кнопка нажата, режим MSD
	  	  	{
	  	  		printf ("MSD Mode... \n");
	  	     	HAL_UART_Transmit (&huart2, (uint8_t*)"MSD Mode... \n\r", strlen("MSD Mode... \n\r"), 1000);

	  	  		MX_USB_DEVICE_Init();
	  	  		while (1);
	  	  	}

/*-----------------------Bootloader Mode---------------------------------*/
   sprintf (str, "BOOTLOADER Mode... \n\r");
   HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);

   MX_FATFS_Init(); // Init FatFs

   FatFs_Status = f_mount(&USERFatFS, (TCHAR const*)USERPath, 0);
   if(FatFs_Status == FR_OK)
   {
	   sprintf (str, "FatFs mount status = %s \n\r", "FR_OK");
	   HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);

	   FatFs_Status = f_open(&USERFile, "APP.bin", FA_READ);
	   if(FatFs_Status == FR_OK)
	   {
		   sprintf (str, "Application file open status = %s \n\r", "FR_OK");
		   HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);

		   appSize = f_size(&USERFile);

		   sprintf (str, "Application file size = %lu \n\r", appSize);
		   HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);

		   for(i = 0; i < appSize; i++) //Byte-to-byte compare files in MSD_MEM and USER_MEM
		   {
			   f_read(&USERFile, AppDataByte, 1, &readBytes);
			   if(*((volatile uint8_t*)(FLASH_USER_START_ADDR + i)) != AppDataByte[0]) break;
		   }
		   if (i != appSize) //=> was done "break" instruction in for(;;) cycle => new firmware in MSD_FLASH
		   {
			i=0;
			sprintf (str, "Find difference in  %d byte \n\r\n\r", i);
			HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);

		    CopyAppToUserMemory();
		   }
		   else
		   {
			sprintf (str, "Versions of firmwares are the same. No update required.\n\r");
			HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);
		   }

		   FatFs_Status = f_close(&USERFile);
		   if  (FatFs_Status == FR_OK)
		   {
			   sprintf (str, "Application file close status = %s \n\r", "FR_OK");
			   HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);
		   }
		   else
		   {
			   sprintf (str, "Application file close status = %s \n\r", "ERROR");
			   HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);
		   }

		   FatFs_Status = f_mount(0, (TCHAR const*)USERPath, 0);
		   if  (FatFs_Status == FR_OK)
		   {
			   sprintf (str, "FatFs unmount status = %s \n\r\n\r", "FR_OK");
			   HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);
		   }
		   else
		   {
			   sprintf (str, "FatFs unmount status = %s \n\r\n\r", "ERROR");
			   HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);
		   }

	   }
	   else
	   {
		   sprintf (str, "Application file open status = %s \n\r", "ERROR");
		   HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);
		   FatFs_Status = f_mount(0, (TCHAR const*)USERPath, 0);
	   }
   }
   else
   {
	   sprintf (str, "FatFs mount status = %s \n\r", "ERROR");
	   HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);
   }

	sprintf (str, "Peripheral deinit and go to main programm \n\r\n\r");
	HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);
	PeriphDeinit();      //Periphery deinit

	JumpToMainProgram(); //Go to main programm*/

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */

void JumpToMainProgram(void)
{
	uint32_t appJumpAddress=0;
	void (*GoToApp)(void);

	appJumpAddress = *(__IO uint32_t*) (FLASH_USER_START_ADDR + 4);
	GoToApp = (void (*)(void))appJumpAddress;
	SCB->VTOR = FLASH_USER_START_ADDR;
	__set_MSP(*((volatile uint32_t*) FLASH_USER_START_ADDR)); //stack pointer (to RAM) for USER app in this address
	GoToApp();

	//return 0;
}
//---------------------------------------------------------------------
void PeriphDeinit(void)
{
	//Выключить прерывания, которые были использованы или все	__disable_irq();
	FATFS_UnLinkDriver(USERPath);

	__HAL_RCC_GPIOC_CLK_DISABLE();
	__HAL_RCC_GPIOA_CLK_DISABLE();
	__HAL_RCC_GPIOB_CLK_DISABLE();
	HAL_GPIO_DeInit(B1_GPIO_Port,B1_Pin);
	HAL_GPIO_DeInit(LD4_GPIO_Port,LD4_Pin);

	HAL_UART_MspDeInit(&huart2);

	//return 0;
}

void CopyAppToUserMemory(void)
{
	uint32_t remain_bytes = 0;
	uint32_t number_of_blok = 0;
	float write_percents = 0;

	sprintf (str, "Start to copy new firmware \n\r");
	HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);

	f_lseek(&USERFile, 0);       // Go to the fist position of file
	appSize = f_size(&USERFile); // Bytes in file

	number_of_blok = appSize/512;
	remain_bytes =  appSize%512; // Количество оставшихся байт до 512
	if (remain_bytes != 0) number_of_blok++;
	sprintf (str, "Total cycles number = %lu. Remain bytes = %lu \n\r",number_of_blok, remain_bytes);
	HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);

	for (int j=0; j<number_of_blok; j++)
	{
	  f_read(&USERFile, AppDataByte, 512, &readBytes); // Read 512 byte from file
	  sprintf (str, "%d Cycle. Read %u bytes \n\r",j ,readBytes);
	  HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);

	  if (readBytes != 512)  // Дополнить буффер 0xFF до 512 байт
	  {
	    for (int k=0; k<(512-readBytes); k++)
	    {
	    	AppDataByte[readBytes+k] = 0xFF;
	    }
	  }

	  flash_512block_write(FLASH_USER_START_ADDR, j, AppDataByte);

	  write_percents = (((float)j+1)/number_of_blok)*100;
	  sprintf (str, "Write percents = %0.2f %% \n\r",write_percents);
	  HAL_UART_Transmit (&huart2, (uint8_t*)str, strlen(str), 1000);
	}

}
//-------------------------------------------------------------------------------------------------------------------


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
