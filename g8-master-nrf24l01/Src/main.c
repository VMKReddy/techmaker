
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include "lcd.h"
#include "nRF24L01.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* Channel */
#define MY_CHANNEL 106
//#define MASTER
/* My address */
#ifdef MASTER
/* My address */
uint8_t MyAddress[] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };
/* Other end address */
uint8_t TxAddress[] = { 0x7E, 0x7E, 0x7E, 0x7E, 0x7E };
#else
/* My address */
uint8_t MyAddress[] = { 0x7E, 0x7E, 0x7E, 0x7E, 0x7E };
/* Other end address */
uint8_t TxAddress[] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };
#endif

/* Data received and data for send */
uint8_t dataOut[32], dataIn[32];
/* NRF transmission status */
NRF24L01_Transmit_Status_t transmissionStatus;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();

  NRF24L01_Init(&hspi3, MY_CHANNEL, 32);
  /* Set 250kBps data rate and -6dBm output power */
  NRF24L01_SetRF(NRF24L01_DataRate_250k, NRF24L01_OutputPower_M6dBm);

  /* Set my address, 5 bytes */
  NRF24L01_SetMyAddress(MyAddress);
  /* Set TX address, 5 bytes */
  NRF24L01_SetTxAddress(TxAddress);
  /* Time variables & received errors counter */

#ifdef MASTER
	uint32_t sendTime = HAL_GetTick();
	uint8_t errors = 0;
#endif
	uint32_t lastTime = HAL_GetTick();
	int16_t i = 0;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifdef MASTER
		/* Every 2 seconds */
		if (HAL_GetTick() - lastTime > 2000) {

			/* Fill data with something */
			sprintf((char *) dataOut, "Good news everyone! #%d", i++);
			LCD_Printf("Sending data: \n");
			LCD_Printf("%s\n", dataOut);
			/* Transmit data, goes automatically to TX mode */
			NRF24L01_Transmit(dataOut);
			/* Wait for data to be sent */
			do {
				/* Get transmission status */
				transmissionStatus = NRF24L01_GetTransmissionStatus();
			} while (transmissionStatus == NRF24L01_Transmit_Status_Sending);
			sendTime = HAL_GetTick();

			/* Go back to RX mode */
			NRF24L01_PowerUpRx();
			/* Wait received data, wait max 100ms, if time is larger, then data were probably lost */
			while (!NRF24L01_DataReady() && (HAL_GetTick() - sendTime) < 100);

			/* Show ping time */
			LCD_Printf("%d ms\n", HAL_GetTick() - sendTime);
			LCD_Printf("Receiving back: \n");
			/* Get data from NRF2L01+ */
			NRF24L01_GetData(dataIn);
			LCD_Printf("%s\n", dataIn);
			/* Check transmit status */
			LCD_Printf("Status: ");
			if (transmissionStatus == NRF24L01_Transmit_Status_Ok) {
				/* Transmit went OK */
				LCD_Printf("OK\n");
			} else if (transmissionStatus == NRF24L01_Transmit_Status_Lost) {
				/* Message was LOST */
				LCD_Printf("LOST\n");
			} else {
				/* This should never happen */
				LCD_Printf("SENDING\n");
			}

			errors = 0;
			for (int k = 0; k < sizeof(dataIn) / sizeof(dataIn[0]); k++) {
				errors += (dataIn[k]!=dataOut[k]);
			}
			LCD_Printf("Errors: %d\n", errors);
			LCD_Printf("\n");
			lastTime = HAL_GetTick();
		}
#else
		/* If data is ready on NRF24L01+ */
		if (NRF24L01_DataReady()) {
			LCD_Printf("\n");
			/* Get data from NRF24L01+ */
			NRF24L01_GetData(dataIn);

			LCD_Printf("Data received:\n");
			LCD_Printf("%s\n", dataIn);

			/* Send it back, automatically goes to TX mode */
			LCD_Printf("Sending it back\n");
			NRF24L01_Transmit(dataIn);

			/* Wait for data to be sent */
			do {
				/* Wait till sending */
				transmissionStatus = NRF24L01_GetTransmissionStatus();
			} while (transmissionStatus == NRF24L01_Transmit_Status_Sending);

			/* Send done */
			/* Check transmit status */
			LCD_Printf("Status: ");
			if (transmissionStatus == NRF24L01_Transmit_Status_Ok) {
				/* Transmit went OK */
				LCD_Printf("OK\n");
			} else {
				/* Message was LOST */
				LCD_Printf("ERROR\n");
			}

			/* Go back to RX mode */
			NRF24L01_PowerUpRx();
			i = 0;
		} else {
			if (HAL_GetTick() - lastTime > 250) {
			    if (i == 0) {
			        LCD_Printf("Waiting for data");
			        i++;
			    } else if (i > 23) {
			        LCD_Printf("\r");
			        i = 0;
			    } else {
			        LCD_Printf(".");
			        i++;
			    }
			    lastTime = HAL_GetTick();
			}
		}
#endif


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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
