
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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include "lcd.h"
#include "BMP280.h"
#include "ringbuffer_dma.h"
#include <string.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* Current pressure at sea level in hPa taken from http://www.aviador.es/Weather/Meteogram/UKKK */

#define QNH 1020
/* Variables for all measurements */
double temp, press, alt;
int8_t com_rslt;

extern DMA_HandleTypeDef hdma_uart4_rx;
/* Ringbuffer for Rx messages */
RingBuffer_DMA rx_buf;
/* Array for DMA to save Rx bytes */
#define BUF_SIZE 256
uint8_t rx[BUF_SIZE];
uint32_t rx_count = 0;
/* Array for received commands */
char cmd[128];
uint8_t icmd = 0;
/* Array for Tx messages */
uint8_t tx[100];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void Process_Command(char * command);

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
  MX_DMA_Init();
  MX_UART4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  LCD_Init();

  /* Start BMP280 and change settings */
  LCD_Printf("Connecting to BMP280...\n");
  bmp280_t bmp280 = {.i2c_handle = &hi2c1};
  com_rslt = BMP280_init(&bmp280);
  com_rslt += BMP280_set_power_mode(BMP280_NORMAL_MODE);
  com_rslt += BMP280_set_work_mode(BMP280_STANDARD_RESOLUTION_MODE);
  com_rslt += BMP280_set_standby_durn(BMP280_STANDBY_TIME_1_MS);
  if (com_rslt != SUCCESS) {
  	LCD_Printf("Check BMP280 connection!\nProgram terminated");
  	return 0;
  }
  LCD_Printf("Connection successful!\n");

  /* Init RingBuffer_DMA object */
  RingBuffer_DMA_Init(&rx_buf, &hdma_uart4_rx, rx, BUF_SIZE);
  /* Start UART4 DMA Reception */
  HAL_UART_Receive_DMA(&huart4, rx, BUF_SIZE);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */


     /* Check number of bytes in RingBuffer */
     rx_count = RingBuffer_DMA_Count(&rx_buf);

     /* Process each byte individually */
     while (rx_count--)
     {
	    /* Read out one byte from RingBuffer */

	    uint8_t b = RingBuffer_DMA_GetByte(&rx_buf);
	    if (b == '\r' || b == '\n') { /* If \r or \n process command */
	  		/* Terminate string with \0 */
	  				cmd[icmd] = 0;
	  				icmd = 0;
	  				/* Process command */
	  				LCD_Printf(">> %s\n", cmd);
	  				Process_Command(cmd);
	  			} else { /* If regular character, put it into cmd[] */
	  				cmd[icmd++] = b;
	  			}
      }

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
void Process_Command(char * command) {
	if (strcmp(command, "bmp280 temp") == 0) {
		/* Read temperature */
		BMP280_read_temperature_double(&temp);
		sprintf((char *) tx, "Temp : %6.2f C\r\n", temp);
	} else if (strcmp(command, "bmp280 press") == 0) {
		/* Read pressure */
		BMP280_read_pressure_double(&press);
		sprintf((char *) tx, "Press: %6.0f Pa\r\n", press);
	} else if (strcmp(command, "bmp280 alt") == 0) {
		/* Calculate current altitude, based on current QNH pressure */
		sprintf((char *) tx, "Alt  : %3.0f m\r\n", BMP280_calculate_altitude(QNH * 100));
	} else if (strcmp(command, "bmp280 all") == 0) {
		/* Read temperature and pressure */
		BMP280_read_temperature_double(&temp);
		BMP280_read_pressure_double(&press);
		/* Calculate current altitude, based on current QNH pressure */
		alt = BMP280_calculate_altitude(QNH * 100);
		sprintf((char *) tx, "Temp : %6.2f C\nPress: %6.0f Pa\nAlt  : %3.0f m\r\n", temp, press, alt);
	} else {
		sprintf((char *) tx, "command unknown\r\n");
	}
	HAL_UART_Transmit(&huart4, tx, strlen((char *) tx), 100);
	LCD_Printf("<< %s", tx);
}

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
