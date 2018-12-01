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
#include "stdarg.h"
#include "string.h"

/* USER CODE BEGIN Includes */
volatile uint8_t BUFF_SIZE = 255;

volatile char Buff_Rx[255];
volatile char Buff_Tx[255];

volatile uint8_t Busy_Tx = 0;
volatile uint8_t Empty_Tx = 0;
volatile uint8_t Busy_Rx = 0;
volatile uint8_t Empty_Rx = 0;

char Bx[255];

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint16_t licznik = 0;
uint16_t okres = 5;
uint16_t temp_zadana = 60;
uint16_t petla_histerezy = 3;
uint16_t czy__ma_byc_wlaczony_nadmuch = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void HAL_SYSTICK_Callback() {
	licznik++;
	if (licznik > okres) {
		licznik = 0;
		petlaGlowna();

	}
}
int16_t odczyt_temp() {
	int16_t temp = 20; //zaimplentowac ds18 b20
	return temp;

}
void wlacz_dmuchawe(uint8_t czyWlaczona) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, czyWlaczona);
}

void petlaGlowna() {
	if (czy__ma_byc_wlaczony_nadmuch == 1) {

		if (odczyt_temp() > (temp_zadana + petla_histerezy)) {
			wlacz_dmuchawe(0);
		} else if (odczyt_temp() < (temp_zadana - petla_histerezy)) {
			wlacz_dmuchawe(1);
		}
	} else {
		wlacz_dmuchawe(0);
	}
}

void odbior_komunikatu(char* komunikat){
	if(strcmp("AT+TEMA?", komunikat) == 0){
		//odczyt pamiêci
		//wys³anie wartoœci
		odczyt_temp();
	}
	if(strcmp("AT+TEMZ=", komunikat) == 0){
		//odczyt wartoœci z komunikatu
		//zapis do pamiêci
		//wys³anie ok
	}
	if(strcmp("AT+PEHI=", komunikat) == 0){

	}
	if(strcmp("AT+NAON=", komunikat) == 0){

	}
	if(strcmp("AT+NOFF=", komunikat) == 0){

	}
}

void USART_fsend(char* format, ...) {

	char tmp_rs[128];
	int i;
	__IO int idx;
	va_list valist;
	va_start(valist, format);
	vsprintf(tmp_rs, format, valist);
	va_end(valist);
	idx = Empty_Tx;
	for (i = 0; i < strlen(tmp_rs); i++) {
		Buff_Tx[idx] = tmp_rs[i];
		idx++;
		if (idx >= BUFF_SIZE)
			idx = 0;
	}
	__disable_irq();

	if ((Empty_Tx == Busy_Tx)
			&& (__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TXE) == SET)) {

		Empty_Tx = idx;
		uint8_t tmp = Buff_Tx[Busy_Tx];
		Busy_Tx++;
		if (Busy_Tx >= BUFF_SIZE)
			Busy_Tx = 0;
		HAL_UART_Transmit_IT(&huart2, &tmp, 1);

	} else {
		Empty_Tx = idx;
	}
	__enable_irq();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		Empty_Rx++;
		if (Empty_Rx >= BUFF_SIZE)
			Empty_Rx = 0;
		HAL_UART_Receive_IT(&huart2, &Buff_Rx[Empty_Rx], 1);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		if (Empty_Tx != Busy_Tx) {
			uint8_t tmp = Buff_Tx[Busy_Tx];
			Busy_Tx++;
			if (Busy_Tx >= BUFF_SIZE)
				Busy_Tx = 0;
			HAL_UART_Transmit_IT(&huart2, &tmp, 1);
		}

	}
}

uint8_t czypustyRx() {
	if (Empty_Rx == Busy_Rx) {
		return 0;
	} else {
		return 1;
	}
}

uint8_t USART_getchar() {

	uint8_t tmp;

	if (Empty_Rx != Busy_Rx) {

		tmp = Buff_Rx[Busy_Rx];
		Busy_Rx++;
		if (Busy_Rx >= BUFF_SIZE)
			Busy_Rx = 0;
		return tmp;
	} else
		return 0;
}

uint8_t USART_getline(char * buff) {

	static uint8_t bf[128];
	static uint8_t idx = 0;

	int i;
	uint8_t ret;

	while (czypustyRx()) {

		bf[idx] = USART_getchar();

		if (bf[idx] == 59) {
			bf[idx] = 0;
			for (i = 0; i <= idx; i++) {
				buff[i] = bf[i];
			}
			ret = idx;
			idx = 0;

			return ret;

		} else {
			idx++;
			if (idx >= 128)
				idx = 0;

		}

	}
	return 0;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void) {
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
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2, &Buff_Rx[0], 1);

	int len = 0;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		if ((len = USART_getline(Bx)) > 0) {

			if (strcmp("ON", Bx) == 0) {

				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			}

			else if ((strcmp("OFF", Bx) == 0)) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			}

			if (strcmp("1", Bx) == 0) {

				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			}

			else if ((strcmp("0", Bx) == 0)) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			} else if ((strcmp("BLINK", Bx) == 0)) {
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			}

		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		/* USER CODE END 3 */

	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
