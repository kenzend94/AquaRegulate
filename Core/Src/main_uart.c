/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main_uart.c
 * @brief          : Main program body
 * Name: AquaRegulate
 * Date: 03/03/2024
 * School: The Univeristy of Utah
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f072xb.h"

/* STM32F072RB
PB10 -> TX
PB11 -> RX */


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
void sendChar(char);
void stringArray(char[]);

// please comment out this line for part 1
void USART3_4_IRQHandler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char temp, number;
int flag;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	HAL_Init();
	SystemClock_Config();

	// Enable GPIOB, GPIOC and USART3
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	// USART3 setup
	USART3->BRR |= 8000000 / 115200; // set baud rate to 115200
	USART3->CR1 |= USART_CR1_TE; // enable transmission

	USART3->CR1 |= USART_CR1_RE; // enable reception
	USART3->CR1 |= USART_CR1_UE | USART_CR1_RXNEIE; // enable USART3 and enable RXNE interrupt

	// Set alternate function mode for pb10 and pb11
	GPIOB->AFR[1] |= (4 << 8);
	GPIOB->AFR[1] |= (4 << 12);
	GPIOB->MODER |= (2 << 20);
	GPIOB->MODER |= (2 << 22);

	// Initialize LED pins
	GPIOC->MODER |= (1 << 12) | 
					(1 << 18) | 
					(1 << 14) | 
					(1 << 16);

	GPIOC->MODER &= ~(1 << 13);
	GPIOC->MODER &= ~(1 << 19);
	GPIOC->MODER &= ~(1 << 15);
	GPIOC->MODER &= ~(1 << 17);

	// Set to output mode
	GPIOC->OTYPER &= ~(1 << 13);
	GPIOC->OTYPER &= ~(1 << 12);
	GPIOC->OTYPER &= ~(1 << 18);
	GPIOC->OTYPER &= ~(1 << 19);
	GPIOC->OTYPER &= ~(1 << 14);
	GPIOC->OTYPER &= ~(1 << 15);
	GPIOC->OTYPER &= ~(1 << 16);
	GPIOC->OTYPER &= ~(1 << 17);

	// Set to low speed
	GPIOC->OSPEEDR &= ~(1 << 12);
	GPIOC->OSPEEDR &= ~(1 << 18);
	GPIOC->OSPEEDR &= ~(1 << 14);
	GPIOC->OSPEEDR &= ~(1 << 16);

	// Set to no pull-up, no pull-down
	GPIOC->PUPDR &= ~(1 << 13);
	GPIOC->PUPDR &= ~(1 << 12);
	GPIOC->PUPDR &= ~(1 << 18);
	GPIOC->PUPDR &= ~(1 << 19);
	GPIOC->PUPDR &= ~(1 << 14);
	GPIOC->PUPDR &= ~(1 << 15);
	GPIOC->PUPDR &= ~(1 << 16);
	GPIOC->PUPDR &= ~(1 << 17);

	// Enable USART3_4_IRQn
	NVIC_EnableIRQ(USART3_4_IRQn);

	char tempPart1;
	flag = 0;
	while (1)
	{
		// sendChar('a');
		// stringArray("abc");
		// Part 1
		// if (USART3->ISR & (1 << 5)) {
		// 	tempPart1 = USART3->RDR;
		// 	if(tempPart1 == 'r') {
		// 		GPIOC->ODR ^= (1 << 6);
		// 	}
		// 	else if(tempPart1 == 'b') {
		// 		GPIOC->ODR ^= (1 << 7);
		// 	}
		// 	else if(tempPart1 == 'g') {
		// 		GPIOC->ODR ^= (1 << 9);
		// 	}
		// 	else if(tempPart1 == 'o') {
		// 		GPIOC->ODR ^= (1 << 8);
		// 	}
		// 	else {
		// 		stringArray("Wrong Key");
		// 	}
		// }

		// Part 2

		stringArray("CMD?");

		HAL_Delay(400);
	}
	/* USER CODE END 3 */
}

// Sending a character
void sendChar(char symbol)
{
	while (!(USART3->ISR & (1 << 7)))
	{
	}

	USART3->TDR = symbol;
}

// Sending a string
void stringArray(char charArr[])
{
	int i = 0;
	while (charArr[i] != 0)
	{
		sendChar(charArr[i]);
		i++;
	}
}

// Interrupt handler
void USART3_4_IRQHandler(void)
{
	if (flag == 0)
	{
		temp = USART3->RDR;
		flag = 1;
	}
	else if (flag == 1)
	{
		number = USART3->RDR;
		flag = 0;
		if (temp == 'r')
		{
			if (number == '0')
				GPIOC->ODR &= ~(1 << 6);
			else if (number == '1')
				GPIOC->ODR |= (1 << 6);
			else if (number == '2')
				GPIOC->ODR ^= (1 << 6);
			else
				stringArray("Wrong Key");
		}
		else if (temp == 'b')
		{
			if (number == '0')
				GPIOC->ODR &= ~(1 << 7);
			else if (number == '1')
				GPIOC->ODR |= (1 << 7);
			else if (number == '2')
				GPIOC->ODR ^= (1 << 7);
			else
				stringArray("Wrong Key");
		}
		else if (temp == 'g')
		{
			if (number == '0')
				GPIOC->ODR &= ~(1 << 9);
			else if (number == '1')
				GPIOC->ODR |= (1 << 9);
			else if (number == '2')
				GPIOC->ODR ^= (1 << 9);
			else
				stringArray("Wrong Key");
		}
		else if (temp == 'o')
		{
			if (number == '0')
				GPIOC->ODR &= ~(1 << 8);
			else if (number == '1')
				GPIOC->ODR |= (1 << 8);
			else if (number == '2')
				GPIOC->ODR ^= (1 << 8);
			else
				stringArray("Wrong Key");
		}
		else
		{
			stringArray("Wrong Key");
		}
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
