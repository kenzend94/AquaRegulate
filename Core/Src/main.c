/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 * Name: AquaRegulate
 * Date: 04/01/2024
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
#include "stm32f0xx_it.h"
#include "stdio.h"

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
void SetEnable();
void InitializeLEDPins();
void Calibration();
void EnableADC();
void SetLEDSByADC();

// void TransmitMoistureValue(uint32_t temp_sensor_value);
void TransmitMoistureValue();

uint32_t temp_sensor_value;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TransmitChar(char c)
{
  while (!(USART3->ISR & USART_ISR_TXE)); // exits once the flag is set.

  // Write the character into the transmit data register.
  USART3->TDR = c;
}

void TransmitString(char* str)
{
  for (int i = 0; str[i] != '\0'; i++)
  {
    TransmitChar(str[i]);
  }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/************ ADC START ************/
	// Basic Setup
	HAL_Init();
	SystemClock_Config();

	// Initialize the LED pins to output.
	SetEnable();
	InitializeLEDPins();

	// Select a GPIO pin to use as the ADC input. (PC0)
	// Configure to Analog mode
	GPIOC->MODER |= (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER0_0);
	// No pull-up, pull-down
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR0_0);

	// Configure the ADC.
	// 8-bit resolution
	ADC1->CFGR1 |= ADC_CFGR1_RES_1;
	ADC1->CFGR1 &= ~ADC_CFGR1_RES_0;
	// Continuous conversion mode
	ADC1->CFGR1 |= ADC_CFGR1_CONT;
	// Hardware triggers disabled
	ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN;

	// Select/enable the input pin’s channel for ADC conversion.
	ADC1->CHSELR |= ADC_CHSELR_CHSEL10;

	// Perform a self-calibration, enable, and start the ADC.
	Calibration();
	EnableADC();
	// Start ADC.
	ADC1->CR |= ADC_CR_ADSTART;
	/************ ADC END ************/
	// printf("Hello kid from South Koera");
	/************ UART START ************/
	unsigned int baud_rate = 115200;

	// Enable the system clock to the desired USART in the RCC peripheral.
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // Enable USART3 clock
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;	  // Enable GPIOB clock

	// 4.1 Preparing to use the USART
	// PB10 - USART3_TX, PB11 - USART3_RX
	// Set MODER to Alternate Function mode
	GPIOB->MODER |= (GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1);
	GPIOB->MODER &= ~(GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0);
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_11);
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR10 | GPIO_OSPEEDR_OSPEEDR11);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR10 | GPIO_PUPDR_PUPDR11);

	GPIOB->AFR[1] |= ((4 << 8) | (4 << 12));

	// Set the Baud rate for communication to be 115200 bits/second.
	USART3->BRR = (uint16_t)(HAL_RCC_GetHCLKFreq() / baud_rate); // 69
	USART3->CR1 |= (USART_CR1_TE | USART_CR1_RE);				 // Enable Receiver
	USART3->CR1 |= USART_CR1_UE;								 // Enable USART
	USART3->CR1 |= USART_CR1_RXNEIE;

	// 4.3 Interrupt-Based Reception
	NVIC_EnableIRQ(USART3_4_IRQn);
	/************ UART END ************/

	// Read the ADC data register and turn on/off LEDs depending on the value.
	while (1)
	{
		temp_sensor_value = ADC1->DR;
		// SetLEDSByADC();
		TransmitMoistureValue();
		// TransmitMoistureValue(temp_sensor_value);
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
void SetEnable()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
}

void InitializeLEDPins()
{
	GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
								GPIO_MODE_OUTPUT_PP,
								GPIO_SPEED_FREQ_LOW,
								GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initStr);
}

void Calibration()
{
	/* (1) Ensure that ADEN = 0 */
	/* (2) Clear ADEN by setting ADDIS */
	/* (3) Clear DMAEN */
	/* (4) Launch the calibration by setting ADCAL */
	/* (5) Wait until ADCAL=0 */
	if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
	{
		ADC1->CR |= ADC_CR_ADDIS; /* (2) */
	}
	while ((ADC1->CR & ADC_CR_ADEN) != 0)
		;
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; /* (3) */
	ADC1->CR |= ADC_CR_ADCAL;		 /* (4) */
	while ((ADC1->CR & ADC_CR_ADCAL) != 0)
		; /* (5) */
}

void EnableADC()
{
	/* Enable the ADC */
	/* (1) Ensure that ADRDY = 0 */
	/* (2) Clear ADRDY */
	/* (3) Enable the ADC */
	/* (4) Wait until ADC ready */
	if (ADC1->ISR & ADC_ISR_ADRDY) /* (1) */
	{
		ADC1->ISR |= ADC_ISR_ADRDY; /* (2) */
	}
	ADC1->CR |= ADC_CR_ADEN; /* (3) */
	while (!(ADC1->ISR & ADC_ISR_ADRDY))
		; /* (4) */
}

void SetLEDSByADC()
{
	int threshold1 = 0;
	int threshold2 = 65;
	int threshold3 = 130;
	int threshold4 = 250;

	// Reset all LEDs
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

	if (ADC1->DR > threshold1 && ADC1->DR <= threshold2)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	}
	else if (ADC1->DR > threshold2 && ADC1->DR <= threshold3)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	}
	else if (ADC1->DR > threshold3 && ADC1->DR <= threshold4)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	}
	else if (ADC1->DR > threshold4)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	}
}

void TransmitMoistureValue()
{
	// Convert temp_sensor_value from hex to dec
	char str[8];
    sprintf(str, "%d\n", temp_sensor_value);
	TransmitString(str);
}


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
