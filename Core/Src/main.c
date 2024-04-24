/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 * Name: AquaRegulate
 * Date: 04/01/2024
 * School: The Univeristy of Utah
 ******************************************************************************
 */
/* USER CODE END Header */
#include "main.h"
#include "stm32f072xb.h"
#include "stm32f0xx_it.h"
#include "stdio.h"
#include "stm32f0xx_hal.h"
// include hal gpio
#include "stm32f0xx_hal_gpio.h"
// include hal adc
#include "stm32f0xx_hal_adc.h"
#include "stm32f0xx_hal_adc_ex.h"


/* STM32F072RB
PB10 -> TX
PB11 -> RX */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void TransmitMoistureValue();
void TransmitString(char *str);
void TransmitChar(char c);

// UART
static void init_UART();
void Start_UART();
UART_HandleTypeDef huart3;

// ADC
void init_ADC();
void Start_ADC(void);
ADC_HandleTypeDef hadc;

// GPIO
void init_GPIO();

uint32_t temp_sensor_value;

int main(void)
{
	/************ ADC START ************/
	// Basic Setup
	HAL_Init();
	SystemClock_Config();

	// Initialize the GPIO
	init_GPIO();

	// Initialize the ADC
	init_ADC();
	init_UART();

	// // 4.3 Interrupt-Based Reception
	// NVIC_EnableIRQ(USART3_4_IRQn);

	// Read the ADC data register and turn on/off LEDs depending on the value.
	while (1)
	{
		// Get the ADC value
		if (HAL_ADC_PollForConversion(&hadc, 1000) == HAL_OK)
		{
			TransmitString("\nADC Value: ");
			temp_sensor_value = HAL_ADC_GetValue(&hadc);
			// SetLEDSByADC();
			TransmitMoistureValue();
			TransmitString("\nEnd");

			// Delay for 1 second
			HAL_Delay(10000);
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


void TransmitMoistureValue()
{
	// Convert temp_sensor_value from hex to dec
	char str[8];
	sprintf(str, "%d", temp_sensor_value);
	TransmitString(str);
}

void TransmitChar(char c)
{
	while (!(USART3->ISR & USART_ISR_TXE))
		; // exits once the flag is set.

	// Write the character into the transmit data register.
	USART3->TDR = c;
}

void init_UART()
{
	// Enable UART Clock
	__HAL_RCC_USART3_CLK_ENABLE();

	// UART Configuration
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;

	// Start UART
	Start_UART();
}

void Start_UART()
{
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
}

void init_GPIO(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIO clocks
	__HAL_RCC_GPIOA_CLK_ENABLE(); // For UART
    __HAL_RCC_GPIOB_CLK_ENABLE(); // For UART
    __HAL_RCC_GPIOC_CLK_ENABLE(); // For ADC
	__HAL_RCC_ADC1_CLK_ENABLE(); // For ADC

    // UART GPIO Configuration
    // PB10 -> USART3_TX, PB11 -> USART3_RX
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // ADC GPIO Configuration
    // PC0 -> ADC1_IN10
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    // No need to set Speed or Alternate as it's analog mode
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void init_ADC()
{
	// ADC Configuration
	ADC_ChannelConfTypeDef sConfig = {0};

	// Configure the ADC peripheral
	hadc.Instance = ADC1;
	hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc.Init.Resolution = ADC_RESOLUTION_8B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	// hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;

	hadc.Init.ScanConvMode = DISABLE; // omit this line if scanning is not used

	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.ContinuousConvMode = ENABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = DISABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;

	if (HAL_ADC_Init(&hadc) != HAL_OK)
	{
		Error_Handler();
	}

	// Configure the ADC channel
	sConfig.Channel = ADC_CHANNEL_10; // Corresponds to PC0
	// sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;

	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	// Start the ADC
	Start_ADC();
}


void Start_ADC(void)
{
	// Start ADC conversion
	if (HAL_ADC_Start(&hadc) != HAL_OK)
	{
		Error_Handler();
	}
}

void TransmitString(char *str)
{
	for (int i = 0; str[i] != '\0'; i++)
	{
		TransmitChar(str[i]);
	}
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