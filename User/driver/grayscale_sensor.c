#include "grayscale_sensor.h"

static ADC_HandleTypeDef hadc1;

const static uint8_t number_of_sensors = 8;

void grayscale_init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
  ADC_ChannelConfTypeDef sConfig;
	GPIO_InitTypeDef GPIO_InitStruct;

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = number_of_sensors;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	HAL_ADC_Init(&hadc1);

	__HAL_RCC_ADC1_CLK_ENABLE();

	/**ADC1 GPIO Configuration    
	PA1     ------> ADC1_IN1
	PA0/WKUP     ------> ADC1_IN0
	PA2     ------> ADC1_IN2
	PA3     ------> ADC1_IN3 
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


	HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ADC_IRQn);

	uint32_t adc_channels[number_of_sensors] = {
		ADC_CHANNEL_0,
		ADC_CHANNEL_1,
		ADC_CHANNEL_2,
		ADC_CHANNEL_3,
		ADC_CHANNEL_4,
		ADC_CHANNEL_5,
		ADC_CHANNEL_8,
		ADC_CHANNEL_9,
	};
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	for (uint8_t i = 0; i < number_of_sensors; i ++) {
		sConfig.Channel = adc_channels[i];
		sConfig.Rank = i + 1;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	}
}



static uint16_t adc_data[4];
uint16_t * grayscale_read (void) {
	ADC_HandleTypeDef * hadc = &hadc1;
	HAL_ADC_Start(hadc);
	
	for (uint8_t i = 0; i < number_of_sensors; i++) {
		HAL_ADC_PollForConversion(hadc, 100);
		adc_data[i] = HAL_ADC_GetValue(hadc);
	}
	HAL_ADC_Stop(hadc);
	return adc_data;
}
