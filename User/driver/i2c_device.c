#include "i2c_device.h"

I2C_HandleTypeDef hi2c2;

void I2C_Init() {
	__HAL_RCC_GPIOF_CLK_ENABLE();

	hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2; 
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c2);
}


void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hi2c->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */
  
    /**I2C2 GPIO Configuration    
    PF0     ------> I2C2_SDA
    PF1     ------> I2C2_SCL 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  /* USER CODE BEGIN I2C2_MspInit 1 */
  /* USER CODE END I2C2_MspInit 1 */
  }

}

uint8_t I2C_Ready(uint8_t address) {
	return HAL_I2C_IsDeviceReady(&hi2c2, address, 1, 10) == HAL_OK;
}

void I2C_Read(uint8_t address, uint8_t * data, uint16_t size) {
	HAL_I2C_Master_Receive(&hi2c2, address, data, size, 10);
}

void I2C_Write(uint8_t address, uint8_t * data, uint16_t size) {
	HAL_I2C_Master_Transmit(&hi2c2, address, data, size, 10);
}


uint8_t External_ADC_Read(uint8_t channel) {
		uint8_t data[1] = {channel};
		I2C_Write(0x90, data, 1);
		I2C_Read(0x91, data, 1);
		return data[0];
}

