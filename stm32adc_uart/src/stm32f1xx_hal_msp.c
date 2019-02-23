#include "main.h"

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef          GPIO_InitStruct;

  RCC_PeriphCLKInitTypeDef  PeriphClkInit;

  __HAL_RCC_ADC1_CLK_ENABLE();
  
  // PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  // PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  // HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}