#ifndef __MAIN_H_
#define __MAIN_H_

#include <string.h>
#include <stdlib.h>

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"

#define USARTx USART2
#define USARTx_CLK_ENABLE() __HAL_RCC_USART2_CLK_ENABLE()
#define USARTx_IRQn USART2_IRQn
#define USARTx_IRQHandler USART2_IRQHandler 
#define GPIO_TX GPIO_PIN_2
#define GPIO_RX GPIO_PIN_3

#endif