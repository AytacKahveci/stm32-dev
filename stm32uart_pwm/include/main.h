#ifndef __MAIN_H_
#define __MAIN_H_

#include "stdlib.h"
#include "stdint.h"
#include "string.h"

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "circular_buffer.h"


#define TIMx TIM1
#define TIMx_CLK_ENABLE() __HAL_RCC_TIM1_CLK_ENABLE()

#define TIMx_IRQn TIM1_IRQn
#define TIMx_IRQHandler TIM1_IRQHandler

#define USARTx USART2
#define USARTx_CLK_ENABLE() __HAL_RCC_USART2_CLK_ENABLE()
#define USARTx_IRQn USART2_IRQn
#define USARTx_IRQHandler USART2_IRQHandler 
#define GPIO_TX GPIO_PIN_2
#define GPIO_RX GPIO_PIN_3

#endif