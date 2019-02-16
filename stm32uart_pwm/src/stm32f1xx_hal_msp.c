#include "main.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_uart.h"



void HAL_TIM_PWM_Msp_Init(TIM_HandleTypeDef* htim)
{
        TIMx_CLK_ENABLE();

        TIM_OC_InitTypeDef oc_init_struct = {0};
        oc_init_struct.OCMode = TIM_OCMODE_PWM1;
        oc_init_struct.Pulse = 100;
        oc_init_struct.OCPolarity = TIM_OCPOLARITY_LOW;
        oc_init_struct.OCIdleState = TIM_OCIDLESTATE_SET;

        HAL_TIM_PWM_Init(htim);

        if(HAL_TIM_PWM_ConfigChannel(htim, &oc_init_struct, TIM_CHANNEL_1) != HAL_OK)
        {
            while(1);
        }

        // __HAL_RCC_GPIOA_CLK_ENABLE();

        // GPIO_InitTypeDef gpio_init_struct;
        // gpio_init_struct.Pin = GPIO_PIN_8;
        // gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        // gpio_init_struct.Pull = GPIO_NOPULL;
        // gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        // HAL_GPIO_Init(GPIOA, &gpio_init_struct);
}


void HAL_UART_Msp_Init(UART_HandleTypeDef* huart)
{
    

    HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(USARTx_IRQn);
}