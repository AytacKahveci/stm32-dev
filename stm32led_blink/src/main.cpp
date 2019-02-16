#include "main.h"

void systemClockConfig(void);
void gpioConfig(void);

void systemClockConfig(void)
{
    RCC_OscInitTypeDef osc_init_struct = {0};
    RCC_ClkInitTypeDef clk_init_struct = {0};

    /*  Configure PLL 
        PLLCLK = HSI / 2 * 16 = 64 MHz
    */
    osc_init_struct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc_init_struct.HSEState = RCC_HSE_OFF;
    osc_init_struct.LSEState = RCC_LSE_OFF;
    osc_init_struct.HSIState = RCC_HSI_ON;
    osc_init_struct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc_init_struct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    osc_init_struct.PLL.PLLState = RCC_PLL_ON;
    osc_init_struct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    osc_init_struct.PLL.PLLMUL = RCC_PLL_MUL16;
    if(HAL_RCC_OscConfig(&osc_init_struct) != HAL_OK)
    {
        while(1);
    }

    clk_init_struct.ClockType = (RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    clk_init_struct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk_init_struct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk_init_struct.APB1CLKDivider = RCC_HCLK_DIV1;
    clk_init_struct.APB2CLKDivider = RCC_HCLK_DIV2;
    if(HAL_RCC_ClockConfig(&clk_init_struct, FLASH_LATENCY_2) != HAL_OK)
    {
        while(1);
    }
}

void gpioConfig(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef gpio_init_struct;

    /* LED LD2 */
    gpio_init_struct.Pin = GPIO_PIN_5;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    /* Button USER B1 */
    gpio_init_struct.Pin = GPIO_PIN_13;
    gpio_init_struct.Mode = GPIO_MODE_AF_INPUT;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);
}

int main(void)
{
    HAL_Init();

    systemClockConfig();

    gpioConfig();

    while(1)
    {
        if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);            
        }
        HAL_Delay(100);
    }


    return 0;
}