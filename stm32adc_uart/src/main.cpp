#include "main.h"
#include "stm32f1xx_hal_adc.h"
#include "stm32f1xx_hal_uart.h"

void systemClockConfig(void);
void gpioConfig(void);
void adcConfig(void);
void uartConfig(void);
void printFunc(char* buffer, uint16_t size);

ADC_HandleTypeDef adc_handle;
UART_HandleTypeDef uart_handle;

void systemClockConfig(void)
{
    RCC_ClkInitTypeDef clk_init_struct;
    RCC_OscInitTypeDef osc_init_struct;

    /* 64Mhz oscillator PLL configuration */
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

    GPIO_InitTypeDef gpio_init_struct;
    gpio_init_struct.Pin = GPIO_PIN_5;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);
}


void adcConfig(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    RCC_PeriphCLKInitTypeDef  PeriphClkInit;    
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    GPIO_InitTypeDef gpio_init_struct;
    gpio_init_struct.Pin = GPIO_PIN_7;
    gpio_init_struct.Mode = GPIO_MODE_ANALOG;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    adc_handle.Instance = ADC1;
    adc_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    adc_handle.Init.ScanConvMode = ADC_SCAN_DISABLE;
    adc_handle.Init.ContinuousConvMode = ENABLE;
    adc_handle.Init.NbrOfConversion = 0;
    adc_handle.Init.DiscontinuousConvMode = DISABLE;
    adc_handle.Init.NbrOfDiscConversion = 1;
    adc_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START; 
    adc_handle.State = RESET;

    HAL_ADC_Init(&adc_handle);

    ADC_ChannelConfTypeDef adc_channel;
    adc_channel.Channel = ADC_CHANNEL_7;
    adc_channel.Rank = 1;
    adc_channel.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

    if(HAL_ADC_ConfigChannel(&adc_handle, &adc_channel) != HAL_OK)
    {
        while(1);
    }

}


void uartConfig(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init_struct;
    gpio_init_struct.Pin = GPIO_TX; // Tx Pin
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;    
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    gpio_init_struct.Pin = GPIO_RX; // Rx Pin
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;    
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    uart_handle.Instance = USART2;
    uart_handle.Init.BaudRate = 9600;
    uart_handle.Init.Mode = UART_MODE_TX_RX;
    uart_handle.Init.Parity = UART_PARITY_NONE;
    uart_handle.Init.StopBits = UART_STOPBITS_1;
    uart_handle.Init.WordLength = UART_WORDLENGTH_8B;
    uart_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart_handle.Init.OverSampling = UART_OVERSAMPLING_16;

    HAL_UART_Init(&uart_handle);
}

void printFunc(char* buffer, uint16_t size)
{
    if(HAL_UART_Transmit(&uart_handle, (uint8_t *)buffer, size, 0xFFFFF) != HAL_OK)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

int main(void)
{
    HAL_Init();

    systemClockConfig();

    gpioConfig();

    uartConfig();

    adcConfig();

    HAL_ADC_Start(&adc_handle);

    uint32_t adc_value;
    char* buffer = (char*)malloc(50);
    while(1)
    {

        if(HAL_ADC_PollForConversion(&adc_handle, 0xFFFF) == HAL_OK)
        {
            adc_value = HAL_ADC_GetValue(&adc_handle);
            float voltage = adc_value * 5.0 / 4096.0;
            float tmp_val = voltage < 0 ? -voltage : voltage;
            int tmp1 = tmp_val;
            float tmp_frac = tmp_val - tmp1;
            int tmp2 = tmp_frac * 10000;
            sprintf(buffer,"Data: %d.%04d\n", tmp1, tmp2);
            printFunc(buffer, 50);
        }

        HAL_Delay(100);
    }

    free(buffer);

    return 0;
}