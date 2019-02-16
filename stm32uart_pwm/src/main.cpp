#include "main.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_uart.h"


void systemClockConfig(void);
void gpioConfig(void);
void timerConfig(void);
void uartConfig(void);
extern void HAL_TIM_PWM_Msp_Init(TIM_HandleTypeDef* htim);
extern void HAL_UART_Msp_Init(UART_HandleTypeDef* huart);
static int read(char* buffer, int size);
void printUart(char* data, uint16_t size);

class ReadState
{
public:
    ReadState()
    {
        state_ = RECEIVE_HEADER1;
    }

    ~ReadState()
    {}

    enum State
    {
        RECEIVE_HEADER1 = 1,
        RECEIVE_HEADER2 = 2,
        RECEIVE_DATA = 3,
        RECEIVE_END = 4
    };

    void operator = (enum State& state)
    {
        state_ = state;
    }

    enum State state_;
};

TIM_HandleTypeDef tim_handle;
UART_HandleTypeDef uart_handle;
CircularBuffer* cbuffer;
ReadState read_state;

void systemClockConfig(void)
{
    RCC_ClkInitTypeDef clk_init_struct;
    RCC_OscInitTypeDef osc_init_struct;

    /*  PLL osc configuration 
        PLL = HSI / 2 * 16 = 64 MHz */
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
    gpio_init_struct.Pin = GPIO_PIN_5;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    gpio_init_struct.Pin = GPIO_PIN_13;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);

    gpio_init_struct.Pin = GPIO_PIN_8;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);
}

void timerConfig(void)
{
    TIMx_CLK_ENABLE();
    tim_handle.Instance = TIMx;
    tim_handle.Init.Prescaler = 0;
    tim_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    tim_handle.Init.Period = SystemCoreClock / 10000 - 1; // 10KHz PWM
    tim_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    tim_handle.Init.RepetitionCounter = 0;
    tim_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if(HAL_TIM_Base_Init(&tim_handle) != HAL_OK)
    {
        while(1);
    }

    HAL_TIM_PWM_Msp_Init(&tim_handle);
}

void uartConfig(void)
{
    USARTx_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init_struct;
    gpio_init_struct.Pin = GPIO_RX;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    gpio_init_struct.Pin = GPIO_TX;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    uart_handle.Instance = USARTx;
    uart_handle.Init.BaudRate = 9600;
    uart_handle.Init.WordLength = UART_WORDLENGTH_8B;
    uart_handle.Init.Parity = UART_PARITY_NONE;
    uart_handle.Init.Mode = UART_MODE_TX_RX;
    uart_handle.Init.StopBits = UART_STOPBITS_1;
    uart_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;

    if(HAL_UART_Init(&uart_handle) != HAL_OK)
    {
        /* Error Handler */
    }

    HAL_UART_Msp_Init(&uart_handle);

    __HAL_UART_ENABLE_IT(&uart_handle, UART_IT_RXNE);
}

static int read(char* buffer, int size)
{
    uint8_t data;
    int data_count = 0;
    
    while(cbuffer->readBuffer(&data) > 0)
    {
		switch (read_state.state_)
		{
		case ReadState::RECEIVE_HEADER1:
			if (data == '5')
				read_state.state_ = ReadState::RECEIVE_HEADER2;
			break;
		case ReadState::RECEIVE_HEADER2:
			if (data == '7')
				read_state.state_ = ReadState::RECEIVE_DATA;
			else
				read_state.state_ = ReadState::RECEIVE_HEADER1;
			break;
		case ReadState::RECEIVE_DATA:
			*(buffer++) = data;
			data_count++;
			if (data_count == size)
				read_state.state_ = ReadState::RECEIVE_END;
			break;
		case ReadState::RECEIVE_END:
			if (data == '#')
			{
				read_state.state_ = ReadState::RECEIVE_HEADER1;
				return 1;
			}
			else
			{
				read_state.state_ = ReadState::RECEIVE_HEADER1;
				data_count = 0;
				return -1;
			}
		default:
			break;
		}
	}
	return -1;
}

void printUart(char* data, uint16_t size)
{
    HAL_UART_Transmit(&uart_handle, (uint8_t*)data, size, 0x000F);
}

int main(void)
{
    cbuffer = new CircularBuffer(50);
    
    HAL_Init();

    systemClockConfig();

    gpioConfig();

    timerConfig();

    HAL_TIM_Base_Start(&tim_handle);

    HAL_TIM_PWM_Start(&tim_handle, TIM_CHANNEL_1);

    uartConfig();

    char command[4];

    while(1)
    {
        if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0)
        {
            __HAL_TIM_SET_COMPARE(&tim_handle, TIM_CHANNEL_1, 300);
        }

        if(read(command, 4) == 1)
        {
            int pulse = atoi(command);
            // char report[40];
            // snprintf(report, 40, "Command is obtained: %d\n", pulse);
            // uint16_t size = sizeof(report) / sizeof(report[0]);
            // printUart(report, size);
            __HAL_TIM_SET_COMPARE(&tim_handle, TIM_CHANNEL_1, pulse);
        }
        HAL_Delay(100);
    }

    return 0;
}