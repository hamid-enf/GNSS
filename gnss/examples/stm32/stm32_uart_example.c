// examples/stm32/stm32_uart_example.c
// مثال کامل STM32 با UART + DMA (HAL)

#include "main.h"
#include "gnss.h"

extern UART_HandleTypeDef huart2;
uint8_t dma_rx_buffer[128];

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2) {
        gnss_feed(dma_rx_buffer, Size);
        
        // شروع مجدد DMA
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_rx_buffer, sizeof(dma_rx_buffer));
    }
}

void gnss_task(void)
{
    if (gnss_has_new_solution()) {
        gnss_solution_t sol;
        if (gnss_get_solution(&sol) == GNSS_OK) {
            // ارسال به LCD یا ارسال به سرور
            printf("Lat: %.7f, Lon: %.7f\n", sol.latitude_deg, sol.longitude_deg);
        }
    }
}

void SystemClock_Config(void); // تابع معمولی STM32

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    // مقداردهی UART + DMA
    MX_USART2_UART_Init();
    gnss_init();

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_rx_buffer, sizeof(dma_rx_buffer));

    while (1) {
        gnss_task();
        HAL_Delay(50);
    }
}
