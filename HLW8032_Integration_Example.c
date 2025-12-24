/*
 * HLW8032_Integration_Example.c
 *
 * Complete HLW8032 integration example with STM32
 * Includes UART and Timer Input Capture
 *
 * Author: Ehsan Zehni
 * Created: 2025
 */

/* ========== In stm32f4xx_it.c file ========== */

#include "stm32f4xx_it.h"
#include "HLW8032.h"

extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim2;

/* UART Interrupt Handler */
void UART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}

/* Timer Input Capture Interrupt Handler */
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

/* UART RX Complete Callback */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        /* Call HLW8032 UART callback */
        extern void HLW8032_UART_RxCpltCallback(UART_HandleTypeDef *huart);
        HLW8032_UART_RxCpltCallback(huart);
    }
}

/* Timer Input Capture Callback */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        /* Call HLW8032 Timer callback */
        extern void HLW8032_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
        HLW8032_TIM_IC_CaptureCallback(htim);
    }
}

/* ========== In main.c file ========== */

/*
#include "main.h"
#include "HLW8032.h"
#include "uart.h"
#include "tim.h"

// External port functions (defined in HLW8032_Example.c)
extern hlw8032_port_t hlw8032_port;
extern hlw8032_handle_t hlw8032_handle;

int main(void)
{
    // HAL initialization
    HAL_Init();
    SystemClock_Config();
    
    // Peripheral initialization
    MX_GPIO_Init();
    MX_UART3_Init();  // UART for HLW8032 TX
    MX_TIM2_Init();   // Timer for PF pin Input Capture
    
    // Initialize HLW8032
    hlw8032_init(&hlw8032_handle, &hlw8032_port);
    hlw8032_set_line_frequency(&hlw8032_handle, 50);  // 50Hz line frequency
    
    // Start UART receive in interrupt mode
    uint8_t rx_byte;
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    
    // Start Timer Input Capture
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    
    // Main loop
    while (1)
    {
        // Process HLW8032 UART data
        hlw8032_process_uart(&hlw8032_handle);
        
        // Read and display data
        hlw8032_data_t data;
        if (hlw8032_read_data(&hlw8032_handle, &data) == HLW8032_OK)
        {
            printf("=== HLW8032 Data ===\r\n");
            printf("Voltage: %.2f V\r\n", data.voltage);
            printf("Current: %.2f A\r\n", data.current);
            printf("Power: %.2f W\r\n", data.active_power);
            printf("Energy: %.6f kWh\r\n", data.energy);
            printf("Power Factor: %.3f\r\n", data.power_factor);
            printf("PF Frequency: %lu Hz\r\n", data.pf_frequency);
            printf("==================\r\n");
        }
        
        HAL_Delay(1000);  // Update every 1 second
    }
}
*/

/* ========== In stm32f4xx_hal_msp.c file ========== */

/*
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* htim_ic)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    if(htim_ic->Instance == TIM2)
    {
        // Enable clocks
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        
        // Configure GPIO for Input Capture
        // PA0 -> TIM2_CH1 (PF pin from HLW8032)
        GPIO_InitStruct.Pin = GPIO_PIN_0;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        
        // Configure interrupt
        HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    if(huart->Instance == USART3)
    {
        // Enable clocks
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        
        // Configure GPIO for UART
        // PB10 -> USART3_TX (to HLW8032 RX)
        // PB11 -> USART3_RX (from HLW8032 TX)
        GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        
        // Configure interrupt
        HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
}
*/

/* ========== Connection Summary ========== */
/*
 * HLW8032 IC Connections:
 * 
 * HLW8032 TX  -> STM32 USART3 RX (PB11)
 * HLW8032 RX  -> STM32 USART3 TX (PB10) [Optional]
 * HLW8032 PF  -> STM32 TIM2_CH1 (PA0) [Input Capture]
 * HLW8032 VCC -> 3.3V or 5V
 * HLW8032 GND -> GND
 * 
 * UART Settings:
 * - Baud Rate: 4800
 * - Data Bits: 8
 * - Stop Bits: 1
 * - Parity: None
 * 
 * Timer Settings for Input Capture:
 * - Timer: TIM2
 * - Channel: CH1
 * - Mode: Input Capture on Rising Edge
 * - Prescaler: 84 (for 1MHz timer frequency)
 * - Period: 0xFFFFFFFF (maximum)
 */

