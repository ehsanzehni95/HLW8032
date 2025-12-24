/*
 * HLW8032_Example.c
 *
 * Complete usage example for HLW8032 library
 * This file demonstrates how to read all parameters
 *
 * Author: Ehsan Zehni
 * Created: 2025
 */

#include "HLW8032.h"
#include "stm32f4xx_hal.h"  /* Adjust for your platform */
#include <stdio.h>
#include <math.h>

/* External handles - adjust for your hardware */
extern UART_HandleTypeDef huart3;  /* UART for HLW8032 TX */
extern TIM_HandleTypeDef htim2;    /* Timer for PF pin input capture */

/* UART RX buffer for DMA or interrupt mode */
static uint8_t uart_rx_buffer[256];
static volatile uint16_t uart_rx_index = 0;
static volatile uint16_t uart_rx_count = 0;

/* Timer variables for PF frequency measurement */
static volatile uint32_t timer_capture_value = 0;
static volatile uint32_t timer_capture_last = 0;
static volatile uint32_t timer_period = 0;
static volatile bool timer_capture_ready = false;
static volatile uint32_t timer_frequency = 1000000;  /* 1MHz timer clock */

/* ========== Port Functions Implementation ========== */

/* UART receive function */
static int uart_receive(uint8_t *data, uint16_t size, uint32_t timeout)
{
    if (HAL_UART_Receive(&huart3, data, size, timeout) == HAL_OK) {
        return 0;  /* Success */
    }
    return -1;  /* Error */
}

/* Get UART RX count */
static int uart_get_rx_count(void)
{
    /* Return number of bytes available in RX buffer */
    /* For interrupt mode, use a circular buffer */
    return uart_rx_count;
}

/* Clear UART RX buffer */
static void uart_clear_rx(void)
{
    /* Clear UART RX flags and buffer */
    __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
    uart_rx_count = 0;
    uart_rx_index = 0;
}

/* Get timer capture value */
static uint32_t timer_get_capture_value(void)
{
    /* Return the period between two PF pulses */
    /* This is calculated in the Input Capture callback */
    if (timer_capture_ready) {
        timer_capture_ready = false;
        return timer_period;
    }
    return timer_capture_value;  /* Fallback to single capture */
}

/* Get timer frequency */
static uint32_t timer_get_frequency(void)
{
    /* Return timer clock frequency in Hz */
    /* Adjust based on your timer configuration */
    return timer_frequency;
}

/* Reset timer capture */
static void timer_reset_capture(void)
{
    /* Reset timer capture registers and variables */
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    timer_capture_value = 0;
    timer_capture_last = 0;
    timer_period = 0;
    timer_capture_ready = false;
    
    /* Restart Input Capture */
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
}

/* Delay function */
static void delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/* Port functions structure */
static hlw8032_port_t hlw8032_port = {
    .uart_receive = uart_receive,
    .uart_get_rx_count = uart_get_rx_count,
    .uart_clear_rx = uart_clear_rx,
    .timer_get_capture_value = timer_get_capture_value,
    .timer_get_frequency = timer_get_frequency,
    .timer_reset_capture = timer_reset_capture,
    .delay_ms = delay_ms
};

/* HLW8032 handle */
static hlw8032_handle_t hlw8032_handle;

/* ========== UART Interrupt Callback ========== */
/* Call this from HAL_UART_RxCpltCallback */
void HLW8032_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart3.Instance) {
        /* Store received byte */
        if (uart_rx_index < sizeof(uart_rx_buffer)) {
            uart_rx_buffer[uart_rx_index++] = huart->Instance->DR;
            uart_rx_count++;
        }
        
        /* Restart UART receive */
        HAL_UART_Receive_IT(&huart3, &uart_rx_buffer[uart_rx_index], 1);
    }
}

/* ========== Timer Input Capture Callback ========== */
/* 
 * This callback must be called from HAL_TIM_IC_CaptureCallback
 * 
 * In stm32f4xx_it.c or main.c:
 * 
 * void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
 * {
 *     if (htim->Instance == TIM2) {
 *         HLW8032_TIM_IC_CaptureCallback(htim);
 *     }
 * }
 */
void HLW8032_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim2.Instance) {
        /* Read current capture value */
        uint32_t current_capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        
        if (timer_capture_last != 0) {
            /* Calculate period between two captures */
            if (current_capture > timer_capture_last) {
                timer_period = current_capture - timer_capture_last;
            } else {
                /* Handle timer overflow */
                timer_period = (0xFFFFFFFF - timer_capture_last) + current_capture;
            }
            timer_capture_ready = true;
        }
        
        timer_capture_last = current_capture;
        timer_capture_value = current_capture;
        
        /* Reset counter for next capture */
        __HAL_TIM_SET_COUNTER(htim, 0);
        
        /* Restart Input Capture */
        HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_1);
    }
}

/* ========== Example Functions ========== */

/**
 * Initialize HLW8032
 */
void HLW8032_Example_Init(void)
{
    hlw8032_status_t status;
    
    /* Initialize HLW8032 library */
    status = hlw8032_init(&hlw8032_handle, &hlw8032_port);
    if (status != HLW8032_OK) {
        /* Error handling */
        printf("HLW8032 init failed!\r\n");
        return;
    }
    
    /* Start UART receive in interrupt mode */
    HAL_UART_Receive_IT(&huart3, &uart_rx_buffer[0], 1);
    
    /* Start Timer Input Capture for PF pin */
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    
    printf("HLW8032 initialized successfully\r\n");
    printf("Timer Input Capture started for PF pin\r\n");
}

/**
 * Process HLW8032 data (call periodically)
 */
void HLW8032_Example_Process(void)
{
    hlw8032_status_t status;
    
    /* Process UART data */
    status = hlw8032_process_uart(&hlw8032_handle);
    
    if (status == HLW8032_OK) {
        /* New frame received and parsed successfully */
        printf("New frame received!\r\n");
    } else if (status == HLW8032_CRC_ERROR) {
        printf("CRC error!\r\n");
    }
}

/**
 * Example 1: Read all parameters using structure
 */
void HLW8032_Example_ReadAllParameters(void)
{
    hlw8032_data_t data;
    hlw8032_status_t status;
    
    /* Read all data at once */
    status = hlw8032_read_data(&hlw8032_handle, &data);
    
    if (status == HLW8032_OK && data.data_valid) {
        printf("=== HLW8032 Measurements ===\r\n");
        printf("Voltage: %.3f V\r\n", data.voltage);
        printf("Current: %.3f A\r\n", data.current);
        printf("Active Power: %.3f W\r\n", data.active_power);
        printf("Energy: %.6f kWh\r\n", data.energy);
        printf("Power Factor: %.3f\r\n", data.power_factor);
        printf("PF Frequency: %lu Hz\r\n", data.pf_frequency);
        printf("===========================\r\n");
    } else {
        printf("Data not ready or invalid\r\n");
    }
}

/**
 * Example 2: Read individual parameters
 */
void HLW8032_Example_ReadIndividualParameters(void)
{
    float voltage, current, power, energy, pf;
    uint32_t pf_freq;
    
    /* Process UART first */
    hlw8032_process_uart(&hlw8032_handle);
    
    /* Read voltage */
    voltage = hlw8032_get_voltage(&hlw8032_handle);
    printf("Voltage: %.3f V\r\n", voltage);
    
    /* Read current */
    current = hlw8032_get_current(&hlw8032_handle);
    printf("Current: %.3f A\r\n", current);
    
    /* Read active power */
    power = hlw8032_get_active_power(&hlw8032_handle);
    printf("Active Power: %.3f W\r\n", power);
    
    /* Read energy */
    energy = hlw8032_get_energy(&hlw8032_handle);
    printf("Energy: %.6f kWh\r\n", energy);
    
    /* Read power factor */
    pf = hlw8032_get_power_factor(&hlw8032_handle);
    printf("Power Factor: %.3f\r\n", pf);
    
    /* Read PF frequency */
    pf_freq = hlw8032_get_pf_frequency(&hlw8032_handle);
    printf("PF Frequency: %lu Hz\r\n", pf_freq);
}

/**
 * Example 3: Read and calculate additional parameters
 */
void HLW8032_Example_CalculateAdditionalParameters(void)
{
    hlw8032_data_t data;
    float apparent_power, reactive_power;
    
    if (hlw8032_read_data(&hlw8032_handle, &data) == HLW8032_OK && data.data_valid) {
        /* Calculate apparent power: S = V * I */
        apparent_power = data.voltage * data.current;
        printf("Apparent Power: %.3f VA\r\n", apparent_power);
        
        /* Calculate reactive power: Q = sqrt(S^2 - P^2) */
        if (apparent_power >= data.active_power) {
            float sqr_diff = (apparent_power * apparent_power) - 
                            (data.active_power * data.active_power);
            if (sqr_diff >= 0) {
                reactive_power = sqrtf(sqr_diff);
                printf("Reactive Power: %.3f VAR\r\n", reactive_power);
            }
        }
        
        /* Calculate power factor from active and apparent power */
        if (apparent_power > 0) {
            float calculated_pf = data.active_power / apparent_power;
            printf("Calculated PF: %.3f\r\n", calculated_pf);
        }
    }
}

/**
 * Example 4: Monitor power consumption
 */
void HLW8032_Example_MonitorPower(void)
{
    static float last_energy = 0.0f;
    hlw8032_data_t data;
    
    if (hlw8032_read_data(&hlw8032_handle, &data) == HLW8032_OK && data.data_valid) {
        /* Calculate energy difference */
        float energy_diff = data.energy - last_energy;
        if (energy_diff < 0) {
            energy_diff = 0;  /* Handle rollover */
        }
        
        printf("Energy consumed since last check: %.6f kWh\r\n", energy_diff);
        printf("Current power consumption: %.3f W\r\n", data.active_power);
        
        last_energy = data.energy;
    }
}

/**
 * Example 5: Check power factor and alert if low
 */
void HLW8032_Example_CheckPowerFactor(void)
{
    float pf = hlw8032_get_power_factor(&hlw8032_handle);
    
    if (pf < 0.8f) {
        printf("WARNING: Low power factor detected: %.3f\r\n", pf);
        printf("Consider adding power factor correction!\r\n");
    } else {
        printf("Power factor OK: %.3f\r\n", pf);
    }
}

/**
 * Example 6: Continuous monitoring loop
 */
void HLW8032_Example_ContinuousMonitoring(void)
{
    uint32_t last_print_time = 0;
    const uint32_t print_interval = 1000;  /* Print every 1 second */
    
    while (1) {
        /* Process UART data continuously */
        hlw8032_process_uart(&hlw8032_handle);
        
        /* Print data periodically */
        if (HAL_GetTick() - last_print_time >= print_interval) {
            HLW8032_Example_ReadAllParameters();
            last_print_time = HAL_GetTick();
        }
        
        /* Small delay to prevent CPU overload */
        HAL_Delay(10);
    }
}

/**
 * Example 7: Read PF frequency separately
 */
void HLW8032_Example_ReadPFFrequency(void)
{
    uint32_t pf_freq;
    
    /* Get PF frequency from timer input capture */
    pf_freq = hlw8032_get_pf_frequency(&hlw8032_handle);
    
    if (pf_freq > 0) {
        printf("PF Pin Frequency: %lu Hz\r\n", pf_freq);
        
        /* Calculate power factor manually */
        float pf = hlw8032_calc_power_factor(pf_freq, 50);  /* 50Hz line frequency */
        printf("Calculated Power Factor: %.3f\r\n", pf);
    } else {
        printf("PF frequency not available\r\n");
    }
}

/**
 * Example 8: Error handling and recovery
 */
void HLW8032_Example_ErrorHandling(void)
{
    hlw8032_status_t status;
    hlw8032_data_t data;
    static uint32_t error_count = 0;
    
    /* Process UART */
    status = hlw8032_process_uart(&hlw8032_handle);
    
    switch (status) {
        case HLW8032_OK:
            /* Success, read data */
            if (hlw8032_read_data(&hlw8032_handle, &data) == HLW8032_OK) {
                printf("Data OK\r\n");
                error_count = 0;
            }
            break;
            
        case HLW8032_CRC_ERROR:
            error_count++;
            printf("CRC Error! Count: %lu\r\n", error_count);
            if (error_count > 10) {
                /* Too many errors, reset */
                hlw8032_reset(&hlw8032_handle);
                error_count = 0;
                printf("HLW8032 reset due to multiple errors\r\n");
            }
            break;
            
        case HLW8032_NOT_READY:
            /* Normal, data not ready yet */
            break;
            
        case HLW8032_ERROR:
        default:
            printf("HLW8032 Error occurred\r\n");
            break;
    }
}

/* ========== Main Usage Example ========== */
/*
void main(void)
{
    // Initialize system
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_UART3_Init();  // HLW8032 UART
    MX_TIM2_Init();   // Timer for PF pin
    
    // Initialize HLW8032
    HLW8032_Example_Init();
    
    // Main loop
    while (1) {
        // Process HLW8032 data
        HLW8032_Example_Process();
        
        // Read and display all parameters
        HLW8032_Example_ReadAllParameters();
        
        // Or read individual parameters
        // HLW8032_Example_ReadIndividualParameters();
        
        // Check power factor
        HLW8032_Example_CheckPowerFactor();
        
        HAL_Delay(1000);  // Wait 1 second
    }
}
*/

