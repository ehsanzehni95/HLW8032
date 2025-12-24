# Complete Input Capture Guide for PF Pin in HLW8032

## 📌 Introduction

The PF pin in HLW8032 IC outputs pulses with frequency proportional to Power Factor. To accurately measure this frequency, we need to use Timer Input Capture.

## 🔧 Timer Configuration for Input Capture

### 1. Configuration in STM32CubeMX

1. **Select Timer**: Choose an appropriate Timer (e.g., TIM2, TIM3, TIM4)
2. **Input Capture Mode**: 
   - Set Channel to Input Capture
   - Mode: Input Capture direct mode
   - Polarity: Rising Edge (or Falling Edge as needed)
3. **Prescaler**: Configure so timer frequency is appropriate
   - Example: If Timer Clock = 84MHz and Prescaler = 84, Timer frequency = 1MHz
4. **Period**: Maximum timer value (Auto-reload register)
   - For accurate measurement, choose a large Period value

### 2. TIM2 Configuration Example

```c
// In stm32f4xx_hal_msp.c or configuration file

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* htim_ic)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    if(htim_ic->Instance == TIM2)
    {
        /* Peripheral clock enable */
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        
        /* TIM2 GPIO Configuration
           PA0 (TIM2_CH1) ------> PF pin from HLW8032
        */
        GPIO_InitStruct.Pin = GPIO_PIN_0;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        
        /* TIM2 interrupt Init */
        HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }
}
```

### 3. Timer Configuration

```c
// In tim.c or configuration file

TIM_HandleTypeDef htim2;

void MX_TIM2_Init(void)
{
    TIM_IC_InitTypeDef sConfigIC = {0};
    
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 84 - 1;        // Prescaler: 84 (84MHz / 84 = 1MHz)
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFF;        // Maximum period
    htim2.Init.ClockDivision = TIM_CLKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    
    if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}
```

## 📝 Port Functions Implementation

### Method 1: Using Two Captures (More Precise)

```c
// Global variables
static volatile uint32_t timer_capture1 = 0;
static volatile uint32_t timer_capture2 = 0;
static volatile uint32_t timer_period = 0;
static volatile bool capture_ready = false;

// Callback in HAL_TIM_IC_CaptureCallback
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        static uint32_t last_capture = 0;
        uint32_t current_capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        
        if (last_capture != 0)
        {
            // Calculate period between two captures
            if (current_capture > last_capture)
            {
                timer_period = current_capture - last_capture;
            }
            else
            {
                // Handle overflow
                timer_period = (0xFFFFFFFF - last_capture) + current_capture;
            }
            capture_ready = true;
        }
        
        last_capture = current_capture;
        
        // Restart capture
        __HAL_TIM_SET_COUNTER(htim, 0);
        HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_1);
    }
}

// get_capture_value function
static uint32_t timer_get_capture_value(void)
{
    if (capture_ready)
    {
        capture_ready = false;
        return timer_period;
    }
    return 0;
}

// get_frequency function
static uint32_t timer_get_frequency(void)
{
    // Timer frequency = APB1 Timer Clock / Prescaler
    // If APB1 = 42MHz and Prescaler = 84, Timer Freq = 1MHz
    return 1000000;  // 1MHz
}
```

### Method 2: Using Single Capture (Simpler)

```c
// Global variables
static volatile uint32_t timer_capture_value = 0;
static volatile uint32_t timer_counter = 0;

// Callback
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        // Read capture value
        timer_capture_value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        
        // Reset counter for next capture
        __HAL_TIM_SET_COUNTER(htim, 0);
        
        // Restart capture
        HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_1);
    }
}

// Period Elapsed Callback (optional)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        timer_counter++;
    }
}

// get_capture_value function
static uint32_t timer_get_capture_value(void)
{
    return timer_capture_value;
}

// get_frequency function
static uint32_t timer_get_frequency(void)
{
    // Timer frequency = System Clock / Prescaler
    // Example: 84MHz / 84 = 1MHz
    return 1000000;  // Adjust based on your configuration
}

// reset_capture function
static void timer_reset_capture(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    timer_capture_value = 0;
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
}
```

## 🚀 Setup and Usage

### 1. Initialization

```c
void HLW8032_Init_With_InputCapture(void)
{
    // Initialize Timer for Input Capture
    MX_TIM2_Init();
    
    // Start Input Capture in interrupt mode
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    
    // Initialize HLW8032 library
    hlw8032_init(&hlw8032_handle, &hlw8032_port);
    
    // Set line frequency
    hlw8032_set_line_frequency(&hlw8032_handle, 50);  // 50Hz
}
```

### 2. Usage in Main Loop

```c
void main_loop(void)
{
    hlw8032_data_t data;
    
    // Process UART data
    hlw8032_process_uart(&hlw8032_handle);
    
    // Read data (PF frequency will be read from timer automatically)
    if (hlw8032_read_data(&hlw8032_handle, &data) == HLW8032_OK)
    {
        printf("Voltage: %.2f V\n", data.voltage);
        printf("Current: %.2f A\n", data.current);
        printf("Power Factor: %.3f\n", data.power_factor);
        printf("PF Frequency: %lu Hz\n", data.pf_frequency);
    }
    
    HAL_Delay(100);
}
```

## 📊 Power Factor Calculation from PF Frequency

The PF pin frequency in HLW8032 is related to Power Factor. General formula:

```
PF = (PF_Frequency / (Line_Frequency * Constant))
```

The Constant value varies depending on HLW8032 configuration. Typically:
- For PF = 1.0, PF frequency ≈ Line_Frequency × 1000
- For PF = 0.5, PF frequency ≈ Line_Frequency × 500

The library automatically performs this calculation, but you can adjust it with calibration.

## ⚠️ Important Notes

1. **Prescaler**: Configure Prescaler for sufficient accuracy
   - For low PF frequencies, smaller Prescaler = higher accuracy
   
2. **Period**: Timer Period must be larger than maximum expected period

3. **Filter**: You can use Input Filter in Timer to reduce noise

4. **Interrupt Priority**: Set appropriate interrupt priority for Timer

5. **Overflow**: Handle timer overflow if it occurs

## 🔍 Debugging

For debugging, you can print raw values:

```c
uint32_t capture = timer_get_capture_value();
uint32_t timer_freq = timer_get_frequency();
uint32_t pf_freq = timer_freq / capture;

printf("Capture Value: %lu\n", capture);
printf("Timer Frequency: %lu Hz\n", timer_freq);
printf("PF Frequency: %lu Hz\n", pf_freq);
```

## 📝 Complete Example

Refer to `HLW8032_Example.c` file which includes complete Input Capture example.
