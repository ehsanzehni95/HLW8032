# HLW8032 Energy Metering IC Library

Comprehensive library for HLW8032 energy metering IC with UART communication and Timer Input Capture support.

## Description

A complete, production-ready library for interfacing with HLW8032 energy metering IC. Features full UART frame handling, Timer Input Capture for PF pin frequency measurement, automatic power factor calculation, calibration support, filtering, statistics, and callback functions. Designed for embedded systems with modular architecture for easy portability.

## Features

### Core Features
- ✅ Complete UART communication for 24-byte frame reception
- ✅ Timer Input Capture support for PF pin frequency measurement
- ✅ Automatic Power Factor calculation
- ✅ CRC validation with error management
- ✅ Automatic frame synchronization (Auto-sync)
- ✅ Modular and portable architecture

### Advanced Features
- ✅ **Calibration**: Full support for voltage, current, power, and PF calibration
- ✅ **Filtering**: Moving Average Filter for noise reduction
- ✅ **Statistics**: Complete statistics collection (max, min, average, error counts)
- ✅ **Callback Functions**: Support for frame and error callbacks
- ✅ **Advanced Calculations**: Apparent power, reactive power, energy cost calculation
- ✅ **Error Management**: Complete error handling with callbacks
- ✅ **Data Validation**: Data validity and freshness checking
- ✅ **Raw Data Access**: Access to raw frame for debugging

## Hardware Connections

- **TX (HLW8032)**: Connected to microcontroller UART RX
- **PF (HLW8032)**: Connected to Timer Input Capture

## File Structure

- `HLW8032.h`: Header file with all definitions and functions
- `HLW8032.c`: Complete library implementation with full 24-byte frame handling
- `HLW8032_Port_Example.c`: Port functions implementation example for STM32
- `HLW8032_Example.c`: Complete usage example with all parameter reading methods
- `HLW8032_InputCapture_Guide.md`: Complete Input Capture guide for PF pin
- `HLW8032_Integration_Example.c`: STM32 integration example

## Quick Start

### 1. Implement Port Functions

Implement port functions according to your hardware:

```c
hlw8032_port_t hlw8032_port = {
    .uart_receive = your_uart_receive,
    .uart_get_rx_count = your_uart_get_count,
    .uart_clear_rx = your_uart_clear,
    .timer_get_capture_value = your_timer_get_capture,
    .timer_get_frequency = your_timer_get_freq,
    .timer_reset_capture = your_timer_reset,
    .delay_ms = your_delay
};
```

### 2. Initialize

```c
hlw8032_handle_t hlw8032;
hlw8032_init(&hlw8032, &hlw8032_port);
hlw8032_set_line_frequency(&hlw8032, 50);  // 50Hz or 60Hz
```

### 3. Process Data

In main loop or interrupt:

```c
hlw8032_process_uart(&hlw8032);
```

### 4. Read Data

```c
hlw8032_data_t data;
if (hlw8032_read_data(&hlw8032, &data) == HLW8032_OK) {
    float voltage = data.voltage;        // Voltage (V)
    float current = data.current;        // Current (A)
    float power = data.active_power;     // Power (W)
    float energy = data.energy;          // Energy (kWh)
    float pf = data.power_factor;        // Power Factor
    uint32_t pf_freq = data.pf_frequency; // PF frequency (Hz)
}
```

## UART Configuration

- Baud Rate: 4800
- Data Bits: 8
- Stop Bits: 1
- Parity: None
- Frame Size: 24 bytes

## Timer Input Capture Configuration for PF Pin

The PF pin in HLW8032 outputs pulses with frequency proportional to Power Factor.

### Configuration Steps:

1. **In STM32CubeMX**:
   - Select a Timer (e.g., TIM2)
   - Configure Channel as Input Capture
   - Connect PF pin to Input Capture input
   - Set Prescaler (e.g., for 1MHz: Prescaler = 84)

2. **Implement Callback**:
   ```c
   void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
   {
       if (htim->Instance == TIM2) {
           HLW8032_TIM_IC_CaptureCallback(htim);
       }
   }
   ```

3. **Start Timer**:
   ```c
   HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
   ```

**Complete Guide**: See `HLW8032_InputCapture_Guide.md` for:
- Complete Timer configuration
- Two implementation methods (precise and simple)
- Complete code examples
- Debugging tips

## HLW8032 Data Format

24-byte frame includes:
- Byte 0: Header (0x55 or 0x68)
- Bytes 2-4: Voltage (24-bit)
- Bytes 5-7: Current (24-bit)
- Bytes 8-10: Active Power (24-bit)
- Bytes 11-13: Energy (24-bit)
- Byte 23: CRC8

## Main Functions

### Basic Functions
- `hlw8032_init()`: Initialize handle
- `hlw8032_process_uart()`: Process UART data (handles complete 24-byte frame)
- `hlw8032_read_data()`: Read all measurement data in one structure
- `hlw8032_reset()`: Reset handle

### Individual Parameter Reading
- `hlw8032_get_voltage()`: Read voltage (V)
- `hlw8032_get_current()`: Read current (A)
- `hlw8032_get_active_power()`: Read active power (W)
- `hlw8032_get_energy()`: Read energy (kWh)
- `hlw8032_get_power_factor()`: Read power factor
- `hlw8032_get_pf_frequency()`: Get PF pin frequency from timer
- `hlw8032_read_extended_data()`: Read advanced data (apparent power, reactive power, cost)

### Calibration Functions
- `hlw8032_set_calibration()`: Set calibration parameters
- `hlw8032_get_calibration()`: Get calibration parameters
- `hlw8032_enable_calibration()`: Enable/disable calibration

### Filter and Processing Functions
- `hlw8032_enable_filter()`: Enable Moving Average Filter
- `hlw8032_calc_apparent_power()`: Calculate apparent power
- `hlw8032_calc_reactive_power()`: Calculate reactive power
- `hlw8032_calc_power_factor_from_power()`: Calculate PF from power

### Statistics and Monitoring Functions
- `hlw8032_get_statistics()`: Get complete statistics
- `hlw8032_reset_statistics()`: Reset statistics
- `hlw8032_is_data_valid()`: Check data validity

### Callback Functions
- `hlw8032_set_frame_callback()`: Set callback for new frame
- `hlw8032_set_error_callback()`: Set callback for errors

### Configuration Functions
- `hlw8032_set_line_frequency()`: Set line frequency (50/60 Hz)
- `hlw8032_set_energy_rate()`: Set energy rate for cost calculation
- `hlw8032_get_raw_frame()`: Get raw frame

## Return Codes

- `HLW8032_OK`: Success
- `HLW8032_ERROR`: General error
- `HLW8032_CRC_ERROR`: CRC error
- `HLW8032_TIMEOUT`: Timeout
- `HLW8032_NOT_READY`: Data not ready

## Usage Examples

### Example 1: Initialization with Calibration and Filter

```c
hlw8032_handle_t hlw8032;
hlw8032_calibration_t cal;

// Initialize
hlw8032_init(&hlw8032, &hlw8032_port);

// Setup calibration
cal.voltage_gain = 1.02f;  // 2% correction
cal.current_gain = 0.98f;   // -2% correction
cal.power_gain = 1.0f;
hlw8032_set_calibration(&hlw8032, &cal);
hlw8032_enable_calibration(&hlw8032, true);

// Enable filter (10 samples)
hlw8032_enable_filter(&hlw8032, 10);

// Set line frequency
hlw8032_set_line_frequency(&hlw8032, 50);

// Set energy rate for cost calculation
hlw8032_set_energy_rate(&hlw8032, 0.15f);  // 0.15 per kWh
```

### Example 2: Using Callbacks

```c
// Frame callback
void on_new_frame(hlw8032_handle_t *handle, hlw8032_data_t *data) {
    printf("New frame: V=%.2fV, I=%.2fA, P=%.2fW\n", 
           data->voltage, data->current, data->active_power);
}

// Error callback
void on_error(hlw8032_handle_t *handle, hlw8032_status_t error) {
    if (error == HLW8032_CRC_ERROR) {
        printf("CRC Error!\n");
    }
}

// Setup callbacks
hlw8032_set_frame_callback(&hlw8032, on_new_frame);
hlw8032_set_error_callback(&hlw8032, on_error);
```

### Example 3: Reading All Parameters

```c
hlw8032_data_t data;
if (hlw8032_read_data(&hlw8032_handle, &data) == HLW8032_OK) {
    printf("Voltage: %.3f V\r\n", data.voltage);
    printf("Current: %.3f A\r\n", data.current);
    printf("Power: %.3f W\r\n", data.active_power);
    printf("Energy: %.6f kWh\r\n", data.energy);
    printf("Power Factor: %.3f\r\n", data.power_factor);
    printf("PF Frequency: %lu Hz\r\n", data.pf_frequency);
}
```

### Example 4: Reading Extended Data

```c
hlw8032_extended_data_t ext_data;
if (hlw8032_read_extended_data(&hlw8032, &ext_data) == HLW8032_OK) {
    printf("Voltage: %.3f V\n", ext_data.basic.voltage);
    printf("Current: %.3f A\n", ext_data.basic.current);
    printf("Active Power: %.3f W\n", ext_data.basic.active_power);
    printf("Apparent Power: %.3f VA\n", ext_data.apparent_power);
    printf("Reactive Power: %.3f VAR\n", ext_data.reactive_power);
    printf("Energy Delta: %.6f kWh\n", ext_data.energy_delta);
    printf("Cost: %.2f\n", ext_data.cost);
}
```

### Example 5: Getting Statistics

```c
hlw8032_statistics_t stats;
hlw8032_get_statistics(&hlw8032, &stats);

printf("Total Frames: %lu\n", stats.total_frames);
printf("Valid Frames: %lu\n", stats.valid_frames);
printf("CRC Errors: %lu\n", stats.crc_errors);
printf("Max Voltage: %.2f V\n", stats.max_voltage);
printf("Max Current: %.2f A\n", stats.max_current);
printf("Max Power: %.2f W\n", stats.max_power);
printf("Avg Voltage: %.2f V\n", stats.avg_voltage);
printf("Avg Current: %.2f A\n", stats.avg_current);
printf("Avg Power: %.2f W\n", stats.avg_power);
```

## Data Structures

### hlw8032_data_t
Main structure for measurement data

### hlw8032_extended_data_t
Advanced structure with additional calculations (apparent power, reactive power, cost)

### hlw8032_calibration_t
Calibration structure for precise adjustment

### hlw8032_statistics_t
Statistics structure for performance monitoring

## Important Notes

1. **Calibration**: Use calibration for higher accuracy
2. **Filter**: Enable Moving Average Filter to reduce noise
3. **Callback**: Use callbacks for asynchronous processing
4. **Statistics**: Use statistics for system performance monitoring
5. **Error Handling**: Always check and handle errors
6. **Thread Safety**: Use mutex in RTOS environment

## Library Files

- **HLW8032.h**: Main header file with all definitions
- **HLW8032.c**: Complete library implementation with Input Capture support
- **HLW8032_Port_Example.c**: Port functions implementation example
- **HLW8032_Example.c**: Complete example with 8+ different methods
- **HLW8032_InputCapture_Guide.md**: Complete Input Capture guide for PF pin
- **HLW8032_Integration_Example.c**: STM32 integration example

## License

[Add your license here]

## Author

Ehsan Zehni
