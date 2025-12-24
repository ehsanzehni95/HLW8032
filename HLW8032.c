/*
 * HLW8032.c
 *
 * HLW8032 Energy Metering IC Library Implementation
 * Supports UART communication and PF pin input capture
 *
 * Author: Ehsan Zehni
 * Created: 2025
 * 
 * Description: Complete implementation of HLW8032 library with
 *              calibration, filtering, statistics, and callbacks
 */

#include "HLW8032.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* Calculate CRC8 for HLW8032 */
static uint8_t hlw8032_crc8(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    uint8_t i, j;
    
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* Apply calibration to value */
static float apply_calibration(float value, float offset, float gain)
{
    return (value + offset) * gain;
}

/* Apply moving average filter */
static float apply_filter(float *history, uint8_t size, uint8_t index, float new_value)
{
    if (history == NULL || size == 0) {
        return new_value;
    }
    
    history[index % size] = new_value;
    
    /* Calculate average - use actual filled size if buffer not full yet */
    uint8_t actual_size = (index < size) ? (index + 1) : size;
    float sum = 0.0f;
    uint8_t i;
    for (i = 0; i < actual_size; i++) {
        sum += history[i];
    }
    
    return sum / actual_size;
}

/* Update statistics */
static void update_statistics(hlw8032_handle_t *handle)
{
    hlw8032_statistics_t *stats = &handle->statistics;
    
    stats->total_frames++;
    stats->valid_frames++;
    
    /* Update max values */
    if (handle->data.voltage > stats->max_voltage) {
        stats->max_voltage = handle->data.voltage;
    }
    if (handle->data.current > stats->max_current) {
        stats->max_current = handle->data.current;
    }
    if (handle->data.active_power > stats->max_power) {
        stats->max_power = handle->data.active_power;
    }
    
    /* Update min values */
    if (stats->min_voltage == 0.0f || handle->data.voltage < stats->min_voltage) {
        stats->min_voltage = handle->data.voltage;
    }
    if (stats->min_current == 0.0f || handle->data.current < stats->min_current) {
        stats->min_current = handle->data.current;
    }
    if (stats->min_power == 0.0f || handle->data.active_power < stats->min_power) {
        stats->min_power = handle->data.active_power;
    }
    
    /* Update averages (simple moving average) */
    float alpha = 0.1f;  /* Smoothing factor */
    stats->avg_voltage = stats->avg_voltage * (1.0f - alpha) + handle->data.voltage * alpha;
    stats->avg_current = stats->avg_current * (1.0f - alpha) + handle->data.current * alpha;
    stats->avg_power = stats->avg_power * (1.0f - alpha) + handle->data.active_power * alpha;
}

/* Parse HLW8032 data frame */
static hlw8032_status_t hlw8032_parse_frame(hlw8032_handle_t *handle)
{
    uint8_t *buf = handle->rx_buffer;
    uint32_t temp;
    float voltage_coef = HLW8032_DEFAULT_VOLTAGE_COEF;
    float current_coef = HLW8032_DEFAULT_CURRENT_COEF;
    float power_coef = HLW8032_DEFAULT_POWER_COEF;
    float energy_coef = HLW8032_DEFAULT_ENERGY_COEF;
    
    /* Check frame header (should be 0x55 or 0x68) */
    if (buf[0] != 0x55 && buf[0] != 0x68) {
        handle->statistics.sync_errors++;
        return HLW8032_ERROR;
    }
    
    /* Verify CRC */
    uint8_t calc_crc = hlw8032_crc8(buf, HLW8032_FRAME_SIZE - 1);
    if (calc_crc != buf[HLW8032_FRAME_SIZE - 1]) {
        handle->statistics.crc_errors++;
        return HLW8032_CRC_ERROR;
    }
    
    /* Parse voltage (bytes 2-4, 24-bit) */
    temp = ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 8) | buf[4];
    float raw_voltage = temp * voltage_coef;
    
    /* Parse current (bytes 5-7, 24-bit) */
    temp = ((uint32_t)buf[5] << 16) | ((uint32_t)buf[6] << 8) | buf[7];
    float raw_current = temp * current_coef;
    
    /* Parse active power (bytes 8-10, 24-bit) */
    temp = ((uint32_t)buf[8] << 16) | ((uint32_t)buf[9] << 8) | buf[10];
    float raw_power = temp * power_coef;
    
    /* Parse energy (bytes 11-13, 24-bit) */
    temp = ((uint32_t)buf[11] << 16) | ((uint32_t)buf[12] << 8) | buf[13];
    handle->data.energy = temp * energy_coef;
    
    /* Apply calibration if enabled */
    if (handle->calibration_enabled) {
        raw_voltage = apply_calibration(raw_voltage, 
                                       handle->calibration.voltage_offset,
                                       handle->calibration.voltage_gain);
        raw_current = apply_calibration(raw_current,
                                      handle->calibration.current_offset,
                                      handle->calibration.current_gain);
        raw_power = apply_calibration(raw_power,
                                     handle->calibration.power_offset,
                                     handle->calibration.power_gain);
    }
    
    /* Apply filter if enabled */
    if (handle->filter_size > 0) {
        handle->data.voltage = apply_filter(handle->voltage_history,
                                           handle->filter_size,
                                           handle->history_index,
                                           raw_voltage);
        handle->data.current = apply_filter(handle->current_history,
                                          handle->filter_size,
                                          handle->history_index,
                                          raw_current);
        handle->data.active_power = apply_filter(handle->power_history,
                                                 handle->filter_size,
                                                 handle->history_index,
                                                 raw_power);
        handle->history_index++;
    } else {
        handle->data.voltage = raw_voltage;
        handle->data.current = raw_current;
        handle->data.active_power = raw_power;
    }
    
    /* Get PF frequency from timer */
    handle->data.pf_frequency = hlw8032_get_pf_frequency(handle);
    
    /* Calculate power factor */
    if (handle->data.pf_frequency > 0) {
        handle->data.power_factor = hlw8032_calc_power_factor(
            handle->data.pf_frequency, handle->line_frequency);
    } else {
        /* Calculate from power if PF frequency not available */
        float apparent = hlw8032_calc_apparent_power(handle->data.voltage, handle->data.current);
        if (apparent > 0) {
            handle->data.power_factor = handle->data.active_power / apparent;
        } else {
            handle->data.power_factor = 0.0f;
        }
    }
    
    /* Apply PF calibration if enabled */
    if (handle->calibration_enabled && handle->calibration.pf_calibration != 0.0f) {
        handle->data.power_factor *= handle->calibration.pf_calibration;
        if (handle->data.power_factor > 1.0f) handle->data.power_factor = 1.0f;
        if (handle->data.power_factor < 0.0f) handle->data.power_factor = 0.0f;
    }
    
    handle->data.timestamp = 0; /* Set with system time if available */
    handle->data.data_valid = true;
    
    /* Update statistics */
    update_statistics(handle);
    
    /* Call frame callback if set */
    if (handle->frame_callback != NULL) {
        handle->frame_callback(handle, &handle->data);
    }
    
    return HLW8032_OK;
}

/* Initialize HLW8032 handle */
hlw8032_status_t hlw8032_init(hlw8032_handle_t *handle, hlw8032_port_t *port)
{
    if (handle == NULL || port == NULL) {
        return HLW8032_ERROR;
    }
    
    memset(handle, 0, sizeof(hlw8032_handle_t));
    handle->port = port;
    handle->rx_index = 0;
    handle->frame_received = false;
    handle->data.data_valid = false;
    handle->line_frequency = HLW8032_DEFAULT_LINE_FREQ;
    handle->calibration_enabled = false;
    handle->filter_size = 0;
    handle->voltage_history = NULL;
    handle->current_history = NULL;
    handle->power_history = NULL;
    
    /* Initialize calibration to defaults (no calibration) */
    handle->calibration.voltage_offset = 0.0f;
    handle->calibration.voltage_gain = 1.0f;
    handle->calibration.current_offset = 0.0f;
    handle->calibration.current_gain = 1.0f;
    handle->calibration.power_offset = 0.0f;
    handle->calibration.power_gain = 1.0f;
    handle->calibration.pf_calibration = 1.0f;
    
    return HLW8032_OK;
}

/* Process UART received data - handles complete frame synchronization */
hlw8032_status_t hlw8032_process_uart(hlw8032_handle_t *handle)
{
    if (handle == NULL || handle->port == NULL) {
        return HLW8032_ERROR;
    }
    
    uint16_t rx_count = handle->port->uart_get_rx_count();
    
    if (rx_count == 0) {
        return HLW8032_NOT_READY;
    }
    
    /* Read available bytes */
    while (rx_count > 0) {
        uint8_t byte;
        if (handle->port->uart_receive(&byte, 1, 10) != 0) {
            break;
        }
        rx_count--;
        
        /* If buffer is empty, wait for frame header */
        if (handle->rx_index == 0) {
            /* Look for frame header (0x55 or 0x68) */
            if (byte == 0x55 || byte == 0x68) {
                handle->rx_buffer[handle->rx_index++] = byte;
            }
            /* Ignore bytes until we find header */
        } else {
            /* Store byte in buffer */
            if (handle->rx_index < HLW8032_FRAME_SIZE) {
                handle->rx_buffer[handle->rx_index++] = byte;
            }
            
            /* Check if we have a complete frame */
            if (handle->rx_index >= HLW8032_FRAME_SIZE) {
                /* Parse the complete frame */
                hlw8032_status_t status = hlw8032_parse_frame(handle);
                if (status == HLW8032_OK) {
                    handle->frame_received = true;
                    handle->last_update_time = 0; /* Update with system time if available */
                    handle->rx_index = 0;
                    return HLW8032_OK;
                } else {
                    /* CRC error or parse error, reset and resync */
                    handle->rx_index = 0;
                    
                    /* Call error callback if set */
                    if (handle->error_callback != NULL) {
                        handle->error_callback(handle, status);
                    }
                    
                    return status;
                }
            }
        }
    }
    
    return HLW8032_NOT_READY;
}

/* Read measurement data */
hlw8032_status_t hlw8032_read_data(hlw8032_handle_t *handle, hlw8032_data_t *data)
{
    if (handle == NULL || data == NULL) {
        return HLW8032_ERROR;
    }
    
    /* Process any pending UART data */
    hlw8032_process_uart(handle);
    
    if (!handle->data.data_valid) {
        return HLW8032_NOT_READY;
    }
    
    /* Copy data */
    memcpy(data, &handle->data, sizeof(hlw8032_data_t));
    
    return HLW8032_OK;
}

/* Get PF pin frequency from timer input capture */
uint32_t hlw8032_get_pf_frequency(hlw8032_handle_t *handle)
{
    if (handle == NULL || handle->port == NULL) {
        return 0;
    }
    
    if (handle->port->timer_get_frequency == NULL || 
        handle->port->timer_get_capture_value == NULL) {
        return 0;
    }
    
    /* Get timer capture value (period between two PF pulses) */
    uint32_t capture_period = handle->port->timer_get_capture_value();
    uint32_t timer_freq = handle->port->timer_get_frequency();
    
    if (capture_period == 0 || timer_freq == 0) {
        return 0;
    }
    
    /* Calculate frequency: timer_freq / capture_period */
    /* capture_period is the number of timer ticks between two PF pulses */
    uint32_t pf_freq = timer_freq / capture_period;
    
    return pf_freq;
}

/* Calculate power factor from PF frequency */
float hlw8032_calc_power_factor(uint32_t pf_freq, uint32_t line_freq)
{
    if (pf_freq == 0 || line_freq == 0) {
        return 0.0f;
    }
    
    /* Power factor calculation based on PF pin frequency */
    /* PF frequency is proportional to power factor */
    /* Typical: PF = (PF_freq / (line_freq * constant)) */
    /* Adjust constant based on your HLW8032 configuration */
    float pf = (float)pf_freq / (float)(line_freq * 1000);
    
    /* Clamp to valid range */
    if (pf > 1.0f) {
        pf = 1.0f;
    }
    if (pf < 0.0f) {
        pf = 0.0f;
    }
    
    return pf;
}

/* Get voltage value */
float hlw8032_get_voltage(hlw8032_handle_t *handle)
{
    if (handle == NULL || !handle->data.data_valid) {
        return 0.0f;
    }
    return handle->data.voltage;
}

/* Get current value */
float hlw8032_get_current(hlw8032_handle_t *handle)
{
    if (handle == NULL || !handle->data.data_valid) {
        return 0.0f;
    }
    return handle->data.current;
}

/* Get active power value */
float hlw8032_get_active_power(hlw8032_handle_t *handle)
{
    if (handle == NULL || !handle->data.data_valid) {
        return 0.0f;
    }
    return handle->data.active_power;
}

/* Get energy value */
float hlw8032_get_energy(hlw8032_handle_t *handle)
{
    if (handle == NULL || !handle->data.data_valid) {
        return 0.0f;
    }
    return handle->data.energy;
}

/* Get power factor value */
float hlw8032_get_power_factor(hlw8032_handle_t *handle)
{
    if (handle == NULL || !handle->data.data_valid) {
        return 0.0f;
    }
    return handle->data.power_factor;
}

/* Set calibration parameters */
hlw8032_status_t hlw8032_set_calibration(hlw8032_handle_t *handle, hlw8032_calibration_t *cal)
{
    if (handle == NULL || cal == NULL) {
        return HLW8032_ERROR;
    }
    
    memcpy(&handle->calibration, cal, sizeof(hlw8032_calibration_t));
    return HLW8032_OK;
}

/* Get calibration parameters */
hlw8032_status_t hlw8032_get_calibration(hlw8032_handle_t *handle, hlw8032_calibration_t *cal)
{
    if (handle == NULL || cal == NULL) {
        return HLW8032_ERROR;
    }
    
    memcpy(cal, &handle->calibration, sizeof(hlw8032_calibration_t));
    return HLW8032_OK;
}

/* Enable/disable calibration */
void hlw8032_enable_calibration(hlw8032_handle_t *handle, bool enable)
{
    if (handle != NULL) {
        handle->calibration_enabled = enable;
    }
}

/* Set line frequency */
void hlw8032_set_line_frequency(hlw8032_handle_t *handle, uint32_t freq)
{
    if (handle != NULL) {
        handle->line_frequency = freq;
    }
}

/* Set energy rate */
void hlw8032_set_energy_rate(hlw8032_handle_t *handle, float rate)
{
    if (handle != NULL) {
        handle->energy_rate = rate;
    }
}

/* Read extended data */
hlw8032_status_t hlw8032_read_extended_data(hlw8032_handle_t *handle, hlw8032_extended_data_t *data)
{
    if (handle == NULL || data == NULL) {
        return HLW8032_ERROR;
    }
    
    /* Read basic data first */
    if (hlw8032_read_data(handle, &data->basic) != HLW8032_OK) {
        return HLW8032_NOT_READY;
    }
    
    /* Calculate extended values */
    data->apparent_power = hlw8032_calc_apparent_power(data->basic.voltage, data->basic.current);
    data->reactive_power = hlw8032_calc_reactive_power(data->apparent_power, data->basic.active_power);
    
    /* Calculate energy delta */
    data->energy_delta = data->basic.energy - handle->last_energy;
    if (data->energy_delta < 0) {
        data->energy_delta = 0;  /* Handle rollover */
    }
    handle->last_energy = data->basic.energy;
    
    /* Calculate cost */
    if (handle->energy_rate > 0) {
        data->cost = data->energy_delta * handle->energy_rate;
    } else {
        data->cost = 0.0f;
    }
    
    return HLW8032_OK;
}

/* Calculate apparent power */
float hlw8032_calc_apparent_power(float voltage, float current)
{
    return voltage * current;
}

/* Calculate reactive power */
float hlw8032_calc_reactive_power(float apparent_power, float active_power)
{
    if (apparent_power >= active_power) {
        float sqr_diff = (apparent_power * apparent_power) - (active_power * active_power);
        if (sqr_diff >= 0) {
            return sqrtf(sqr_diff);
        }
    }
    return 0.0f;
}

/* Calculate power factor from power */
float hlw8032_calc_power_factor_from_power(float active_power, float apparent_power)
{
    if (apparent_power > 0) {
        float pf = active_power / apparent_power;
        if (pf > 1.0f) pf = 1.0f;
        if (pf < 0.0f) pf = 0.0f;
        return pf;
    }
    return 0.0f;
}

/* Get statistics */
hlw8032_status_t hlw8032_get_statistics(hlw8032_handle_t *handle, hlw8032_statistics_t *stats)
{
    if (handle == NULL || stats == NULL) {
        return HLW8032_ERROR;
    }
    
    memcpy(stats, &handle->statistics, sizeof(hlw8032_statistics_t));
    return HLW8032_OK;
}

/* Reset statistics */
void hlw8032_reset_statistics(hlw8032_handle_t *handle)
{
    if (handle != NULL) {
        memset(&handle->statistics, 0, sizeof(hlw8032_statistics_t));
    }
}

/* Set frame callback */
void hlw8032_set_frame_callback(hlw8032_handle_t *handle, hlw8032_frame_callback_t callback)
{
    if (handle != NULL) {
        handle->frame_callback = callback;
    }
}

/* Set error callback */
void hlw8032_set_error_callback(hlw8032_handle_t *handle, hlw8032_error_callback_t callback)
{
    if (handle != NULL) {
        handle->error_callback = callback;
    }
}

/* Check if data is valid */
bool hlw8032_is_data_valid(hlw8032_handle_t *handle, uint32_t timeout_ms)
{
    if (handle == NULL) {
        return false;
    }
    
    if (!handle->data.data_valid) {
        return false;
    }
    
    /* Check timeout if timestamp is available */
    if (handle->data.timestamp > 0 && timeout_ms > 0) {
        uint32_t current_time = 0; /* Get from system if available */
        if (current_time - handle->data.timestamp > timeout_ms) {
            return false;
        }
    }
    
    return true;
}

/* Get raw frame */
hlw8032_status_t hlw8032_get_raw_frame(hlw8032_handle_t *handle, uint8_t *buffer)
{
    if (handle == NULL || buffer == NULL) {
        return HLW8032_ERROR;
    }
    
    memcpy(buffer, handle->rx_buffer, HLW8032_FRAME_SIZE);
    return HLW8032_OK;
}

/* Enable filter */
hlw8032_status_t hlw8032_enable_filter(hlw8032_handle_t *handle, uint8_t filter_size)
{
    if (handle == NULL) {
        return HLW8032_ERROR;
    }
    
    /* Free existing buffers */
    if (handle->voltage_history != NULL) {
        free(handle->voltage_history);
        handle->voltage_history = NULL;
    }
    if (handle->current_history != NULL) {
        free(handle->current_history);
        handle->current_history = NULL;
    }
    if (handle->power_history != NULL) {
        free(handle->power_history);
        handle->power_history = NULL;
    }
    
    if (filter_size == 0) {
        handle->filter_size = 0;
        return HLW8032_OK;
    }
    
    /* Allocate memory for filter */
    handle->voltage_history = (float *)malloc(filter_size * sizeof(float));
    handle->current_history = (float *)malloc(filter_size * sizeof(float));
    handle->power_history = (float *)malloc(filter_size * sizeof(float));
    
    if (handle->voltage_history == NULL || 
        handle->current_history == NULL || 
        handle->power_history == NULL) {
        /* Free what was allocated */
        if (handle->voltage_history != NULL) free(handle->voltage_history);
        if (handle->current_history != NULL) free(handle->current_history);
        if (handle->power_history != NULL) free(handle->power_history);
        handle->voltage_history = NULL;
        handle->current_history = NULL;
        handle->power_history = NULL;
        return HLW8032_ERROR;
    }
    
    /* Initialize filter buffers */
    memset(handle->voltage_history, 0, filter_size * sizeof(float));
    memset(handle->current_history, 0, filter_size * sizeof(float));
    memset(handle->power_history, 0, filter_size * sizeof(float));
    
    handle->filter_size = filter_size;
    handle->history_index = 0;
    
    return HLW8032_OK;
}

/* Reset handle state */
void hlw8032_reset(hlw8032_handle_t *handle)
{
    if (handle == NULL) {
        return;
    }
    
    handle->rx_index = 0;
    handle->frame_received = false;
    handle->data.data_valid = false;
    
    if (handle->port != NULL && handle->port->uart_clear_rx != NULL) {
        handle->port->uart_clear_rx();
    }
    
    if (handle->port != NULL && handle->port->timer_reset_capture != NULL) {
        handle->port->timer_reset_capture();
    }
}

