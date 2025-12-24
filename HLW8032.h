/*
 * HLW8032.h
 *
 * HLW8032 Energy Metering IC Library
 * Supports UART communication and PF pin input capture
 *
 * Author: Ehsan Zehni
 * Created: 2025
 * 
 * Description: Comprehensive library for HLW8032 energy metering IC
 *              with UART communication and Timer Input Capture support
 */

#ifndef HLW8032_H
#define HLW8032_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* HLW8032 data frame size */
#define HLW8032_FRAME_SIZE 24

/* Configuration constants */
#define HLW8032_DEFAULT_LINE_FREQ 50        /* Default line frequency (Hz) */
#define HLW8032_DEFAULT_VOLTAGE_COEF 0.001f /* Voltage coefficient */
#define HLW8032_DEFAULT_CURRENT_COEF 0.001f /* Current coefficient */
#define HLW8032_DEFAULT_POWER_COEF 0.0001f  /* Power coefficient */
#define HLW8032_DEFAULT_ENERGY_COEF 0.0003125f /* Energy coefficient (1/3200) */

/* Return codes */
typedef enum {
    HLW8032_OK = 0,
    HLW8032_ERROR = -1,
    HLW8032_CRC_ERROR = -2,
    HLW8032_TIMEOUT = -3,
    HLW8032_NOT_READY = -4,
    HLW8032_INVALID_PARAM = -5,
    HLW8032_BUFFER_FULL = -6
} hlw8032_status_t;

/* Measurement data structure */
typedef struct {
    float voltage;          /* Voltage in V */
    float current;          /* Current in A */
    float active_power;    /* Active power in W */
    float energy;          /* Energy in kWh */
    float power_factor;    /* Power factor (0.0 to 1.0) */
    uint32_t pf_frequency; /* PF pin frequency in Hz */
    bool data_valid;       /* Data validity flag */
    uint32_t timestamp;    /* Timestamp of measurement */
} hlw8032_data_t;

/* Extended measurement data with calculated values */
typedef struct {
    hlw8032_data_t basic;      /* Basic measurements */
    float apparent_power;      /* Apparent power in VA */
    float reactive_power;      /* Reactive power in VAR */
    float energy_delta;        /* Energy difference since last read */
    float cost;                /* Energy cost (if rate is set) */
} hlw8032_extended_data_t;

/* Calibration structure */
typedef struct {
    float voltage_offset;      /* Voltage offset */
    float voltage_gain;        /* Voltage gain multiplier */
    float current_offset;       /* Current offset */
    float current_gain;        /* Current gain multiplier */
    float power_offset;        /* Power offset */
    float power_gain;          /* Power gain multiplier */
    float pf_calibration;      /* PF calibration constant */
} hlw8032_calibration_t;

/* Statistics structure */
typedef struct {
    uint32_t total_frames;     /* Total frames received */
    uint32_t valid_frames;     /* Valid frames (CRC OK) */
    uint32_t crc_errors;       /* CRC errors */
    uint32_t sync_errors;      /* Sync errors */
    float max_voltage;         /* Maximum voltage recorded */
    float max_current;         /* Maximum current recorded */
    float max_power;           /* Maximum power recorded */
    float min_voltage;         /* Minimum voltage recorded */
    float min_current;         /* Minimum current recorded */
    float min_power;           /* Minimum power recorded */
    float avg_voltage;         /* Average voltage */
    float avg_current;         /* Average current */
    float avg_power;           /* Average power */
} hlw8032_statistics_t;

/* Callback function types */
typedef void (*hlw8032_frame_callback_t)(hlw8032_handle_t *handle, hlw8032_data_t *data);
typedef void (*hlw8032_error_callback_t)(hlw8032_handle_t *handle, hlw8032_status_t error);

/* Port functions structure - user must implement */
typedef struct {
    /* UART functions */
    int (*uart_receive)(uint8_t *data, uint16_t size, uint32_t timeout);
    int (*uart_get_rx_count)(void);
    void (*uart_clear_rx)(void);
    
    /* Timer input capture functions */
    uint32_t (*timer_get_capture_value)(void);
    uint32_t (*timer_get_frequency)(void);
    void (*timer_reset_capture)(void);
    
    /* Delay function */
    void (*delay_ms)(uint32_t ms);
} hlw8032_port_t;

/* HLW8032 handle structure */
typedef struct {
    hlw8032_port_t *port;
    uint8_t rx_buffer[HLW8032_FRAME_SIZE];
    uint8_t rx_index;
    bool frame_received;
    uint32_t last_update_time;
    hlw8032_data_t data;
    hlw8032_calibration_t calibration;
    hlw8032_statistics_t statistics;
    hlw8032_frame_callback_t frame_callback;
    hlw8032_error_callback_t error_callback;
    uint32_t line_frequency;   /* Line frequency (50 or 60 Hz) */
    float energy_rate;          /* Energy rate for cost calculation */
    float last_energy;         /* Last energy reading for delta calculation */
    bool calibration_enabled; /* Enable/disable calibration */
    uint32_t filter_size;       /* Moving average filter size */
    float *voltage_history;    /* Voltage history for filtering */
    float *current_history;    /* Current history for filtering */
    float *power_history;      /* Power history for filtering */
    uint8_t history_index;     /* Current history index */
} hlw8032_handle_t;

/**
 * Initialize HLW8032 handle
 * @param handle Pointer to handle structure
 * @param port Pointer to port functions structure
 * @return HLW8032_OK on success
 */
hlw8032_status_t hlw8032_init(hlw8032_handle_t *handle, hlw8032_port_t *port);

/**
 * Process received UART data (call from UART RX interrupt or polling)
 * @param handle Pointer to handle structure
 * @return HLW8032_OK when complete frame received
 */
hlw8032_status_t hlw8032_process_uart(hlw8032_handle_t *handle);

/**
 * Read measurement data
 * @param handle Pointer to handle structure
 * @param data Pointer to data structure to fill
 * @return HLW8032_OK on success
 */
hlw8032_status_t hlw8032_read_data(hlw8032_handle_t *handle, hlw8032_data_t *data);

/**
 * Get PF pin frequency from timer input capture
 * @param handle Pointer to handle structure
 * @return Frequency in Hz, 0 on error
 */
uint32_t hlw8032_get_pf_frequency(hlw8032_handle_t *handle);

/**
 * Get voltage value
 * @param handle Pointer to handle structure
 * @return Voltage in V, 0.0 on error
 */
float hlw8032_get_voltage(hlw8032_handle_t *handle);

/**
 * Get current value
 * @param handle Pointer to handle structure
 * @return Current in A, 0.0 on error
 */
float hlw8032_get_current(hlw8032_handle_t *handle);

/**
 * Get active power value
 * @param handle Pointer to handle structure
 * @return Active power in W, 0.0 on error
 */
float hlw8032_get_active_power(hlw8032_handle_t *handle);

/**
 * Get energy value
 * @param handle Pointer to handle structure
 * @return Energy in kWh, 0.0 on error
 */
float hlw8032_get_energy(hlw8032_handle_t *handle);

/**
 * Get power factor value
 * @param handle Pointer to handle structure
 * @return Power factor (0.0 to 1.0), 0.0 on error
 */
float hlw8032_get_power_factor(hlw8032_handle_t *handle);

/**
 * Calculate power factor from PF frequency
 * @param pf_freq PF pin frequency in Hz
 * @param line_freq Line frequency (50 or 60 Hz)
 * @return Power factor (0.0 to 1.0)
 */
float hlw8032_calc_power_factor(uint32_t pf_freq, uint32_t line_freq);

/**
 * Reset handle state
 * @param handle Pointer to handle structure
 */
void hlw8032_reset(hlw8032_handle_t *handle);

/**
 * Set calibration parameters
 * @param handle Pointer to handle structure
 * @param cal Pointer to calibration structure
 * @return HLW8032_OK on success
 */
hlw8032_status_t hlw8032_set_calibration(hlw8032_handle_t *handle, hlw8032_calibration_t *cal);

/**
 * Get calibration parameters
 * @param handle Pointer to handle structure
 * @param cal Pointer to calibration structure to fill
 * @return HLW8032_OK on success
 */
hlw8032_status_t hlw8032_get_calibration(hlw8032_handle_t *handle, hlw8032_calibration_t *cal);

/**
 * Enable/disable calibration
 * @param handle Pointer to handle structure
 * @param enable True to enable, false to disable
 */
void hlw8032_enable_calibration(hlw8032_handle_t *handle, bool enable);

/**
 * Set line frequency
 * @param handle Pointer to handle structure
 * @param freq Line frequency (50 or 60 Hz)
 */
void hlw8032_set_line_frequency(hlw8032_handle_t *handle, uint32_t freq);

/**
 * Set energy rate for cost calculation
 * @param handle Pointer to handle structure
 * @param rate Energy rate (cost per kWh)
 */
void hlw8032_set_energy_rate(hlw8032_handle_t *handle, float rate);

/**
 * Read extended data with calculated values
 * @param handle Pointer to handle structure
 * @param data Pointer to extended data structure
 * @return HLW8032_OK on success
 */
hlw8032_status_t hlw8032_read_extended_data(hlw8032_handle_t *handle, hlw8032_extended_data_t *data);

/**
 * Calculate apparent power
 * @param voltage Voltage in V
 * @param current Current in A
 * @return Apparent power in VA
 */
float hlw8032_calc_apparent_power(float voltage, float current);

/**
 * Calculate reactive power
 * @param apparent_power Apparent power in VA
 * @param active_power Active power in W
 * @return Reactive power in VAR
 */
float hlw8032_calc_reactive_power(float apparent_power, float active_power);

/**
 * Calculate power factor from active and apparent power
 * @param active_power Active power in W
 * @param apparent_power Apparent power in VA
 * @return Power factor (0.0 to 1.0)
 */
float hlw8032_calc_power_factor_from_power(float active_power, float apparent_power);

/**
 * Get statistics
 * @param handle Pointer to handle structure
 * @param stats Pointer to statistics structure
 * @return HLW8032_OK on success
 */
hlw8032_status_t hlw8032_get_statistics(hlw8032_handle_t *handle, hlw8032_statistics_t *stats);

/**
 * Reset statistics
 * @param handle Pointer to handle structure
 */
void hlw8032_reset_statistics(hlw8032_handle_t *handle);

/**
 * Set frame received callback
 * @param handle Pointer to handle structure
 * @param callback Callback function pointer
 */
void hlw8032_set_frame_callback(hlw8032_handle_t *handle, hlw8032_frame_callback_t callback);

/**
 * Set error callback
 * @param handle Pointer to handle structure
 * @param callback Callback function pointer
 */
void hlw8032_set_error_callback(hlw8032_handle_t *handle, hlw8032_error_callback_t callback);

/**
 * Check if data is valid and recent
 * @param handle Pointer to handle structure
 * @param timeout_ms Timeout in milliseconds
 * @return true if data is valid and recent
 */
bool hlw8032_is_data_valid(hlw8032_handle_t *handle, uint32_t timeout_ms);

/**
 * Get raw frame data
 * @param handle Pointer to handle structure
 * @param buffer Pointer to buffer (must be at least HLW8032_FRAME_SIZE)
 * @return HLW8032_OK on success
 */
hlw8032_status_t hlw8032_get_raw_frame(hlw8032_handle_t *handle, uint8_t *buffer);

/**
 * Enable moving average filter
 * @param handle Pointer to handle structure
 * @param filter_size Filter size (0 to disable)
 * @return HLW8032_OK on success, HLW8032_ERROR on failure
 */
hlw8032_status_t hlw8032_enable_filter(hlw8032_handle_t *handle, uint8_t filter_size);

#ifdef __cplusplus
}
#endif

#endif /* HLW8032_H */

