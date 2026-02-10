/******************************************************************************
tmp102.h
TMP102 Temperature Sensor ESP-IDF Component
Converted from SparkFun TMP102 Library by Alex Wende

C API for TMP102 I2C temperature sensor using board_hal abstraction

Original SparkFun library:
https://github.com/sparkfun/Digital_Temperature_Sensor_Breakout_-_TMP102
https://github.com/sparkfun/Temperature_Sensor_TMP102_Qwiic

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef TMP102_H
#define TMP102_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief   Default I2C address for TMP102
 * @details Device ID options: 0x48-GND (default), 0x49-VCC, 0x4A-SDA, 0x4B-SCL
 */
#define TMP102_I2C_ADDR_DEFAULT 0x49U

/**
 * @brief   Conversion rate settings (0-3)
 */
typedef enum
{
    TMP102_CONV_RATE_0_25_HZ = 0U, /**< 0.25 Hz conversion rate */
    TMP102_CONV_RATE_1_HZ = 1U,    /**< 1 Hz conversion rate */
    TMP102_CONV_RATE_4_HZ = 2U,    /**< 4 Hz conversion rate (default) */
    TMP102_CONV_RATE_8_HZ = 3U     /**< 8 Hz conversion rate */
} tmp102_conv_rate_t;

/**
 * @brief   Fault queue settings (consecutive faults before alert)
 */
typedef enum
{
    TMP102_FAULT_1 = 0U, /**< 1 fault triggers alert */
    TMP102_FAULT_2 = 1U, /**< 2 faults trigger alert */
    TMP102_FAULT_4 = 2U, /**< 4 faults trigger alert */
    TMP102_FAULT_6 = 3U  /**< 6 faults trigger alert */
} tmp102_fault_t;

/**
 * @brief   Initialize TMP102 sensor
 * @details Probes I2C bus for device presence at specified address
 *
 * @param[in] dev_addr I2C device address (7-bit), use TMP102_I2C_ADDR_DEFAULT if unsure
 *
 * @return ESP_OK on success
 * @retval ESP_ERR_INVALID_ARG if dev_addr is invalid
 * @retval ESP_ERR_NOT_FOUND if device not detected
 * @retval ESP_FAIL if I2C communication failed
 *
 * @pre    I2C bus must be initialized via board_hal
 * @post   If ESP_OK returned, sensor is ready for operations
 * @post   If error returned, sensor operations will return ESP_ERR_INVALID_STATE
 *
 * @nasa   Rule E1.1 - Return value must be checked
 */
esp_err_t tmp102_init(uint8_t dev_addr);

/**
 * @brief   Deinitialize TMP102 sensor
 * @details Marks sensor as uninitialized, subsequent operations will fail
 *
 * @return ESP_OK on success
 *
 * @post   Sensor is marked as uninitialized
 */
esp_err_t tmp102_deinit(void);

/**
 * @brief   Check if TMP102 sensor is present and responding
 * @details Non-invasive presence check (does not modify state)
 *
 * @return true if sensor is initialized and responding
 * @return false if sensor is not initialized or not responding
 *
 * @safety O(1) operation, bounded I2C timeout
 */
bool tmp102_is_present(void);

/**
 * @brief   Read temperature in degrees Celsius
 * @details Reads 12-bit or 13-bit temperature depending on extended mode
 *
 * @param[out] temp_c Pointer to store temperature in °C
 *
 * @return ESP_OK on success
 * @retval ESP_ERR_INVALID_ARG if temp_c is NULL
 * @retval ESP_ERR_INVALID_STATE if sensor not initialized
 * @retval ESP_ERR_INVALID_RESPONSE if temperature out of range (-55 to +150°C)
 * @retval ESP_FAIL if I2C communication failed
 *
 * @pre    Sensor must be initialized via tmp102_init()
 * @pre    temp_c must not be NULL
 * @post   If ESP_OK returned, temp_c contains valid temperature
 * @post   If error returned, temp_c is unchanged
 *
 * @nasa   Rule E1.1 - Return value must be checked
 * @misra  Rule 1.3 - No NAN propagation (unlike original library)
 */
esp_err_t tmp102_read_temp_c(float* temp_c);

/**
 * @brief   Read temperature in degrees Fahrenheit
 * @details Reads Celsius and converts to Fahrenheit
 *
 * @param[out] temp_f Pointer to store temperature in °F
 *
 * @return ESP_OK on success
 * @retval ESP_ERR_INVALID_ARG if temp_f is NULL
 * @retval ESP_ERR_INVALID_STATE if sensor not initialized
 * @retval ESP_ERR_INVALID_RESPONSE if temperature out of range
 * @retval ESP_FAIL if I2C communication failed
 *
 * @pre    Sensor must be initialized via tmp102_init()
 * @pre    temp_f must not be NULL
 * @post   If ESP_OK returned, temp_f contains valid temperature
 * @post   If error returned, temp_f is unchanged
 */
esp_err_t tmp102_read_temp_f(float* temp_f);

/**
 * @brief   Put sensor in low-power sleep mode
 * @details Sets shutdown (SD) bit in configuration register
 *
 * @return ESP_OK on success
 * @retval ESP_ERR_INVALID_STATE if sensor not initialized
 * @retval ESP_FAIL if I2C communication failed
 *
 * @pre    Sensor must be initialized
 * @post   Sensor in shutdown mode (low power)
 */
esp_err_t tmp102_sleep(void);

/**
 * @brief   Wake sensor from sleep mode
 * @details Clears shutdown (SD) bit in configuration register
 *
 * @return ESP_OK on success
 * @retval ESP_ERR_INVALID_STATE if sensor not initialized
 * @retval ESP_FAIL if I2C communication failed
 *
 * @pre    Sensor must be initialized
 * @post   Sensor in normal operation mode
 */
esp_err_t tmp102_wakeup(void);

/**
 * @brief   Read alert status
 * @details Returns state of alert register (bit 5 of config byte 2)
 *
 * @param[out] alert_state Pointer to store alert state (true=alert active)
 *
 * @return ESP_OK on success
 * @retval ESP_ERR_INVALID_ARG if alert_state is NULL
 * @retval ESP_ERR_INVALID_STATE if sensor not initialized
 * @retval ESP_FAIL if I2C communication failed
 *
 * @pre    Sensor must be initialized
 * @pre    alert_state must not be NULL
 * @post   If ESP_OK returned, alert_state contains current alert status
 */
esp_err_t tmp102_alert(bool* alert_state);

/**
 * @brief   Trigger one-shot conversion or check conversion status
 * @details When set=true, triggers conversion in shutdown mode
 *          When set=false, returns conversion ready status
 *
 * @param[in]  set   true=trigger conversion, false=check status
 * @param[out] ready Pointer to store ready status (true=conversion complete)
 *
 * @return ESP_OK on success
 * @retval ESP_ERR_INVALID_ARG if ready is NULL
 * @retval ESP_ERR_INVALID_STATE if sensor not initialized
 * @retval ESP_FAIL if I2C communication failed
 *
 * @pre    Sensor must be initialized
 * @pre    ready must not be NULL
 * @post   If set=true and ESP_OK: conversion started, ready=false
 * @post   If set=false and ESP_OK: ready indicates conversion status
 */
esp_err_t tmp102_one_shot(bool set, bool* ready);

/**
 * @brief   Set conversion rate
 * @details Configures CR0/CR1 bits (bits 6-7 of config byte 2)
 *
 * @param[in] rate Conversion rate setting (0-3)
 *
 * @return ESP_OK on success
 * @retval ESP_ERR_INVALID_STATE if sensor not initialized
 * @retval ESP_FAIL if I2C communication failed
 *
 * @pre    Sensor must be initialized
 * @post   Conversion rate configured
 */
esp_err_t tmp102_set_conversion_rate(tmp102_conv_rate_t rate);

/**
 * @brief   Enable or disable extended mode
 * @details Extended mode: 13-bit (-55°C to +150°C)
 *          Normal mode: 12-bit (-55°C to +128°C)
 *
 * @param[in] mode true=extended mode, false=normal mode
 *
 * @return ESP_OK on success
 * @retval ESP_ERR_INVALID_STATE if sensor not initialized
 * @retval ESP_FAIL if I2C communication failed
 *
 * @pre    Sensor must be initialized
 * @post   Extended mode configured
 */
esp_err_t tmp102_set_extended_mode(bool mode);

/**
 * @brief   Set alert pin polarity
 * @details Configures POL bit (bit 2 of config byte 1)
 *
 * @param[in] polarity false=active low, true=active high
 *
 * @return ESP_OK on success
 * @retval ESP_ERR_INVALID_STATE if sensor not initialized
 * @retval ESP_FAIL if I2C communication failed
 *
 * @pre    Sensor must be initialized
 * @post   Alert polarity configured
 */
esp_err_t tmp102_set_alert_polarity(bool polarity);

/**
 * @brief   Set fault queue
 * @details Configures F0/F1 bits (bits 3-4 of config byte 1)
 *
 * @param[in] fault Fault queue setting (0-3)
 *
 * @return ESP_OK on success
 * @retval ESP_ERR_INVALID_STATE if sensor not initialized
 * @retval ESP_FAIL if I2C communication failed
 *
 * @pre    Sensor must be initialized
 * @post   Fault queue configured
 */
esp_err_t tmp102_set_fault(tmp102_fault_t fault);

/**
 * @brief   Set alert mode
 * @details Configures TM bit (bit 1 of config byte 1)
 *
 * @param[in] mode false=comparator mode, true=thermostat mode
 *
 * @return ESP_OK on success
 * @retval ESP_ERR_INVALID_STATE if sensor not initialized
 * @retval ESP_FAIL if I2C communication failed
 *
 * @pre    Sensor must be initialized
 * @post   Alert mode configured
 *
 * @note Comparator mode: Active from temp > T_HIGH until temp < T_LOW
 * @note Thermostat mode: Active when temp > T_HIGH until any read operation
 */
esp_err_t tmp102_set_alert_mode(bool mode);

/**
 * @brief   Set low temperature threshold in Celsius
 * @details Writes T_LOW register, clamped to -55°C to +150°C
 *
 * @param[in] temp Temperature threshold in °C
 *
 * @return ESP_OK on success
 * @retval ESP_ERR_INVALID_STATE if sensor not initialized
 * @retval ESP_FAIL if I2C communication failed
 *
 * @pre    Sensor must be initialized
 * @post   T_LOW register updated
 *
 * @note Temperature is clamped to sensor range (-55 to +150°C)
 */
esp_err_t tmp102_set_low_temp_c(float temp);

/**
 * @brief   Set high temperature threshold in Celsius
 * @details Writes T_HIGH register, clamped to -55°C to +150°C
 *
 * @param[in] temp Temperature threshold in °C
 *
 * @return ESP_OK on success
 * @retval ESP_ERR_INVALID_STATE if sensor not initialized
 * @retval ESP_FAIL if I2C communication failed
 *
 * @pre    Sensor must be initialized
 * @post   T_HIGH register updated
 *
 * @note Temperature is clamped to sensor range (-55 to +150°C)
 */
esp_err_t tmp102_set_high_temp_c(float temp);

/**
 * @brief   Set low temperature threshold in Fahrenheit
 * @details Converts to Celsius and writes T_LOW register
 *
 * @param[in] temp Temperature threshold in °F
 *
 * @return ESP_OK on success
 * @retval ESP_ERR_INVALID_STATE if sensor not initialized
 * @retval ESP_FAIL if I2C communication failed
 *
 * @pre    Sensor must be initialized
 * @post   T_LOW register updated
 */
esp_err_t tmp102_set_low_temp_f(float temp);

/**
 * @brief   Set high temperature threshold in Fahrenheit
 * @details Converts to Celsius and writes T_HIGH register
 *
 * @param[in] temp Temperature threshold in °F
 *
 * @return ESP_OK on success
 * @retval ESP_ERR_INVALID_STATE if sensor not initialized
 * @retval ESP_FAIL if I2C communication failed
 *
 * @pre    Sensor must be initialized
 * @post   T_HIGH register updated
 */
esp_err_t tmp102_set_high_temp_f(float temp);

/**
 * @brief   Read low temperature threshold in Celsius
 * @details Reads T_LOW register
 *
 * @param[out] temp Pointer to store temperature in °C
 *
 * @return ESP_OK on success
 * @retval ESP_ERR_INVALID_ARG if temp is NULL
 * @retval ESP_ERR_INVALID_STATE if sensor not initialized
 * @retval ESP_ERR_INVALID_RESPONSE if temperature out of range
 * @retval ESP_FAIL if I2C communication failed
 *
 * @pre    Sensor must be initialized
 * @pre    temp must not be NULL
 * @post   If ESP_OK returned, temp contains T_LOW value
 */
esp_err_t tmp102_read_low_temp_c(float* temp);

/**
 * @brief   Read high temperature threshold in Celsius
 * @details Reads T_HIGH register
 *
 * @param[out] temp Pointer to store temperature in °C
 *
 * @return ESP_OK on success
 * @retval ESP_ERR_INVALID_ARG if temp is NULL
 * @retval ESP_ERR_INVALID_STATE if sensor not initialized
 * @retval ESP_ERR_INVALID_RESPONSE if temperature out of range
 * @retval ESP_FAIL if I2C communication failed
 *
 * @pre    Sensor must be initialized
 * @pre    temp must not be NULL
 * @post   If ESP_OK returned, temp contains T_HIGH value
 */
esp_err_t tmp102_read_high_temp_c(float* temp);

/**
 * @brief   Read low temperature threshold in Fahrenheit
 * @details Reads T_LOW register and converts to Fahrenheit
 *
 * @param[out] temp Pointer to store temperature in °F
 *
 * @return ESP_OK on success
 * @retval ESP_ERR_INVALID_ARG if temp is NULL
 * @retval ESP_ERR_INVALID_STATE if sensor not initialized
 * @retval ESP_ERR_INVALID_RESPONSE if temperature out of range
 * @retval ESP_FAIL if I2C communication failed
 *
 * @pre    Sensor must be initialized
 * @pre    temp must not be NULL
 * @post   If ESP_OK returned, temp contains T_LOW value in °F
 */
esp_err_t tmp102_read_low_temp_f(float* temp);

/**
 * @brief   Read high temperature threshold in Fahrenheit
 * @details Reads T_HIGH register and converts to Fahrenheit
 *
 * @param[out] temp Pointer to store temperature in °F
 *
 * @return ESP_OK on success
 * @retval ESP_ERR_INVALID_ARG if temp is NULL
 * @retval ESP_ERR_INVALID_STATE if sensor not initialized
 * @retval ESP_ERR_INVALID_RESPONSE if temperature out of range
 * @retval ESP_FAIL if I2C communication failed
 *
 * @pre    Sensor must be initialized
 * @pre    temp must not be NULL
 * @post   If ESP_OK returned, temp contains T_HIGH value in °F
 */
esp_err_t tmp102_read_high_temp_f(float* temp);

#ifdef __cplusplus
}
#endif

#endif  // TMP102_H
