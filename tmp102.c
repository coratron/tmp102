/******************************************************************************
tmp102.c
TMP102 Temperature Sensor ESP-IDF Component
Converted from SparkFun TMP102 Library by Alex Wende

C implementation for TMP102 I2C temperature sensor using board_hal abstraction

Original SparkFun library:
https://github.com/sparkfun/Digital_Temperature_Sensor_Breakout_-_TMP102
https://github.com/sparkfun/Temperature_Sensor_TMP102_Qwiic

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#include "tmp102.h"

#include <string.h>

#include "esp_log.h"
#include "hal_i2c.h"

/* Register definitions */
#define TMP102_REG_TEMPERATURE 0x00U
#define TMP102_REG_CONFIG      0x01U
#define TMP102_REG_T_LOW       0x02U
#define TMP102_REG_T_HIGH      0x03U

/* Temperature range limits */
#define TMP102_TEMP_MIN_C -55.0f
#define TMP102_TEMP_MAX_C 150.0f

/* I2C communication timeout */
#define TMP102_I2C_TIMEOUT_MS 100U

/* Module state */
static const char* TAG = "tmp102";
static uint8_t g_dev_addr = 0U;
static bool g_initialized = false;

/* Helper function prototypes */
static esp_err_t open_pointer_register(uint8_t reg);
static esp_err_t read_register_byte(uint8_t byte_index, uint8_t* value);
static esp_err_t read_temperature_raw(float* temperature);
static esp_err_t write_temperature_threshold(uint8_t reg, float temperature);

/* Public API implementation */

esp_err_t tmp102_init(uint8_t dev_addr)
{
    /* Validate I2C address (7-bit range) */
    if((dev_addr < 0x03U) || (dev_addr > 0x77U))
    {
        ESP_LOGE(TAG, "Invalid I2C address: 0x%02X", dev_addr);
        return ESP_ERR_INVALID_ARG;
    }

    /* Store address */
    g_dev_addr = dev_addr;

    /* Probe device */
    hal_err_t ret = board_hal_i2c_probe(HAL_I2C_PORT_0, g_dev_addr);
    if(ret != HAL_OK)
    {
        ESP_LOGE(TAG, "Device not found at address 0x%02X", g_dev_addr);
        g_initialized = false;
        return ESP_ERR_NOT_FOUND;
    }

    g_initialized = true;
    ESP_LOGI(TAG, "TMP102 initialized at address 0x%02X", g_dev_addr);
    return ESP_OK;
}

esp_err_t tmp102_deinit(void)
{
    g_initialized = false;
    g_dev_addr = 0U;
    ESP_LOGI(TAG, "TMP102 deinitialized");
    return ESP_OK;
}

bool tmp102_is_present(void)
{
    if(!g_initialized)
    {
        return false;
    }

    /* Probe device to confirm presence */
    hal_err_t ret = board_hal_i2c_probe(HAL_I2C_PORT_0, g_dev_addr);
    return (ret == HAL_OK);
}

esp_err_t tmp102_read_temp_c(float* temp_c)
{
    /* Parameter validation */
    if(temp_c == NULL)
    {
        ESP_LOGE(TAG, "temp_c is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    /* State validation */
    if(!g_initialized)
    {
        ESP_LOGE(TAG, "Sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Read temperature */
    return read_temperature_raw(temp_c);
}

esp_err_t tmp102_read_temp_f(float* temp_f)
{
    esp_err_t ret;
    float temp_c = 0.0f;

    /* Parameter validation */
    if(temp_f == NULL)
    {
        ESP_LOGE(TAG, "temp_f is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    /* State validation */
    if(!g_initialized)
    {
        ESP_LOGE(TAG, "Sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Read temperature in Celsius */
    ret = read_temperature_raw(&temp_c);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* Convert to Fahrenheit */
    *temp_f = (temp_c * 9.0f / 5.0f) + 32.0f;
    return ESP_OK;
}

esp_err_t tmp102_sleep(void)
{
    esp_err_t ret;
    uint8_t config_byte = 0U;

    /* State validation */
    if(!g_initialized)
    {
        ESP_LOGE(TAG, "Sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Set pointer to config register */
    ret = open_pointer_register(TMP102_REG_CONFIG);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* Read first byte of config register */
    ret = read_register_byte(0U, &config_byte);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* Set SD (shutdown) bit */
    config_byte |= 0x01U;

    /* Write back config register */
    uint8_t write_buf[2] = {TMP102_REG_CONFIG, config_byte};
    hal_err_t hal_ret = board_hal_i2c_write(HAL_I2C_PORT_0, g_dev_addr, write_buf, 2U, TMP102_I2C_TIMEOUT_MS);

    if(hal_ret != HAL_OK)
    {
        ESP_LOGE(TAG, "Failed to enter sleep mode");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t tmp102_wakeup(void)
{
    esp_err_t ret;
    uint8_t config_byte = 0U;

    /* State validation */
    if(!g_initialized)
    {
        ESP_LOGE(TAG, "Sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Set pointer to config register */
    ret = open_pointer_register(TMP102_REG_CONFIG);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* Read first byte of config register */
    ret = read_register_byte(0U, &config_byte);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* Clear SD (shutdown) bit */
    config_byte &= 0xFEU;

    /* Write back config register */
    uint8_t write_buf[2] = {TMP102_REG_CONFIG, config_byte};
    hal_err_t hal_ret = board_hal_i2c_write(HAL_I2C_PORT_0, g_dev_addr, write_buf, 2U, TMP102_I2C_TIMEOUT_MS);

    if(hal_ret != HAL_OK)
    {
        ESP_LOGE(TAG, "Failed to wake from sleep mode");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t tmp102_alert(bool* alert_state)
{
    esp_err_t ret;
    uint8_t config_byte = 0U;

    /* Parameter validation */
    if(alert_state == NULL)
    {
        ESP_LOGE(TAG, "alert_state is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    /* State validation */
    if(!g_initialized)
    {
        ESP_LOGE(TAG, "Sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Set pointer to config register */
    ret = open_pointer_register(TMP102_REG_CONFIG);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* Read second byte of config register */
    ret = read_register_byte(1U, &config_byte);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* Extract alert bit (bit 5) */
    *alert_state = ((config_byte & 0x20U) != 0U);
    return ESP_OK;
}

esp_err_t tmp102_one_shot(bool set, bool* ready)
{
    esp_err_t ret;
    uint8_t config_byte = 0U;

    /* Parameter validation */
    if(ready == NULL)
    {
        ESP_LOGE(TAG, "ready is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    /* State validation */
    if(!g_initialized)
    {
        ESP_LOGE(TAG, "Sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Read first byte of config register using write-read */
    uint8_t reg = TMP102_REG_CONFIG;
    hal_err_t hal_ret = board_hal_i2c_write_read(HAL_I2C_PORT_0, g_dev_addr, &reg, 1U, &config_byte, 1U, TMP102_I2C_TIMEOUT_MS);

    if(hal_ret != HAL_OK)
    {
        ESP_LOGE(TAG, "Failed to read config register");
        return ESP_FAIL;
    }

    if(set)
    {
        /* Enable one-shot by writing 1 to OS bit (bit 7) */
        config_byte |= (1U << 7U);

        /* Write back config register */
        uint8_t write_buf[2] = {TMP102_REG_CONFIG, config_byte};
        hal_ret = board_hal_i2c_write(HAL_I2C_PORT_0, g_dev_addr, write_buf, 2U, TMP102_I2C_TIMEOUT_MS);

        if(hal_ret != HAL_OK)
        {
            ESP_LOGE(TAG, "Failed to set one-shot mode");
            return ESP_FAIL;
        }

        *ready = false;
    }
    else
    {
        /* Return OS bit status (0=not ready, 1=conversion complete) */
        *ready = ((config_byte & (1U << 7U)) != 0U);
    }

    return ESP_OK;
}

esp_err_t tmp102_set_conversion_rate(tmp102_conv_rate_t rate)
{
    esp_err_t ret;
    uint8_t config_bytes[2] = {0U, 0U};

    /* State validation */
    if(!g_initialized)
    {
        ESP_LOGE(TAG, "Sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Clamp rate to valid range (0-3) */
    uint8_t rate_val = (uint8_t)rate & 0x03U;

    /* Set pointer to config register */
    ret = open_pointer_register(TMP102_REG_CONFIG);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* Read both config bytes */
    ret = read_register_byte(0U, &config_bytes[0]);
    if(ret != ESP_OK)
    {
        return ret;
    }
    ret = read_register_byte(1U, &config_bytes[1]);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* Clear CR0/CR1 bits (bits 6-7 of second byte) and set new rate */
    config_bytes[1] &= 0x3FU;
    config_bytes[1] |= (rate_val << 6U);

    /* Write back config register */
    uint8_t write_buf[3] = {TMP102_REG_CONFIG, config_bytes[0], config_bytes[1]};
    hal_err_t hal_ret = board_hal_i2c_write(HAL_I2C_PORT_0, g_dev_addr, write_buf, 3U, TMP102_I2C_TIMEOUT_MS);

    if(hal_ret != HAL_OK)
    {
        ESP_LOGE(TAG, "Failed to set conversion rate");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t tmp102_set_extended_mode(bool mode)
{
    esp_err_t ret;
    uint8_t config_bytes[2] = {0U, 0U};

    /* State validation */
    if(!g_initialized)
    {
        ESP_LOGE(TAG, "Sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Set pointer to config register */
    ret = open_pointer_register(TMP102_REG_CONFIG);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* Read both config bytes */
    ret = read_register_byte(0U, &config_bytes[0]);
    if(ret != ESP_OK)
    {
        return ret;
    }
    ret = read_register_byte(1U, &config_bytes[1]);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* Clear EM bit (bit 4 of second byte) and set new mode */
    config_bytes[1] &= 0xEFU;
    if(mode)
    {
        config_bytes[1] |= (1U << 4U);
    }

    /* Write back config register */
    uint8_t write_buf[3] = {TMP102_REG_CONFIG, config_bytes[0], config_bytes[1]};
    hal_err_t hal_ret = board_hal_i2c_write(HAL_I2C_PORT_0, g_dev_addr, write_buf, 3U, TMP102_I2C_TIMEOUT_MS);

    if(hal_ret != HAL_OK)
    {
        ESP_LOGE(TAG, "Failed to set extended mode");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t tmp102_set_alert_polarity(bool polarity)
{
    esp_err_t ret;
    uint8_t config_byte = 0U;

    /* State validation */
    if(!g_initialized)
    {
        ESP_LOGE(TAG, "Sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Set pointer to config register */
    ret = open_pointer_register(TMP102_REG_CONFIG);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* Read first byte of config register */
    ret = read_register_byte(0U, &config_byte);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* Clear POL bit (bit 2) and set new polarity */
    config_byte &= 0xFBU;
    if(polarity)
    {
        config_byte |= (1U << 2U);
    }

    /* Write back config register */
    uint8_t write_buf[2] = {TMP102_REG_CONFIG, config_byte};
    hal_err_t hal_ret = board_hal_i2c_write(HAL_I2C_PORT_0, g_dev_addr, write_buf, 2U, TMP102_I2C_TIMEOUT_MS);

    if(hal_ret != HAL_OK)
    {
        ESP_LOGE(TAG, "Failed to set alert polarity");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t tmp102_set_fault(tmp102_fault_t fault)
{
    esp_err_t ret;
    uint8_t config_byte = 0U;

    /* State validation */
    if(!g_initialized)
    {
        ESP_LOGE(TAG, "Sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Clamp fault setting to valid range (0-3) */
    uint8_t fault_val = (uint8_t)fault & 0x03U;

    /* Set pointer to config register */
    ret = open_pointer_register(TMP102_REG_CONFIG);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* Read first byte of config register */
    ret = read_register_byte(0U, &config_byte);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* Clear F0/F1 bits (bits 3-4) and set new fault setting */
    config_byte &= 0xE7U;
    config_byte |= (fault_val << 3U);

    /* Write back config register */
    uint8_t write_buf[2] = {TMP102_REG_CONFIG, config_byte};
    hal_err_t hal_ret = board_hal_i2c_write(HAL_I2C_PORT_0, g_dev_addr, write_buf, 2U, TMP102_I2C_TIMEOUT_MS);

    if(hal_ret != HAL_OK)
    {
        ESP_LOGE(TAG, "Failed to set fault queue");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t tmp102_set_alert_mode(bool mode)
{
    esp_err_t ret;
    uint8_t config_byte = 0U;

    /* State validation */
    if(!g_initialized)
    {
        ESP_LOGE(TAG, "Sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Set pointer to config register */
    ret = open_pointer_register(TMP102_REG_CONFIG);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* Read first byte of config register */
    ret = read_register_byte(0U, &config_byte);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* Clear TM bit (bit 1) and set new mode */
    config_byte &= 0xFDU;
    if(mode)
    {
        config_byte |= (1U << 1U);
    }

    /* Write back config register */
    uint8_t write_buf[2] = {TMP102_REG_CONFIG, config_byte};
    hal_err_t hal_ret = board_hal_i2c_write(HAL_I2C_PORT_0, g_dev_addr, write_buf, 2U, TMP102_I2C_TIMEOUT_MS);

    if(hal_ret != HAL_OK)
    {
        ESP_LOGE(TAG, "Failed to set alert mode");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t tmp102_set_low_temp_c(float temp)
{
    /* State validation */
    if(!g_initialized)
    {
        ESP_LOGE(TAG, "Sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    return write_temperature_threshold(TMP102_REG_T_LOW, temp);
}

esp_err_t tmp102_set_high_temp_c(float temp)
{
    /* State validation */
    if(!g_initialized)
    {
        ESP_LOGE(TAG, "Sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    return write_temperature_threshold(TMP102_REG_T_HIGH, temp);
}

esp_err_t tmp102_set_low_temp_f(float temp)
{
    float temp_c;

    /* State validation */
    if(!g_initialized)
    {
        ESP_LOGE(TAG, "Sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Convert to Celsius */
    temp_c = (temp - 32.0f) * 5.0f / 9.0f;
    return write_temperature_threshold(TMP102_REG_T_LOW, temp_c);
}

esp_err_t tmp102_set_high_temp_f(float temp)
{
    float temp_c;

    /* State validation */
    if(!g_initialized)
    {
        ESP_LOGE(TAG, "Sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Convert to Celsius */
    temp_c = (temp - 32.0f) * 5.0f / 9.0f;
    return write_temperature_threshold(TMP102_REG_T_HIGH, temp_c);
}

esp_err_t tmp102_read_low_temp_c(float* temp)
{
    esp_err_t ret;
    uint8_t config_bytes[2] = {0U, 0U};
    uint8_t temp_bytes[2] = {0U, 0U};
    bool extended_mode = false;
    int16_t digital_temp;

    /* Parameter validation */
    if(temp == NULL)
    {
        ESP_LOGE(TAG, "temp is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    /* State validation */
    if(!g_initialized)
    {
        ESP_LOGE(TAG, "Sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Read config register to determine extended mode */
    ret = open_pointer_register(TMP102_REG_CONFIG);
    if(ret != ESP_OK)
    {
        return ret;
    }

    ret = read_register_byte(0U, &config_bytes[0]);
    if(ret != ESP_OK)
    {
        return ret;
    }

    ret = read_register_byte(1U, &config_bytes[1]);
    if(ret != ESP_OK)
    {
        return ret;
    }

    extended_mode = ((config_bytes[1] & 0x10U) != 0U);

    /* Read T_LOW register */
    ret = open_pointer_register(TMP102_REG_T_LOW);
    if(ret != ESP_OK)
    {
        return ret;
    }

    ret = read_register_byte(0U, &temp_bytes[0]);
    if(ret != ESP_OK)
    {
        return ret;
    }

    ret = read_register_byte(1U, &temp_bytes[1]);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* Check for invalid reading */
    if((temp_bytes[0] == 0xFFU) && (temp_bytes[1] == 0xFFU))
    {
        ESP_LOGE(TAG, "Invalid T_LOW reading");
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* Convert to digital temperature based on mode */
    if(extended_mode)
    {
        /* 13-bit mode */
        digital_temp = (int16_t)(((uint16_t)temp_bytes[0] << 5U) | ((uint16_t)temp_bytes[1] >> 3U));
        if(digital_temp > 0x0FFF)
        {
            digital_temp |= (int16_t)0xE000;
        }
    }
    else
    {
        /* 12-bit mode */
        digital_temp = (int16_t)(((uint16_t)temp_bytes[0] << 4U) | ((uint16_t)temp_bytes[1] >> 4U));
        if(digital_temp > 0x07FF)
        {
            digital_temp |= (int16_t)0xF000;
        }
    }

    /* Convert to Celsius */
    *temp = (float)digital_temp * 0.0625f;

    /* Validate range */
    if((*temp < TMP102_TEMP_MIN_C) || (*temp > TMP102_TEMP_MAX_C))
    {
        ESP_LOGE(TAG, "T_LOW out of range: %f", *temp);
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

esp_err_t tmp102_read_high_temp_c(float* temp)
{
    esp_err_t ret;
    uint8_t config_bytes[2] = {0U, 0U};
    uint8_t temp_bytes[2] = {0U, 0U};
    bool extended_mode = false;
    int16_t digital_temp;

    /* Parameter validation */
    if(temp == NULL)
    {
        ESP_LOGE(TAG, "temp is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    /* State validation */
    if(!g_initialized)
    {
        ESP_LOGE(TAG, "Sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Read config register to determine extended mode */
    ret = open_pointer_register(TMP102_REG_CONFIG);
    if(ret != ESP_OK)
    {
        return ret;
    }

    ret = read_register_byte(0U, &config_bytes[0]);
    if(ret != ESP_OK)
    {
        return ret;
    }

    ret = read_register_byte(1U, &config_bytes[1]);
    if(ret != ESP_OK)
    {
        return ret;
    }

    extended_mode = ((config_bytes[1] & 0x10U) != 0U);

    /* Read T_HIGH register */
    ret = open_pointer_register(TMP102_REG_T_HIGH);
    if(ret != ESP_OK)
    {
        return ret;
    }

    ret = read_register_byte(0U, &temp_bytes[0]);
    if(ret != ESP_OK)
    {
        return ret;
    }

    ret = read_register_byte(1U, &temp_bytes[1]);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* Check for invalid reading */
    if((temp_bytes[0] == 0xFFU) && (temp_bytes[1] == 0xFFU))
    {
        ESP_LOGE(TAG, "Invalid T_HIGH reading");
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* Convert to digital temperature based on mode */
    if(extended_mode)
    {
        /* 13-bit mode */
        digital_temp = (int16_t)(((uint16_t)temp_bytes[0] << 5U) | ((uint16_t)temp_bytes[1] >> 3U));
        if(digital_temp > 0x0FFF)
        {
            digital_temp |= (int16_t)0xE000;
        }
    }
    else
    {
        /* 12-bit mode */
        digital_temp = (int16_t)(((uint16_t)temp_bytes[0] << 4U) | ((uint16_t)temp_bytes[1] >> 4U));
        if(digital_temp > 0x07FF)
        {
            digital_temp |= (int16_t)0xF000;
        }
    }

    /* Convert to Celsius */
    *temp = (float)digital_temp * 0.0625f;

    /* Validate range */
    if((*temp < TMP102_TEMP_MIN_C) || (*temp > TMP102_TEMP_MAX_C))
    {
        ESP_LOGE(TAG, "T_HIGH out of range: %f", *temp);
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

esp_err_t tmp102_read_low_temp_f(float* temp)
{
    esp_err_t ret;
    float temp_c = 0.0f;

    /* Parameter validation */
    if(temp == NULL)
    {
        ESP_LOGE(TAG, "temp is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    /* Read in Celsius */
    ret = tmp102_read_low_temp_c(&temp_c);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* Convert to Fahrenheit */
    *temp = (temp_c * 9.0f / 5.0f) + 32.0f;
    return ESP_OK;
}

esp_err_t tmp102_read_high_temp_f(float* temp)
{
    esp_err_t ret;
    float temp_c = 0.0f;

    /* Parameter validation */
    if(temp == NULL)
    {
        ESP_LOGE(TAG, "temp is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    /* Read in Celsius */
    ret = tmp102_read_high_temp_c(&temp_c);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* Convert to Fahrenheit */
    *temp = (temp_c * 9.0f / 5.0f) + 32.0f;
    return ESP_OK;
}

/* Helper functions */

static esp_err_t open_pointer_register(uint8_t reg)
{
    hal_err_t ret = board_hal_i2c_write(HAL_I2C_PORT_0, g_dev_addr, &reg, 1U, TMP102_I2C_TIMEOUT_MS);
    if(ret != HAL_OK)
    {
        ESP_LOGE(TAG, "Failed to set pointer register to 0x%02X", reg);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t read_register_byte(uint8_t byte_index, uint8_t* value)
{
    uint8_t register_bytes[2] = {0U, 0U};

    /* Read two bytes from TMP102 */
    hal_err_t ret = board_hal_i2c_read(HAL_I2C_PORT_0, g_dev_addr, register_bytes, 2U, TMP102_I2C_TIMEOUT_MS);
    if(ret != HAL_OK)
    {
        ESP_LOGE(TAG, "Failed to read register");
        return ESP_FAIL;
    }

    /* Validate byte index */
    if(byte_index > 1U)
    {
        ESP_LOGE(TAG, "Invalid byte index: %u", byte_index);
        return ESP_ERR_INVALID_ARG;
    }

    *value = register_bytes[byte_index];
    return ESP_OK;
}

static esp_err_t read_temperature_raw(float* temperature)
{
    esp_err_t ret;
    uint8_t temp_bytes[2] = {0U, 0U};
    int16_t digital_temp;

    /* Set pointer to temperature register */
    ret = open_pointer_register(TMP102_REG_TEMPERATURE);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* Read temperature bytes */
    ret = read_register_byte(0U, &temp_bytes[0]);
    if(ret != ESP_OK)
    {
        return ret;
    }

    ret = read_register_byte(1U, &temp_bytes[1]);
    if(ret != ESP_OK)
    {
        return ret;
    }

    /* Check for invalid reading */
    if((temp_bytes[0] == 0xFFU) && (temp_bytes[1] == 0xFFU))
    {
        ESP_LOGE(TAG, "Invalid temperature reading");
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* Bit 0 of second byte indicates 12-bit (0) or 13-bit (1) mode */
    if((temp_bytes[1] & 0x01U) != 0U)
    {
        /* 13-bit mode */
        digital_temp = (int16_t)(((uint16_t)temp_bytes[0] << 5U) | ((uint16_t)temp_bytes[1] >> 3U));
        if(digital_temp > 0x0FFF)
        {
            digital_temp |= (int16_t)0xE000;
        }
    }
    else
    {
        /* 12-bit mode */
        digital_temp = (int16_t)(((uint16_t)temp_bytes[0] << 4U) | ((uint16_t)temp_bytes[1] >> 4U));
        if(digital_temp > 0x07FF)
        {
            digital_temp |= (int16_t)0xF000;
        }
    }

    /* Convert to Celsius (0.0625°C per LSB) */
    *temperature = (float)digital_temp * 0.0625f;

    /* Validate range */
    if((*temperature < TMP102_TEMP_MIN_C) || (*temperature > TMP102_TEMP_MAX_C))
    {
        ESP_LOGE(TAG, "Temperature out of range: %f", *temperature);
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

static esp_err_t write_temperature_threshold(uint8_t reg, float temperature)
{
    esp_err_t ret;
    uint8_t config_bytes[2] = {0U, 0U};
    uint8_t temp_bytes[2] = {0U, 0U};
    bool extended_mode = false;
    int16_t digital_temp;

    /* Clamp temperature to sensor range */
    if(temperature > TMP102_TEMP_MAX_C)
    {
        temperature = TMP102_TEMP_MAX_C;
    }
    if(temperature < TMP102_TEMP_MIN_C)
    {
        temperature = TMP102_TEMP_MIN_C;
    }

    /* Read config register to determine extended mode */
    ret = open_pointer_register(TMP102_REG_CONFIG);
    if(ret != ESP_OK)
    {
        return ret;
    }

    ret = read_register_byte(0U, &config_bytes[0]);
    if(ret != ESP_OK)
    {
        return ret;
    }

    ret = read_register_byte(1U, &config_bytes[1]);
    if(ret != ESP_OK)
    {
        return ret;
    }

    extended_mode = ((config_bytes[1] & 0x10U) != 0U);

    /* Convert temperature to digital value */
    digital_temp = (int16_t)(temperature / 0.0625f);

    /* Split into bytes based on mode */
    if(extended_mode)
    {
        /* 13-bit mode */
        temp_bytes[0] = (uint8_t)((uint16_t)digital_temp >> 5U);
        temp_bytes[1] = (uint8_t)((uint16_t)digital_temp << 3U);
    }
    else
    {
        /* 12-bit mode */
        temp_bytes[0] = (uint8_t)((uint16_t)digital_temp >> 4U);
        temp_bytes[1] = (uint8_t)((uint16_t)digital_temp << 4U);
    }

    /* Write to threshold register */
    uint8_t write_buf[3] = {reg, temp_bytes[0], temp_bytes[1]};
    hal_err_t hal_ret = board_hal_i2c_write(HAL_I2C_PORT_0, g_dev_addr, write_buf, 3U, TMP102_I2C_TIMEOUT_MS);

    if(hal_ret != HAL_OK)
    {
        ESP_LOGE(TAG, "Failed to write temperature threshold");
        return ESP_FAIL;
    }

    return ESP_OK;
}
