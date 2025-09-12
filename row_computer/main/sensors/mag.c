#include "esp_log.h"
#include "driver/i2c.h"
#include "config/pin_definitions.h"
#include "sensors/sensors_common.h"

static const char *TAG = "MAG";

void mag_init(void)
{
    esp_err_t err;
    
    // Configuration Register A
    err = mag_write_byte(HMC5883L_REG_CONFIG_A, HMC5883L_SAMPLES_8);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure HMC5883L register A: %s", esp_err_to_name(err));
        return;
    }
    
    // Configuration Register B
    err = mag_write_byte(HMC5883L_REG_CONFIG_B, HMC5883L_DATARATE_15HZ);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure HMC5883L register B: %s", esp_err_to_name(err));
        return;
    }
    
    // Mode Register
    err = mag_write_byte(HMC5883L_REG_MODE, HMC5883L_MEAS_NORMAL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set HMC5883L mode: %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGI(TAG, "HMC5883L initialized successfully");
}

esp_err_t mag_read(float *x, float *y, float *z)
{
    uint8_t data[6];
    esp_err_t err = mag_read_bytes(HMC5883L_REG_DATA_X_MSB, data, 6);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read HMC5883L data: %s", esp_err_to_name(err));
        return err;
    }

    int16_t raw_x = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_z = (int16_t)((data[2] << 8) | data[3]);
    int16_t raw_y = (int16_t)((data[4] << 8) | data[5]);

    ESP_LOGI(TAG, "Raw int16: X=%d Y=%d Z=%d", raw_x, raw_y, raw_z);

    *x = raw_x * MAG_SCALE_1_3_GAUSS;
    *y = raw_y * MAG_SCALE_1_3_GAUSS;
    *z = raw_z * MAG_SCALE_1_3_GAUSS;

    return ESP_OK;
}


