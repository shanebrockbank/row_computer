#include "protocol_init.h"
#include "pin_definitions.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "esp_log.h"

static const char *TAG = "PROTOCOLS";

esp_err_t i2c_master_init(void) {
    ESP_LOGI(TAG, "Initializing I2C master...");
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return err;
    }
    
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, 
                            I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "I2C master initialized successfully");
    return ESP_OK;
}

// Start with bare minimum hardware validation
void test_i2c_bus(void) {
    ESP_LOGI(TAG, "Scanning I2C bus...");
    int devices_found = 0;
    
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found I2C device at address 0x%02X", addr);
            devices_found++;
        }
    }
    
    if (devices_found == 0) {
        ESP_LOGW(TAG, "No I2C devices found - check connections");
    } else {
        ESP_LOGI(TAG, "I2C scan complete - found %d devices", devices_found);
    }
}

esp_err_t spi_master_init(void) {
    ESP_LOGI(TAG, "Initializing SPI master...");
    
    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_MISO_PIN,
        .mosi_io_num = SPI_MOSI_PIN,
        .sclk_io_num = SPI_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4092,
    };
    
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus initialize failed: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "SPI master initialized successfully");
    return ESP_OK;
}

esp_err_t protocols_init(void) {
    ESP_LOGI(TAG, "Initializing communication protocols...");
    
    esp_err_t err;
    
    // Initialize I2C (for IMU, magnetometer, etc.)
    err = i2c_master_init();
    if (err != ESP_OK) {
        return err;
    }
    
    // Test I2C bus to verify sensors are connected
    test_i2c_bus();
    
    // Note: GPS UART is now initialized in gps_init() to avoid conflicts
    
    // Initialize SPI (if needed for additional sensors)
    // Uncomment if you have SPI sensors
    // err = spi_master_init();
    // if (err != ESP_OK) {
    //     return err;
    // }
    
    ESP_LOGI(TAG, "Core protocols initialized successfully");
    return ESP_OK;
}

esp_err_t protocols_deinit(void) {
    ESP_LOGI(TAG, "Deinitializing protocols...");
    
    i2c_driver_delete(I2C_MASTER_NUM);
    // Note: GPS UART cleanup is handled in GPS module
    // spi_bus_free(SPI2_HOST);  // If SPI was initialized
    
    ESP_LOGI(TAG, "All protocols deinitialized");
    return ESP_OK;
}