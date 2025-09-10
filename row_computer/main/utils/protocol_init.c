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

esp_err_t uart_gps_init(void) {
    ESP_LOGI(TAG, "Initializing UART for GPS...");
    
    const uart_config_t uart_config = {
        .baud_rate = GPS_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    esp_err_t err = uart_driver_install(GPS_UART_NUM, GPS_UART_BUF_SIZE * 2, 
                                       0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        return err;
    }
    
    err = uart_param_config(GPS_UART_NUM, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(err));
        return err;
    }
    
    err = uart_set_pin(GPS_UART_NUM, GPS_UART_TXD_PIN, GPS_UART_RXD_PIN, 
                      GPS_UART_RTS_PIN, GPS_UART_CTS_PIN);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "UART for GPS initialized successfully");
    return ESP_OK;
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
    ESP_LOGI(TAG, "Initializing all communication protocols...");
    
    esp_err_t err;
    
    // Initialize I2C (for IMU, magnetometer, etc.)
    err = i2c_master_init();
    if (err != ESP_OK) {
        return err;
    }
    
    // Initialize UART (for GPS)
    err = uart_gps_init();
    if (err != ESP_OK) {
        return err;
    }
    
    // Initialize SPI (if needed for additional sensors)
    // Uncomment if you have SPI sensors
    // err = spi_master_init();
    // if (err != ESP_OK) {
    //     return err;
    // }
    
    ESP_LOGI(TAG, "All protocols initialized successfully");
    return ESP_OK;
}

esp_err_t protocols_deinit(void) {
    ESP_LOGI(TAG, "Deinitializing protocols...");
    
    i2c_driver_delete(I2C_MASTER_NUM);
    uart_driver_delete(GPS_UART_NUM);
    // spi_bus_free(SPI2_HOST);  // If SPI was initialized
    
    ESP_LOGI(TAG, "All protocols deinitialized");
    return ESP_OK;
}