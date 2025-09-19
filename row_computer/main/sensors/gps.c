#include "esp_log.h"
#include "driver/uart.h"
#include "config/pin_definitions.h"
#include "config/common_constants.h"
#include "gps.h"
#include "esp_mac.h"
#include <string.h>

static const char *TAG = "GPS";

// UBX commands to configure GPS module
static const uint8_t UBX_ENABLE_GGA[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
    0xF0, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x17, 0x0A
};

static const uint8_t UBX_ENABLE_RMC[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
    0xF0, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x21, 0x0D
};

// Set update rate to 1Hz (recommended for rowing applications)
static const uint8_t UBX_SET_1HZ[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
    0xE8, 0x03, 0x01, 0x00, 0x01, 0x00,
    0x01, 0x39
};

static gps_data_t gps_data = {0};

// Initialize UART specifically for GPS
esp_err_t gps_uart_init(void) {
    ESP_LOGI(TAG, "Initializing GPS UART...");
    
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
    
    ESP_LOGI(TAG, "GPS UART initialized successfully");
    return ESP_OK;
}

// Test if GPS module is responding
esp_err_t gps_test_communication(void) {
    ESP_LOGI(TAG, "Testing GPS communication...");
    
    // Clear any existing data in buffer
    uart_flush(GPS_UART_NUM);
    
    // Wait for some data to arrive (GPS should be transmitting)
    char test_buffer[256];
    int len = uart_read_bytes(GPS_UART_NUM, (uint8_t*)test_buffer,
                             sizeof(test_buffer)-1, pdMS_TO_TICKS(GPS_COMMUNICATION_TEST_TIMEOUT_MS));
    
    if (len > 0) {
        test_buffer[len] = '\0';
        ESP_LOGI(TAG, "GPS communication OK - received %d bytes", len);
        ESP_LOGI(TAG, "Sample data: %.50s", test_buffer); // Show first 50 chars
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "No data received from GPS module");
        ESP_LOGE(TAG, "Check wiring: TX=%d, RX=%d", GPS_UART_TXD_PIN, GPS_UART_RXD_PIN);
        return ESP_ERR_NOT_FOUND;
    }
}

// Configure GPS module with UBX commands
esp_err_t gps_configure_module(void) {
    ESP_LOGI(TAG, "Configuring GPS module...");
    
    // Send UBX commands to enable specific NMEA sentences
    uart_write_bytes(GPS_UART_NUM, (const char*)UBX_SET_1HZ, sizeof(UBX_SET_1HZ));
    vTaskDelay(pdMS_TO_TICKS(SENSOR_STABILIZE_DELAY_MS));
    
    uart_write_bytes(GPS_UART_NUM, (const char*)UBX_ENABLE_GGA, sizeof(UBX_ENABLE_GGA));
    vTaskDelay(pdMS_TO_TICKS(SENSOR_STABILIZE_DELAY_MS));
    
    uart_write_bytes(GPS_UART_NUM, (const char*)UBX_ENABLE_RMC, sizeof(UBX_ENABLE_RMC));
    vTaskDelay(pdMS_TO_TICKS(SENSOR_STABILIZE_DELAY_MS));
    
    ESP_LOGI(TAG, "GPS configuration commands sent");
    return ESP_OK;
}

esp_err_t gps_init(void) {
    ESP_LOGI(TAG, "Initializing GPS module...");
    
    // Initialize UART for GPS
    esp_err_t err = gps_uart_init();
    if (err != ESP_OK) {
        return err;
    }
    
    // Test communication
    err = gps_test_communication();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "GPS communication test failed - module may not be connected");
        // Don't return error - allow system to continue
    }
    
    // Configure GPS module
    gps_configure_module();
    
    ESP_LOGI(TAG, "GPS initialization complete");
    return ESP_OK;
}

// Convert NMEA coordinate to decimal degrees
double nmea_to_decimal(const char *coord, const char *direction) {
    if (!coord || strlen(coord) == 0) return 0.0;
    
    double val = atof(coord);
    if (val < 0 || val > 18000) return 0.0; // Basic sanity check
    
    int degrees = (int)(val / 100);
    double minutes = val - (degrees * 100);
    
    if (minutes >= 60.0) return 0.0;
    
    double decimal = degrees + (minutes / 60.0);
    
    // Apply hemisphere direction
    if (direction && (*direction == 'S' || *direction == 'W'))
        decimal = -decimal;
    
    return decimal;
}

// Parse RMC sentence
static void parse_rmc(char *sentence) {
    char *tokens[12];
    char *token = strtok(sentence, ",");
    int i = 0;

    while (token && i < 12) {
        tokens[i++] = token;
        token = strtok(NULL, ",");
    }

    if (i >= 2 && strlen(tokens[1]) >= 6) {
        snprintf(gps_data.time, sizeof(gps_data.time), "%.2s:%.2s:%.2s",
                 tokens[1], tokens[1]+2, tokens[1]+4);
        gps_data.valid_fix = (tokens[2][0] == 'A');
    }

    if (i >= 5 && strlen(tokens[3]) > 0 && strlen(tokens[4]) > 0)
        gps_data.latitude = nmea_to_decimal(tokens[3], tokens[4]);

    if (i >= 7 && strlen(tokens[5]) > 0 && strlen(tokens[6]) > 0)
        gps_data.longitude = nmea_to_decimal(tokens[5], tokens[6]);

    if (i >= 8 && strlen(tokens[7]) > 0)
        gps_data.speed_knots = atof(tokens[7]);

    if (i >= 9 && strlen(tokens[8]) > 0)
        gps_data.heading = atof(tokens[8]);
}

// Parse GGA sentence
static void parse_gga(char *sentence) {
    char *token = strtok(sentence, ",");
    int field = 0;

    while (token) {
        if (field == 7 && strlen(token) > 0) { // satellites
            gps_data.satellites = atoi(token);
            break;
        }
        token = strtok(NULL, ",");
        field++;
    }
}

// Process any NMEA sentence
static void process_nmea_sentence(char *sentence) {
    char buffer[GPS_UART_BUF_SIZE];
    strncpy(buffer, sentence, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    // Log interesting sentences for debugging
    if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0) {
        ESP_LOGD(TAG, "RMC: %s", sentence);
        parse_rmc(buffer);
    }
    else if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0) {
        ESP_LOGD(TAG, "GGA: %s", sentence);
        parse_gga(buffer);
    }
}

// Read raw UART data with timeout and connection monitoring
static esp_err_t gps_read_uart_data(char *buffer, size_t buffer_size, int *bytes_read) {
    static uint32_t last_data_time = 0;

    *bytes_read = uart_read_bytes(GPS_UART_NUM, (uint8_t*)buffer,
                                  buffer_size-1, pdMS_TO_TICKS(GPS_TIMEOUT_MS));

    if (*bytes_read <= 0) {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (current_time - last_data_time > GPS_DATA_TIMEOUT_MS) { // No GPS data warning
            ESP_LOGW(TAG, "No GPS data received - check connections");
            last_data_time = current_time;
        }
        return ESP_ERR_NOT_FOUND;
    }

    // Update last data time and add debug logging
    last_data_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    static int debug_counter = 0;
    if (++debug_counter >= GPS_DEBUG_LOG_INTERVAL) { // Debug logging interval
        ESP_LOGD(TAG, "Raw GPS data (%d bytes): %.50s", *bytes_read, buffer);
        debug_counter = 0;
    }

    return ESP_OK;
}

// Parse UART buffer into NMEA sentences
static void gps_parse_nmea_buffer(const char *buffer, int buffer_len) {
    static char nmea_line[MAX_NMEA_LEN];
    static int line_pos = 0;

    for (int i = 0; i < buffer_len; i++) {
        char c = buffer[i];
        if (c == '\n' || c == '\r') {
            if (line_pos > 0) {
                nmea_line[line_pos] = '\0';
                process_nmea_sentence(nmea_line);
                line_pos = 0;
            }
        } else if (line_pos < MAX_NMEA_LEN-1) {
            nmea_line[line_pos++] = c;
        } else {
            // Buffer overflow, reset
            ESP_LOGW(TAG, "NMEA line buffer overflow");
            line_pos = 0;
        }
    }
}

// Main GPS read function - now simplified and focused
esp_err_t gps_read(gps_data_t *out_data) {
    if (out_data == NULL) {
        ESP_LOGE(TAG, "GPS output data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    char rx_buffer[GPS_UART_BUF_SIZE];
    int bytes_read;

    // Read UART data
    esp_err_t err = gps_read_uart_data(rx_buffer, sizeof(rx_buffer), &bytes_read);
    if (err != ESP_OK) {
        return err;
    }

    // Parse NMEA sentences from buffer
    gps_parse_nmea_buffer(rx_buffer, bytes_read);

    // Copy parsed data to caller
    *out_data = gps_data;
    return ESP_OK;
}

// Simplified GPS debug function (replaces multiple verbose test functions)
void gps_debug_raw_data(void) {
    ESP_LOGI(TAG, "GPS Debug: Reading 3 samples...");
    char buffer[256];

    for (int i = 0; i < 3; i++) {
        int len = uart_read_bytes(GPS_UART_NUM, (uint8_t*)buffer,
                                 sizeof(buffer)-1, pdMS_TO_TICKS(1000));
        if (len > 0) {
            buffer[len] = '\0';
            ESP_LOGI(TAG, "Sample %d (%d bytes): %.100s", i+1, len, buffer);
        } else {
            ESP_LOGI(TAG, "Sample %d: No data", i+1);
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // Short delay between debug samples
    }
}