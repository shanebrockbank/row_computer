#include "esp_log.h"
#include "driver/uart.h"
#include "config/pin_definitions.h"
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
                             sizeof(test_buffer)-1, pdMS_TO_TICKS(2000)); // 2 second timeout
    
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
    vTaskDelay(pdMS_TO_TICKS(100));
    
    uart_write_bytes(GPS_UART_NUM, (const char*)UBX_ENABLE_GGA, sizeof(UBX_ENABLE_GGA));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    uart_write_bytes(GPS_UART_NUM, (const char*)UBX_ENABLE_RMC, sizeof(UBX_ENABLE_RMC));
    vTaskDelay(pdMS_TO_TICKS(100));
    
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

esp_err_t gps_read(gps_data_t *out_data) {
    static char rx_buffer[GPS_UART_BUF_SIZE];
    static char nmea_line[MAX_NMEA_LEN];
    static int line_pos = 0;
    static uint32_t last_data_time = 0;
    
    int len = uart_read_bytes(GPS_UART_NUM, (uint8_t*)rx_buffer,
                              sizeof(rx_buffer)-1, pdMS_TO_TICKS(GPS_TIMEOUT_MS)); 
    
    if (len <= 0) {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (current_time - last_data_time > 5000) { // No data for 5 seconds
            ESP_LOGW(TAG, "No GPS data received for 5 seconds - check connections");
            last_data_time = current_time;
        }
        return ESP_ERR_NOT_FOUND;
    }

    // Update last data time
    last_data_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Log raw data occasionally for debugging
    static int debug_counter = 0;
    if (++debug_counter >= 50) { // Every ~50 calls
        ESP_LOGD(TAG, "Raw GPS data (%d bytes): %.50s", len, rx_buffer);
        debug_counter = 0;
    }

    for (int i = 0; i < len; i++) {
        char c = rx_buffer[i];
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

    // Copy internal gps_data to caller
    *out_data = gps_data;
    return ESP_OK;
}

// Print formatted GPS data
void print_gps_data(void) {
    printf("\n=== GPS Data ===\n");
    printf("Time:      %s UTC\n", strlen(gps_data.time) > 0 ? gps_data.time : "No fix");
    printf("Status:    %s\n", gps_data.valid_fix ? "Valid Fix" : "No Fix");
    printf("Latitude:  %.6f°\n", gps_data.latitude);
    printf("Longitude: %.6f°\n", gps_data.longitude);
    printf("Speed:     %.1f knots (%.1f km/h)\n", gps_data.speed_knots, gps_data.speed_knots * 1.852);
    printf("Heading:   %.1f°\n", gps_data.heading);
    printf("Satellites: %d\n", gps_data.satellites);
    printf("================\n\n");
}

// Simple test function to debug GPS
void gps_debug_raw_data(void) {
    ESP_LOGI(TAG, "=== GPS Raw Data Debug ===");
    char buffer[512];
    
    for (int i = 0; i < 10; i++) { // Read 10 times
        int len = uart_read_bytes(GPS_UART_NUM, (uint8_t*)buffer, 
                                 sizeof(buffer)-1, pdMS_TO_TICKS(1000));
        if (len > 0) {
            buffer[len] = '\0';
            ESP_LOGI(TAG, "Read %d: %s", i, buffer);
        } else {
            ESP_LOGI(TAG, "Read %d: No data", i);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void manual_gps_test(void) {
    ESP_LOGI("MANUAL_TEST", "=== Manual GPS Test ===");
    
    // Test 1: Check if UART is receiving any data
    ESP_LOGI("MANUAL_TEST", "Test 1: Raw UART data check");
    char buffer[256];
    for (int i = 0; i < 5; i++) {
        int len = uart_read_bytes(GPS_UART_NUM, (uint8_t*)buffer, 
                                 sizeof(buffer)-1, pdMS_TO_TICKS(2000));
        if (len > 0) {
            buffer[len] = '\0';
            ESP_LOGI("MANUAL_TEST", "Received %d bytes: %s", len, buffer);
        } else {
            ESP_LOGE("MANUAL_TEST", "No data received (attempt %d)", i+1);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Test 2: GPS parsing test
    ESP_LOGI("MANUAL_TEST", "Test 2: GPS data parsing");
    gps_data_t gps_data;
    for (int i = 0; i < 10; i++) {
        esp_err_t err = gps_read(&gps_data);
        if (err == ESP_OK) {
            ESP_LOGI("MANUAL_TEST", "GPS read OK - Fix: %s, Sats: %d", 
                    gps_data.valid_fix ? "YES" : "NO", gps_data.satellites);
            if (gps_data.valid_fix) {
                ESP_LOGI("MANUAL_TEST", "Position: %.6f, %.6f", 
                        gps_data.latitude, gps_data.longitude);
                break; // Success!
            }
        } else {
            ESP_LOGW("MANUAL_TEST", "GPS read failed: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}