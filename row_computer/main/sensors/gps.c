#include "esp_log.h"
#include "driver/uart.h"
#include "config/pin_definitions.h"
#include "config/common_constants.h"
#include "gps.h"
#include "esp_mac.h"
#include <string.h>

static const char *TAG = "GPS";

// UBX utility functions
static uint16_t ubx_calculate_checksum(const uint8_t* data, uint16_t length) {
    uint8_t ck_a = 0, ck_b = 0;
    for (uint16_t i = 0; i < length; i++) {
        ck_a += data[i];
        ck_b += ck_a;
    }
    return (ck_b << 8) | ck_a; // Return as 16-bit value, caller extracts bytes
}

static bool ubx_parse_header(const uint8_t* buffer, uint8_t* msg_class, uint8_t* msg_id, uint16_t* payload_length) {
    // Check sync bytes
    if (buffer[0] != 0xB5 || buffer[1] != 0x62) {
        return false;
    }

    *msg_class = buffer[2];
    *msg_id = buffer[3];
    *payload_length = (buffer[5] << 8) | buffer[4]; // Little endian

    return true;
}

static uint16_t ubx_create_packet(uint8_t msg_class, uint8_t msg_id, const uint8_t* payload, uint16_t payload_len, uint8_t* output) {
    uint16_t index = 0;

    // Sync bytes
    output[index++] = 0xB5;
    output[index++] = 0x62;

    // Header
    output[index++] = msg_class;
    output[index++] = msg_id;
    output[index++] = payload_len & 0xFF;        // Length low byte
    output[index++] = (payload_len >> 8) & 0xFF; // Length high byte

    // Payload
    for (uint16_t i = 0; i < payload_len; i++) {
        output[index++] = payload[i];
    }

    // Calculate checksum over class, id, length, and payload
    uint16_t checksum = ubx_calculate_checksum(&output[2], 4 + payload_len);
    output[index++] = checksum & 0xFF;      // CK_A
    output[index++] = (checksum >> 8) & 0xFF; // CK_B

    return index; // Total packet length
}

// UBX configuration commands for pure UBX mode
static bool ubx_send_config_command(const uint8_t* command, uint16_t length) {
    uart_write_bytes(GPS_UART_NUM, (const char*)command, length);
    vTaskDelay(pdMS_TO_TICKS(200)); // Wait for processing

    // Simple ACK check - look for UBX-ACK-ACK (0x05, 0x01)
    uint8_t response[32];
    int len = uart_read_bytes(GPS_UART_NUM, response, sizeof(response), pdMS_TO_TICKS(500));

    for (int i = 0; i < len - 5; i++) {
        if (response[i] == 0xB5 && response[i+1] == 0x62 &&
            response[i+2] == 0x05 && response[i+3] == 0x01) {
            return true; // Found ACK
        }
    }
    return false; // No ACK found
}

static gps_data_t gps_data = {0};
static gps_health_t gps_health = {0};

// Parse UBX-NAV-PVT message (92 bytes payload)
static void parse_ubx_nav_pvt(const uint8_t* payload) {
    // Extract time fields (bytes 8-10 for HH:MM:SS)
    uint8_t hour = payload[8];
    uint8_t min = payload[9];
    uint8_t sec = payload[10];

    // Format time as HH:MM:SS
    snprintf(gps_data.time, sizeof(gps_data.time), "%02d:%02d:%02d", hour, min, sec);

    // Extract fix type (byte 20)
    uint8_t fix_type = payload[20];
    gps_data.valid_fix = (fix_type >= 2); // 2=2D fix, 3=3D fix

    // Extract number of satellites (byte 23)
    gps_data.satellites = payload[23];

    // Extract longitude (bytes 24-27, int32_t, 1e-7 degrees)
    int32_t lon_raw = (int32_t)((payload[27] << 24) | (payload[26] << 16) | (payload[25] << 8) | payload[24]);
    gps_data.longitude = (double)lon_raw * 1e-7;

    // Extract latitude (bytes 28-31, int32_t, 1e-7 degrees)
    int32_t lat_raw = (int32_t)((payload[31] << 24) | (payload[30] << 16) | (payload[29] << 8) | payload[28]);
    gps_data.latitude = (double)lat_raw * 1e-7;

    // Extract ground speed (bytes 60-63, uint32_t, mm/s)
    uint32_t speed_mm_s = (payload[63] << 24) | (payload[62] << 16) | (payload[61] << 8) | payload[60];
    gps_data.speed_knots = (float)speed_mm_s * 0.001944f; // Convert mm/s to knots

    // Extract heading of motion (bytes 64-67, int32_t, 1e-5 degrees)
    int32_t heading_raw = (int32_t)((payload[67] << 24) | (payload[66] << 16) | (payload[65] << 8) | payload[64]);
    gps_data.heading = (float)heading_raw * 1e-5f;

    // Ensure heading is in 0-360 range
    if (gps_data.heading < 0) gps_data.heading += 360.0f;
    if (gps_data.heading >= 360.0f) gps_data.heading -= 360.0f;

    // Extract health/accuracy data
    // Horizontal accuracy (bytes 40-43, uint32_t, mm)
    gps_health.horizontal_accuracy = (payload[43] << 24) | (payload[42] << 16) | (payload[41] << 8) | payload[40];

    // Speed accuracy (bytes 68-71, uint32_t, mm/s)
    gps_health.speed_accuracy = (payload[71] << 24) | (payload[70] << 16) | (payload[69] << 8) | payload[68];

    // Fix type and satellite count (same as main GPS data)
    gps_health.fix_type = fix_type;
    gps_health.satellites = payload[23];
}

// Initialize UART specifically for GPS
esp_err_t gps_uart_init(void) {
    ESP_LOGD(TAG, "Initializing GPS UART...");
    
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
    
    ESP_LOGD(TAG, "GPS UART initialized successfully");
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

// Configure GPS module for UBX-only mode
esp_err_t gps_configure_module(void) {
    ESP_LOGI(TAG, "Configuring GPS for UBX mode...");

    uint8_t packet_buffer[64];
    uint16_t packet_len;
    bool success = true;

    // 1. Configure UART port to UBX output only (CFG-PRT)
    uint8_t cfg_prt_payload[] = {
        0x01,               // Port ID (UART1)
        0x00,               // Reserved
        0x00, 0x00,         // TX Ready
        0xD0, 0x08, 0x00, 0x00, // Mode: 8N1
        0x80, 0x25, 0x00, 0x00, // Baud rate: 9600
        0x01, 0x00,         // Input protocols: UBX only
        0x01, 0x00,         // Output protocols: UBX only
        0x00, 0x00,         // Flags
        0x00, 0x00          // Reserved
    };

    packet_len = ubx_create_packet(0x06, 0x00, cfg_prt_payload, sizeof(cfg_prt_payload), packet_buffer);
    if (!ubx_send_config_command(packet_buffer, packet_len)) {
        ESP_LOGW(TAG, "UBX-CFG-PRT command failed");
        success = false;
    } else {
        ESP_LOGI(TAG, "✓ Switched to UBX protocol");
    }

    // 2. Enable UBX-NAV-PVT messages at 1Hz rate (CFG-MSG)
    uint8_t cfg_msg_payload[] = {
        0x01, 0x07,         // Message class and ID (NAV-PVT)
        0x01                // Rate: 1 message per navigation solution
    };

    packet_len = ubx_create_packet(0x06, 0x01, cfg_msg_payload, sizeof(cfg_msg_payload), packet_buffer);
    if (!ubx_send_config_command(packet_buffer, packet_len)) {
        ESP_LOGW(TAG, "UBX-CFG-MSG command failed");
        success = false;
    } else {
        ESP_LOGI(TAG, "✓ Enabled NAV-PVT messages");
    }

    // 3. Set navigation rate to 1Hz (CFG-RATE)
    uint8_t cfg_rate_payload[] = {
        0xE8, 0x03,         // Measurement rate: 1000ms = 1Hz
        0x01, 0x00,         // Navigation rate: 1 cycle
        0x01, 0x00          // Time reference: GPS time
    };

    packet_len = ubx_create_packet(0x06, 0x08, cfg_rate_payload, sizeof(cfg_rate_payload), packet_buffer);
    if (!ubx_send_config_command(packet_buffer, packet_len)) {
        ESP_LOGW(TAG, "UBX-CFG-RATE command failed");
        success = false;
    } else {
        ESP_LOGI(TAG, "✓ Set 1Hz update rate");
    }

    if (success) {
        ESP_LOGI(TAG, "GPS UBX configuration complete");
    } else {
        ESP_LOGW(TAG, "GPS UBX configuration partial - some commands failed");
    }

    return ESP_OK;
}

esp_err_t gps_init(void) {
    ESP_LOGD(TAG, "Initializing GPS module...");
    
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


// Read raw UART data with timeout and connection monitoring
static esp_err_t gps_read_uart_data(uint8_t *buffer, size_t buffer_size, int *bytes_read) {
    static uint32_t last_data_time = 0;

    *bytes_read = uart_read_bytes(GPS_UART_NUM, buffer, buffer_size, pdMS_TO_TICKS(GPS_TIMEOUT_MS));

    if (*bytes_read <= 0) {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (current_time - last_data_time > GPS_DATA_TIMEOUT_MS) {
            ESP_LOGW(TAG, "No GPS data received - check connections");
            last_data_time = current_time;
        }
        return ESP_ERR_NOT_FOUND;
    }

    // Update last data time and add debug logging
    last_data_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    static int debug_counter = 0;
    if (++debug_counter >= GPS_DEBUG_LOG_INTERVAL) {
        ESP_LOGD(TAG, "Raw GPS data (%d bytes): [UBX binary]", *bytes_read);
        debug_counter = 0;
    }

    return ESP_OK;
}

// Parse UART buffer for UBX packets
static void gps_parse_ubx_buffer(const uint8_t *buffer, int buffer_len) {
    static uint8_t ubx_buffer[512];
    static int ubx_pos = 0;
    static bool in_packet = false;
    static uint16_t expected_length = 0;
    static bool first_nav_pvt_logged = false;

    for (int i = 0; i < buffer_len; i++) {
        uint8_t byte = buffer[i];

        if (!in_packet) {
            // Look for UBX sync bytes (0xB5, 0x62)
            if (ubx_pos == 0 && byte == 0xB5) {
                ubx_buffer[ubx_pos++] = byte;
            } else if (ubx_pos == 1 && byte == 0x62) {
                ubx_buffer[ubx_pos++] = byte;
                in_packet = true;
            } else {
                ubx_pos = 0; // Reset if not sync sequence
            }
        } else {
            // Collecting packet data
            ubx_buffer[ubx_pos++] = byte;

            // Check if we have header complete (6 bytes)
            if (ubx_pos == 6) {
                expected_length = (ubx_buffer[5] << 8) | ubx_buffer[4]; // Little endian length
                expected_length += 8; // Header (6) + checksum (2)
            }

            // Check if packet is complete
            if (ubx_pos >= 8 && ubx_pos >= expected_length) {
                uint8_t msg_class, msg_id;
                uint16_t payload_length;

                if (ubx_parse_header(ubx_buffer, &msg_class, &msg_id, &payload_length)) {
                    // Check for NAV-PVT message (0x01, 0x07)
                    if (msg_class == 0x01 && msg_id == 0x07 && payload_length == 92) {
                        parse_ubx_nav_pvt(&ubx_buffer[6]); // Skip header, point to payload

                        // Set timestamp for health data
                        gps_health.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

                        if (!first_nav_pvt_logged) {
                            ESP_LOGI(TAG, "✓ UBX-NAV-PVT message received - UBX mode active");
                            first_nav_pvt_logged = true;
                        }
                    }
                }

                // Reset for next packet
                ubx_pos = 0;
                in_packet = false;
                expected_length = 0;
            }

            // Prevent buffer overflow
            if (ubx_pos >= sizeof(ubx_buffer)) {
                ESP_LOGW(TAG, "UBX buffer overflow, resetting");
                ubx_pos = 0;
                in_packet = false;
            }
        }
    }
}

// Main GPS read function - UBX packet processing
esp_err_t gps_read(gps_data_t *out_data) {
    if (out_data == NULL) {
        ESP_LOGE(TAG, "GPS output data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t rx_buffer[GPS_UART_BUF_SIZE];
    int bytes_read;

    // Read UART data
    esp_err_t err = gps_read_uart_data(rx_buffer, sizeof(rx_buffer), &bytes_read);
    if (err != ESP_OK) {
        return err;
    }

    // Parse UBX packets from buffer
    gps_parse_ubx_buffer(rx_buffer, bytes_read);

    // Copy parsed data to caller
    *out_data = gps_data;
    return ESP_OK;
}

// Read GPS health data
esp_err_t gps_read_health(gps_health_t *out_health) {
    if (out_health == NULL) {
        ESP_LOGE(TAG, "GPS health output data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Copy health data to caller
    *out_health = gps_health;
    return ESP_OK;
}

// Simplified GPS debug function for UBX data
void gps_debug_raw_data(void) {
    ESP_LOGI(TAG, "GPS Debug: Reading 3 UBX samples...");
    uint8_t buffer[256];

    for (int i = 0; i < 3; i++) {
        int len = uart_read_bytes(GPS_UART_NUM, buffer, sizeof(buffer), pdMS_TO_TICKS(1000));
        if (len > 0) {
            // Show hex dump of UBX data
            ESP_LOGI(TAG, "Sample %d (%d bytes): UBX binary data", i+1, len);

            // Look for UBX sync bytes and log packet info
            for (int j = 0; j < len - 5; j++) {
                if (buffer[j] == 0xB5 && buffer[j+1] == 0x62) {
                    uint8_t class = buffer[j+2];
                    uint8_t id = buffer[j+3];
                    uint16_t length = (buffer[j+5] << 8) | buffer[j+4];
                    ESP_LOGI(TAG, "  UBX packet: Class=0x%02X, ID=0x%02X, Length=%d", class, id, length);
                }
            }
        } else {
            ESP_LOGI(TAG, "Sample %d: No data", i+1);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}