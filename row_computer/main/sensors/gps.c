#include "esp_log.h"
#include "driver/uart.h"
#include "config/pin_definitions.h"
#include "gps.h"
#include "esp_mac.h"


#include <string.h>

static const char *TAG = "GPS";

// UBX command to enable GGA
static const uint8_t UBX_ENABLE_GGA[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
    0xF0, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x17, 0x0A
};

// UBX command to enable RMC
static const uint8_t UBX_ENABLE_RMC[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
    0xF0, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x21, 0x0D
};

esp_err_t gps_enable_nmea(void) {
    ESP_LOGI(TAG, "Sending UBX commands to enable GGA/RMC");
    uart_write_bytes(GPS_UART_NUM, (const char*)UBX_ENABLE_GGA, sizeof(UBX_ENABLE_GGA));
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_write_bytes(GPS_UART_NUM, (const char*)UBX_ENABLE_RMC, sizeof(UBX_ENABLE_RMC));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "UBX commands sent");
    return ESP_OK;
}

esp_err_t gps_init(void) {    
    ESP_LOGI(TAG, "GPS UART initialized on pins TX=%d, RX=%d at %d baud", 
             GPS_UART_TXD_PIN, GPS_UART_RXD_PIN, GPS_UART_BAUD_RATE);

    //gps_enable_nmea();
    
    return ESP_OK;
}

static gps_data_t gps_data = {0};

//TODO add in actual logging to the retun 0.0 lines
// Convert NMEA coordinate to decimal degrees
double nmea_to_decimal(const char *coord, const char *direction) {
    if (!coord || strlen(coord) == 0) return 0.0;
    
    double val = atof(coord);
    // Validate coordinate range before processing
    // coord = "4807.038" (48°07.038'), val = 4807.038
    if (val < 0 || val > 18000) return 0.0; // Basic sanity check
    
    int degrees = (int)(val / 100); // degrees = 48
    double minutes = val - (degrees * 100); // minutes = 7.038
    
    // Validate minutes range 
    if (minutes >= 60.0) return 0.0;
    
    double decimal = degrees + (minutes / 60.0); // 48 + (7.038/60) = 48.1173°
    
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

    if (i >= 2) {
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

    if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0)
        parse_rmc(buffer);
    else if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0)
        parse_gga(buffer);
}

esp_err_t gps_read(gps_data_t *out_data) {
    static char rx_buffer[GPS_UART_BUF_SIZE];
    static char nmea_line[MAX_NMEA_LEN];
    static int line_pos = 0;

    int len = uart_read_bytes(GPS_UART_NUM, (uint8_t*)rx_buffer,
                              sizeof(rx_buffer)-1, 50 / portTICK_PERIOD_MS);
    if (len <= 0) {
        return ESP_ERR_NOT_FOUND;
    }

    for (int i = 0; i < len; i++) {
        char c = rx_buffer[i];
        if (c == '\n' || c == '\r') {
            if (line_pos > 0) {
                nmea_line[line_pos] = '\0';
                process_nmea_sentence(nmea_line); // updates internal gps_data
                line_pos = 0;
            }
        } else if (line_pos < MAX_NMEA_LEN-1) {
            nmea_line[line_pos++] = c;
        } else {
            // overflow, reset
            line_pos = 0;
        }
    }

    // Copy internal gps_data to caller
    out_data->latitude    = gps_data.latitude;
    out_data->longitude   = gps_data.longitude;
    out_data->speed_knots = gps_data.speed_knots;
    out_data->heading     = gps_data.heading;
    out_data->satellites  = gps_data.satellites;
    out_data->valid_fix   = gps_data.valid_fix;
    strncpy(out_data->time, gps_data.time, sizeof(out_data->time)-1);
    out_data->time[sizeof(out_data->time)-1] = '\0';

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

// Read GPS from UART
// esp_err_t gps_read(gps_data_t *gps_data) {
//     static char rx_buffer[GPS_UART_BUF_SIZE];
//     int len = uart_read_bytes(GPS_UART_NUM, (uint8_t*)rx_buffer,
//                               sizeof(rx_buffer)-1, 100 / portTICK_PERIOD_MS);

//     if (len <= 0) {
//         gps_data->latitude = 0.0;
//         gps_data->longitude = 0.0;
//         gps_data->speed_knots = 0.0f;
//         return ESP_ERR_NOT_FOUND;               
//     }
//     rx_buffer[len] = '\0';

//     // Split by line endings
//     char *line = strtok(rx_buffer, "\r\n");
//     while (line) {
//         process_nmea_sentence(line);
//         line = strtok(NULL, "\r\n");
//     }

//     return ESP_OK;
// }

