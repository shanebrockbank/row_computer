#ifndef GPS_H
#define GPS_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    char time[16];
    double latitude;
    double longitude;
    float speed_knots;
    float heading;
    int satellites;
    bool valid_fix;
    uint32_t timestamp_ms;
} gps_data_t;

// Main GPS functions
esp_err_t gps_init(void);
esp_err_t gps_read(gps_data_t *gps_data);

// Debug functions
esp_err_t gps_test_communication(void);
void gps_debug_raw_data(void);

// Internal functions (can be used for testing)
esp_err_t gps_uart_init(void);
esp_err_t gps_configure_module(void);
double nmea_to_decimal(const char *coord, const char *direction);

#endif