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

typedef struct {
    uint32_t horizontal_accuracy;  // mm - position accuracy estimate
    uint32_t speed_accuracy;       // mm/s - speed accuracy estimate
    uint8_t fix_type;              // 0=no fix, 2=2D, 3=3D, 4=GNSS+DR
    uint8_t satellites;            // Number of satellites (duplicated for convenience)
    uint32_t timestamp_ms;         // When this health data was captured
} gps_health_t;

// Main GPS functions
esp_err_t gps_init(void);
esp_err_t gps_read(gps_data_t *gps_data);
esp_err_t gps_read_health(gps_health_t *gps_health);

// Debug functions
esp_err_t gps_test_communication(void);
void gps_debug_raw_data(void);

// Internal functions (can be used for testing)
esp_err_t gps_uart_init(void);
esp_err_t gps_configure_module(void);

#endif