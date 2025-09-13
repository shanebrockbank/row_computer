#ifndef GPS_H
#define GPS_H

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

esp_err_t gps_init(void);
esp_err_t gps_read(gps_data_t *gps_data);
void uart_loopback_test(void);

#endif