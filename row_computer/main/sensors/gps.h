#ifndef GPS_H
#define GPS_H

esp_err_t gps_init(void);
esp_err_t gps_read(float *lat, float *lon, float *spd);

#endif