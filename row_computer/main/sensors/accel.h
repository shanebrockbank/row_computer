#ifndef ACCEL_H
#define ACCEL_H

esp_err_t accel_init(void);
esp_err_t accel_read(float *x, float *y, float *z);

#endif