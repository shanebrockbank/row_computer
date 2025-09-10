#ifndef GYRO_H
#define GYRO_H

esp_err_t gyro_init(void);
esp_err_t gyro_read(float *x, float *y, float *z);

#endif