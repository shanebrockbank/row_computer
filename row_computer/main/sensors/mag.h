#ifndef MAG_H
#define MAG_H

esp_err_t mag_init(void);
esp_err_t mag_read(float *x, float *y, float *z);

#endif