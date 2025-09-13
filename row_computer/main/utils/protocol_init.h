#ifndef PROTOCOL_INIT_H
#define PROTOCOL_INIT_H

#include "esp_err.h"

/**
 * @brief Initialize all communication protocols
 * Call this ONCE during system startup before any sensor init
 */
esp_err_t protocols_init(void);

/**
 * @brief Deinitialize all communication protocols
 */
esp_err_t protocols_deinit(void);

/**
 * @brief Individual protocol init functions (can be called separately if needed)
 */
esp_err_t i2c_master_init(void);

void test_i2c_bus(void);

esp_err_t uart_gps_init(void);
esp_err_t spi_master_init(void);

#endif // PROTOCOL_INIT_H